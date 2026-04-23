import sys
from PyQt5.QtWidgets import (QMainWindow, QVBoxLayout, QTableWidgetItem, 
                            QMessageBox, QGraphicsScene, QApplication, 
                            QHeaderView, QFileDialog, QTreeWidgetItem)
from PyQt5 import QtWidgets, QtGui
import PyQt5.QtCore as QtCore
from PyQt5.QtCore import QTimer, QCoreApplication, pyqtSignal, QObject, QDir, QSettings
from PyQt5.QtGui import QImage, QPixmap, QFont, QCursor
from info_window import Ui_MainWindow as main_window
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FC
from matplotlib.backends.backend_qt5 import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import pickle
import numpy as np
import zipfile
import tempfile
import pandas as pd
import os
from scipy.optimize import curve_fit
from scipy.interpolate import make_interp_spline
import copy
from numpy import array, float64, arange
import traceback
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit, least_squares
from scipy.stats import chi2
import warnings
from enum import Enum
import inspect
from typing import Optional, List

# 常量定义
DEFAULT_FIT_OPTIONS = {
    'DiffMinChange': 1e-8,
    'DiffMaxChange': 0.1,
    'MaxIter': 400,
    'MaxFunEvals': 600,
    'TolFun': 1e-6,
    'TolX': 1e-6
}

UNIT_MAPPING = {'s': 1, 'ms': 1000, 'μs': 1000000, 'ns': 1000000000}

class RobustMethod(Enum):
    OFF = 'Off'
    LAR = 'LAR'
    BISQUARE = 'Bisquare'

class Algorithm(Enum):
    TRUST_REGION = 'Trust-Region'
    LEVENBERG_MARQUARDT = 'Levenberg-Marquardt'

class MATLABCurveFitOptions:
    """完全对应MATLAB的fitoptions参数"""
    def __init__(self):
        # 稳健性选项 (MATLAB: 'Robust')
        self.Robust = RobustMethod.OFF
        
        # 算法选项 (MATLAB: 'Algorithm')
        self.Algorithm = Algorithm.LEVENBERG_MARQUARDT
        
        # 差分步长设置 (MATLAB: 'DiffMinChange', 'DiffMaxChange')
        self.DiffMinChange = 1e-8
        self.DiffMaxChange = 0.1
        
        # 迭代控制 (MATLAB: 'MaxIter', 'MaxFunEvals')
        self.MaxIter = 400
        self.MaxFunEvals = 600
        
        # 收敛容差 (MATLAB: 'TolFun', 'TolX')
        self.TolFun = 1e-6
        self.TolX = 1e-6
        
        # 参数边界 (MATLAB: 'Lower', 'Upper')
        self.Lower = -np.inf
        self.Upper = np.inf
        
        # 起始点 (MATLAB: 'StartPoint')
        self.StartPoint = None
        
        # 权重 (MATLAB: 'Weights')
        self.Weights = None
        
        # 排除点 (MATLAB: 'Exclude')
        self.Exclude = None

class MATLABFitResult:
    """完全对应MATLAB的fitresult输出"""
    def __init__(self):
        # 基本拟合结果
        self.success = False
        self.message = ""
        self.iterations = 0
        self.funcCount = 0
        self.algorithm = ""
        
        # 参数估计
        self.params = None
        self.param_names = None
        
        # 拟合统计
        self.sse = 0.0  # 误差平方和
        self.rsquare = 0.0  # R²
        self.dfe = 0  # 误差自由度
        self.adjrsquare = 0.0  # 调整R²
        self.rmse = 0.0  # 均方根误差
        
        # 残差分析
        self.residuals = None
        
        # 雅可比矩阵 (用于误差分析)
        self.jacobian = None
        
    def __str__(self):
        """类似MATLAB的输出格式"""
        output = []
        output.append("拟合结果:")
        output.append(f"     算法: {self.algorithm}")
        output.append(f"     收敛: {'是' if self.success else '否'}")
        output.append(f"     迭代次数: {self.iterations}")
        output.append(f"     函数计算次数: {self.funcCount}")
        output.append("")
        output.append("系数估计 (95% 置信区间):")
        
        for i, (name, value) in enumerate(zip(self.param_names, self.params)):
            output.append(f"     {name} = {value:.6f}")
        
        output.append("")
        output.append("拟合优度统计:")
        output.append(f"     SSE: {self.sse:.6f}")
        output.append(f"     R²: {self.rsquare:.6f}")
        output.append(f"     调整R²: {self.adjrsquare:.6f}")
        output.append(f"     RMSE: {self.rmse:.6f}")
        
        return "\n".join(output)

class MATLABCurveFitter:
    """完全复现MATLAB的fit函数功能"""
    
    def __init__(self):
        self.options = MATLABCurveFitOptions()
        self.cancel_requested = None
    
    def set_options(self, **kwargs):
        """设置拟合选项，参数名与MATLAB一致"""
        valid_options = ['Robust', 'Algorithm', 'DiffMinChange', 'DiffMaxChange', 
                        'MaxIter', 'MaxFunEvals', 'TolFun', 'TolX', 'Lower', 'Upper', 
                        'StartPoint', 'Weights', 'Exclude']
        
        for key, value in kwargs.items():
            if hasattr(self.options, key):
                setattr(self.options, key, value)
            else:
                warnings.warn(f"MATLAB中不支持的选项: {key}")
    
    def _robust_weight_function(self, residuals, method):
        """MATLAB稳健权重函数"""
        if method == RobustMethod.OFF:
            return np.ones_like(residuals)
        
        # 计算标准化残差
        mad = np.median(np.abs(residuals - np.median(residuals)))
        if mad == 0:
            mad = np.mean(np.abs(residuals))
        u = residuals / (1.4826 * mad)  # 一致性常数
        
        if method == RobustMethod.LAR:
            # LAR (最小绝对残差) - MATLAB实现
            weights = np.ones_like(u)
            mask = np.abs(u) > 1e-8
            weights[mask] = 1.0 / np.abs(u[mask])
            
        elif method == RobustMethod.BISQUARE:
            # Bisquare (Tukey's biweight) - MATLAB实现
            weights = np.zeros_like(u)
            mask = np.abs(u) < 4.685  # MATLAB默认调谐常数
            weights[mask] = (1 - (u[mask]/4.685)**2)**2
        
        return weights
    
    def _apply_exclude(self, x, y, exclude):
        """应用排除点"""
        if exclude is None:
            return x, y, np.ones_like(x, dtype=bool)
        
        if callable(exclude):
            include_mask = ~exclude(x, y)
        else:
            include_mask = ~exclude
            
        return x[include_mask], y[include_mask], include_mask
    
    def _apply_weights(self, residuals, weights):
        """应用权重"""
        if weights is None:
            return residuals
        return residuals * np.sqrt(weights)
    
    def _calculate_goodness_of_fit(self, y, y_pred, residuals, n_params, weights=None):
        """计算拟合优度统计量 (MATLAB方法)"""
        n = len(y)
        
        # 加权残差
        if weights is not None:
            weighted_residuals = residuals * np.sqrt(weights)
            ss_res = np.sum(weighted_residuals**2)
        else:
            ss_res = np.sum(residuals**2)
        
        # 加权总平方和
        if weights is not None:
            y_weighted = y * np.sqrt(weights)
            y_mean_weighted = np.average(y, weights=weights)
            ss_tot = np.sum((y_weighted - y_mean_weighted)**2)
        else:
            ss_tot = np.sum((y - np.mean(y))**2)
        
        # 计算统计量
        sse = ss_res
        rsquare = 1 - ss_res/ss_tot if ss_tot != 0 else 0
        dfe = n - n_params  # 误差自由度
        adjrsquare = 1 - (1 - rsquare) * (n - 1) / dfe if dfe > 0 else rsquare
        rmse = np.sqrt(sse / n)
        
        return sse, rsquare, dfe, adjrsquare, rmse
    
    def fit_curve(self, x, y, model_type, initial_guess=None, param_names=None):
        """
        拟合预定义模型 (完全复现MATLAB fit函数)
        
        参数:
            x: 自变量数据
            y: 因变量数据
            model_type: 模型类型 ('linear', 'exponential', 'polynomial2', 'polynomial3', 
                                'power', 'logarithmic', 'gaussian', 'sine')
            initial_guess: 参数初始值
            param_names: 参数名称
            
        返回:
            MATLABFitResult对象，包含所有MATLAB输出
        """
        
        # 获取模型定义
        model_def = self._get_model_definition(model_type)
        if model_def is None:
            raise ValueError(f"不支持的模型类型: {model_type}")
        
        func = model_def['function']
        default_param_names = model_def['param_names']
        n_params = len(default_param_names)

        # 设置参数名称
        if param_names is None:
            param_names = default_param_names
        
        # 设置初始猜测
        if initial_guess is None:
            if self.options.StartPoint is not None:
                initial_guess = self.options.StartPoint
            else:
                initial_guess = self._get_default_start_point(model_type, x, y)
        
        # 应用排除点
        x_fit, y_fit, include_mask = self._apply_exclude(x, y, self.options.Exclude)
        
        # 记录当前模型类型，供内部优化逻辑使用
        self.current_model_type = model_type

        # 执行拟合
        result = self._fit_implementation(x_fit, y_fit, func, initial_guess, param_names)


        # 计算完整数据的残差（仅在拟合成功时）
        if result.params is not None:
            y_pred_full = func(x, *result.params)
            result.residuals = y - y_pred_full
        
        return result
    
    
    def _get_model_definition(self, model_type):
        """修改双指数模型定义"""
        models = {
            'linear': {
                'function': lambda x, a, b: a * x + b,
                'param_names': ['a', 'b'],
                'description': 'y = a*x + b'
            },
            '单指数': {
                'function': lambda x, a, b, c: a * np.exp(-x/b) + c,  # 修改为 -x/b
                'param_names': ['a', 'b', 'c'],
                'description': 'y = a*exp(-x/b) + c'
            },
            '双指数': {
                'function': lambda x, a, t1, c, t2, e: a * np.exp(-x/t1) + c * np.exp(-x/t2) + e,
                'param_names': ['a', 't1', 'c', 't2', 'e'],
                'description': 'y = a*exp(-x/t1) + c*exp(-x/t2) + e'
            },
            'polynomial2': {
                'function': lambda x, a, b, c: a * x**2 + b * x + c,
                'param_names': ['a', 'b', 'c'],
                'description': 'y = a*x² + b*x + c'
            },
        }
        return models.get(model_type)

    def _get_default_start_point(self, model_type, x, y):
        """优化初始参数计算 - 更智能的初始值估计"""
        if model_type == '双指数':
            y_range = np.max(y) - np.min(y)
            if y_range == 0:
                y_range = 1.0

            # 使用数据起点的一段距离来估计I0（初始强度）
            # 取前10%的数据点来估计初始值，避免噪声影响
            start_fraction = 0.1  # 使用前10%的数据
            n_start_points = max(5, int(len(x) * start_fraction))  # 至少5个点
            
            if n_start_points < len(x):
                x_start = x[:n_start_points]
                y_start = y[:n_start_points]
                
                # 对于荧光衰减数据，基线I0应该是衰减结束后的稳定水平
                # 使用末尾一段数据的平均值作为基线，避免噪声影响
                end_section = max(10, len(y)//20)  # 至少10个点，或数据长度的5%，取较大者
                baseline_est = np.mean(y[-end_section:])  # 基线取末尾一段的平均值
                initial_intensity = np.max(y_start) - baseline_est
                
                # 确保初始强度为正值且合理
                initial_intensity = max(initial_intensity, y_range * 0.1)
                I0_estimate = baseline_est  # I0是基线参数
            else:
                # 如果数据点太少，使用传统方法
                initial_intensity = y_range * 0.7
                I0_estimate = np.min(y)

            # 改进的时间常数估计方法
            # 方法1：基于数据特征的衰减时间估计
            y_normalized = (y - np.min(y)) / y_range

            # 找到衰减到不同百分比的点
            decay_levels = [0.9, 0.5, 0.1]  # 90%, 50%, 10% 衰减点
            decay_times = []

            for level in decay_levels:
                # 找到最接近该衰减水平的点
                idx = np.argmin(np.abs(y_normalized - level))
                if idx < len(x) - 1:  # 确保不是最后一个点
                    decay_times.append(x[idx])

            # 如果找到的点不够，用默认值补充
            while len(decay_times) < 3:
                decay_times.append(x[len(x)//(len(decay_times)+1)])

            # 排序确保时间顺序
            decay_times.sort()

            # 计算时间常数 - 使用更稳定的方法
            # t1: 快速分量 (从90%到50%衰减的时间)
            if len(decay_times) >= 2:
                t1_guess = (decay_times[1] - decay_times[0]) / np.log(2)  # 半衰期
            else:
                t1_guess = x[len(x)//4] / 2

            # t2: 慢速分量 (从50%到10%衰减的时间)
            if len(decay_times) >= 3:
                t2_guess = (decay_times[2] - decay_times[1]) / (np.log(10) - np.log(2))  # 从50%到10%的衰减时间
            else:
                t2_guess = x[len(x)//2] / 3

            # 确保 t1 < t2（快速分量衰减更快）
            if t1_guess >= t2_guess:
                t1_guess, t2_guess = t2_guess, t1_guess
                # 如果仍然不满足，设置合理的比例
                if t1_guess >= t2_guess:
                    t1_guess = t2_guess / 3

            # 确保时间常数在合理范围内 - 使用更智能的边界
            # 基于数据的时间范围和采样率来设置合理的边界
            dt = np.mean(np.diff(x)) if len(x) > 1 else x[0]  # 平均时间间隔
            min_tau = max(dt * 0.01, 1e-12)  # 最小时间常数至少是时间间隔的0.01倍，更小的下限
            max_tau = x[-1] * 100  # 最大时间常数是数据总长的100倍，更大的上限

            t1_guess = np.clip(t1_guess, min_tau, max_tau)
            t2_guess = np.clip(t2_guess, t1_guess * 1.1, max_tau)

            # 振幅分配 - 基于I0估计重新分配
            # 快速分量通常占较大比例，但要确保总和不超过初始强度
            # 使用更保守的比例来避免边界问题
            a_guess = initial_intensity * 0.5  # 快速分量振幅 - 降低比例
            c_guess = initial_intensity * 0.2  # 慢速分量振幅 - 降低比例
            e_guess = I0_estimate               # 基线

            # 确保振幅不为负
            a_guess = max(a_guess, 0)
            c_guess = max(c_guess, 0)

            return [a_guess, t1_guess, c_guess, t2_guess, e_guess]

        elif model_type == '单指数':
            # 单指数衰减模型的初始值计算 - 多方法融合
            y_range = np.max(y) - np.min(y)
            if y_range == 0:
                y_range = 1.0

            # 方法1：改进的线性回归法
            y_adj = y - np.min(y) + 1e-10
            valid_idx = y_adj > 0.05 * np.max(y_adj)  # 稍微严格的阈值

            tau_candidates = []
            amplitude_candidates = []

            if np.sum(valid_idx) > 5:
                # 使用加权线性回归
                log_y = np.log(y_adj[valid_idx])
                x_valid = x[valid_idx]

                # 尝试不同的权重方案
                weights = y_adj[valid_idx] / np.max(y_adj[valid_idx])
                try:
                    slope, intercept = np.polyfit(x_valid, log_y, 1, w=weights)
                    if slope < -1e-10:  # 确保有足够的衰减
                        tau_reg = -1.0 / slope
                        amplitude_reg = np.exp(intercept)
                        tau_candidates.append(tau_reg)
                        amplitude_candidates.append(amplitude_reg)
                except:
                    pass

            # 方法2：基于半衰期的估计
            try:
                y_half = (np.max(y) + np.min(y)) / 2
                half_idx = np.where(y <= y_half)[0]
                if len(half_idx) > 0:
                    t_half = x[half_idx[0]]
                    tau_half = t_half / np.log(2)  # τ = t_half / ln(2)
                    if tau_half > 0:
                        tau_candidates.append(tau_half)
                        amplitude_candidates.append(y_range * 0.9)
            except:
                pass

            # 方法3：基于数据范围的保守估计
            tau_range = x[len(x)//4] / np.log(2)  # 在1/4处衰减一半
            tau_candidates.append(tau_range)
            amplitude_candidates.append(y_range * 0.8)

            # 选择最合理的tau
            dt = np.mean(np.diff(x)) if len(x) > 1 else x[0]
            min_tau = max(dt * 0.01, 1e-12)
            x_span = abs(x[-1] - x[0]) if len(x) > 1 else max(abs(x[0]), 1.0)
            max_tau = x_span * 100  # 放宽上限，避免慢衰减单指数卡在边界

            # 过滤合理的tau值
            valid_tau = [t for t in tau_candidates if min_tau <= t <= max_tau]
            if valid_tau:
                tau = np.median(valid_tau)  # 使用中位数提高稳健性
            else:
                tau = x[len(x)//3] / np.log(2)

            # 选择对应的amplitude
            if valid_tau:
                tau_idx = tau_candidates.index(tau) if tau in tau_candidates else 0
                amplitude = amplitude_candidates[min(tau_idx, len(amplitude_candidates)-1)]
            else:
                amplitude = y_range * 0.7

            # 最终约束
            tau = np.clip(tau, min_tau, max_tau)
            amplitude = np.clip(amplitude, y_range * 0.05, y_range * 3)
            baseline = np.min(y)

            return [amplitude, tau, baseline]

        # 其他模型的默认初始值
        return [1.0] * len(self._get_model_definition(model_type)['param_names'])
        
        
    def _fit_implementation(self, x, y, func, initial_guess, param_names):
        """改进的拟合实现核心"""
        result = MATLABFitResult()
        result.param_names = param_names

        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        initial_guess = np.asarray(initial_guess, dtype=float)
        x_original = x.copy()

        # 对单指数进行x平移，提升数值稳定性
        single_exp_model = (
            len(param_names) == 3
            and hasattr(self, 'current_model_type')
            and self.current_model_type == '单指数'
        )
        double_exp_model = (
            len(param_names) == 5
            and hasattr(self, 'current_model_type')
            and self.current_model_type == '双指数'
        )
        x_offset = float(np.min(x)) if (single_exp_model and len(x) > 0) else 0.0
        if single_exp_model:
            x = x - x_offset

        # 初始化边界约束
        bounds_lower = np.full_like(initial_guess, -np.inf, dtype=float)
        bounds_upper = np.full_like(initial_guess, np.inf, dtype=float)
        
        # 应用用户设置的约束
        if self.options.Lower is not None:
            bounds_lower = np.maximum(bounds_lower, self.options.Lower)
        if self.options.Upper is not None:
            bounds_upper = np.minimum(bounds_upper, self.options.Upper)
        
        # 设置参数边界 (特别是对指数衰减模型)
        is_exp_model = any(name in ['b', 'tau', 't1', 't2', 'd'] for name in param_names)
        if is_exp_model:
            # 对衰减率参数设置合理边界
            for i, name in enumerate(param_names):
                if name in ['t1', 't2', 'b', 'd']:  # 衰减率参数
                    x_span = abs(x[-1] - x[0]) if len(x) > 1 else max(abs(x[0]), 1.0)
                    dt = abs(np.mean(np.diff(x))) if len(x) > 1 else max(abs(x[0]), 1e-9)
                    min_tau = max(dt * 0.01, 1e-12)
                    max_tau = max(x_span * 100, min_tau * 10)
                    bounds_lower[i] = max(bounds_lower[i], min_tau)
                    bounds_upper[i] = min(bounds_upper[i], max_tau)
                elif name in ['a', 'c']:  # 振幅参数
                    bounds_lower[i] = max(bounds_lower[i], 0)  # 振幅必须非负

            # 特殊处理单指数模型
            if len(param_names) == 3 and param_names[1] in ['b', 'tau']:  # 单指数模型
                tau_idx = 1
                amp_idx = 0
                baseline_idx = 2

                # 单指数的tau边界更严格
                dt = np.mean(np.diff(x)) if len(x) > 1 else x[0] if len(x) > 0 else 1e-9
                bounds_lower[tau_idx] = max(bounds_lower[tau_idx], dt * 0.01)  # 最小tau
                x_span = abs(x[-1] - x[0]) if len(x) > 1 else max(abs(x[0]), 1.0)
                bounds_upper[tau_idx] = min(bounds_upper[tau_idx], x_span * 100)  # 最大tau

                # 振幅边界基于数据范围
                y_range = np.max(y) - np.min(y)
                if y_range <= 0:
                    y_range = max(abs(np.max(y)), 1.0)
                bounds_lower[amp_idx] = max(bounds_lower[amp_idx], y_range * 0.01)
                bounds_upper[amp_idx] = min(bounds_upper[amp_idx], y_range * 5)

                # 基线边界
                bounds_lower[baseline_idx] = max(bounds_lower[baseline_idx], np.min(y) - y_range * 0.1)
                bounds_upper[baseline_idx] = min(bounds_upper[baseline_idx], np.max(y))

            # 如果是双指数模型，添加 t1 < t2 的约束
            if len(param_names) >= 5 and 't1' in param_names and 't2' in param_names:
                t1_idx = param_names.index('t1')
                t2_idx = param_names.index('t2')
                # 确保 t1 < t2 的边界约束
                bounds_upper[t1_idx] = min(bounds_upper[t1_idx], bounds_upper[t2_idx])  # t1 的上限不超过 t2 的上限

        # 修正非法边界并将初值投影到边界内部
        invalid_bounds = bounds_lower >= bounds_upper
        if np.any(invalid_bounds):
            eps = 1e-12
            bounds_upper[invalid_bounds] = bounds_lower[invalid_bounds] + eps
        initial_guess = np.clip(initial_guess, bounds_lower + 1e-14, bounds_upper - 1e-14)

        # 设置优化选项
        method = 'trf' if self.options.Algorithm == Algorithm.TRUST_REGION else 'lm'
        has_finite_bounds = np.any(np.isfinite(bounds_lower)) or np.any(np.isfinite(bounds_upper))
        needs_robust = self.options.Robust != RobustMethod.OFF
        if method == 'lm' and (has_finite_bounds or needs_robust):
            method = 'trf'

        lsq_options = {
            'method': method,
            'max_nfev': self.options.MaxFunEvals,
            'ftol': self.options.TolFun,
            'xtol': self.options.TolX,
            'gtol': self.options.TolX,
            'x_scale': 'jac',
            'verbose': 0,
        }

        if method != 'lm':
            lsq_options['bounds'] = (bounds_lower, bounds_upper)
            # 参数维度很小(3/5)，exact 通常比 lsmr 更快且稳定
            lsq_options['tr_solver'] = 'exact'
            lsq_options['loss'] = 'soft_l1' if needs_robust else 'linear'
        
        try:
            # 定义目标函数
            def objective(params):
                if callable(self.cancel_requested) and self.cancel_requested():
                    raise InterruptedError("用户取消拟合")

                fit_params = params
                # 双指数采用平滑重排，避免 t1<t2 硬惩罚导致的非光滑目标
                if double_exp_model and len(params) == 5:
                    a0, t1_0, c0, t2_0, e0 = params
                    if t1_0 > t2_0:
                        fit_params = np.array([c0, t2_0, a0, t1_0, e0], dtype=float)

                y_pred = func(x, *fit_params)
                if not np.all(np.isfinite(y_pred)):
                    return np.full_like(y, 1e10, dtype=float)
                residuals = y - y_pred
                residuals = np.where(np.isfinite(residuals), residuals, 1e10)

                # 应用稳健权重
                if self.options.Robust != RobustMethod.OFF:
                    weights = self._robust_weight_function(residuals, self.options.Robust)
                    residuals = residuals * np.sqrt(weights)
                
                # 应用用户权重
                if self.options.Weights is not None:
                    residuals = residuals * np.sqrt(self.options.Weights)
                
                return residuals
            
            # 执行拟合 - 分阶段优化策略
            print(f"初始参数: {initial_guess}")

            # 第一阶段: 使用较宽容容差快速收敛
            phase1_options = lsq_options.copy()
            phase1_options['ftol'] = max(self.options.TolFun, 1e-6)
            phase1_options['xtol'] = max(self.options.TolX, 1e-6)
            phase1_options['gtol'] = max(self.options.TolX, 1e-6)
            phase1_trial_options = phase1_options.copy()
            # 多起点试探阶段：降低单次预算以提速，最终仍由第二阶段精修
            phase1_trial_options['max_nfev'] = max(60, min(120, max(80, self.options.MaxFunEvals // 6)))
            
            # 单指数：采用与双指数一致的多起点 + 局部精修流程
            if len(initial_guess) == 3 and hasattr(self, 'current_model_type') and self.current_model_type == '单指数':
                y_range = np.max(y) - np.min(y)
                if y_range <= 0:
                    y_range = max(abs(np.max(y)), 1.0)
                x_range = abs(x[-1] - x[0]) if len(x) > 1 else max(abs(x[0]), 1.0)
                dt = abs(np.mean(np.diff(x))) if len(x) > 1 else max(abs(x[0]), 1e-9)
                tau_floor = max(dt * 0.01, 1e-12)
                tau_cap = max(x_range * 100, tau_floor * 10)

                a0, tau0, c0 = initial_guess
                tau0 = np.clip(max(tau0, tau_floor), tau_floor, tau_cap)
                a0 = np.clip(max(a0, y_range * 0.01), y_range * 0.01, y_range * 5)

                tail_n = max(8, len(y) // 10)
                y_tail = y[-tail_n:] if len(y) >= tail_n else y
                baseline_candidates = [np.min(y), np.quantile(y, 0.1), np.median(y_tail), np.mean(y_tail)]
                tau_scales = [0.25, 0.5, 1.0, 2.0, 4.0]
                amp_scales = [0.4, 0.7, 1.0, 1.4, 2.0]

                initial_guesses = [np.array([a0, tau0, c0], dtype=float)]

                # 基于初值做网格扩展
                for ts in tau_scales:
                    for aps in amp_scales:
                        for bg in baseline_candidates:
                            guess = np.array([a0 * aps, tau0 * ts, bg], dtype=float)
                            guess = np.clip(guess, bounds_lower + 1e-14, bounds_upper - 1e-14)
                            initial_guesses.append(guess)

                # 追加一组对数线性回归候选，增强单指数全局性
                y_baseline_ref = np.quantile(y_tail, 0.5)
                y_adj = y - y_baseline_ref
                valid = y_adj > max(np.max(y_adj) * 0.05, 1e-12)
                if np.sum(valid) > 5:
                    try:
                        slope, intercept = np.polyfit(x[valid], np.log(y_adj[valid]), 1)
                        if slope < -1e-12:
                            tau_log = np.clip(-1.0 / slope, tau_floor, tau_cap)
                            amp_log = np.clip(np.exp(intercept), y_range * 0.01, y_range * 5)
                            guess = np.array([amp_log, tau_log, y_baseline_ref], dtype=float)
                            guess = np.clip(guess, bounds_lower + 1e-14, bounds_upper - 1e-14)
                            initial_guesses.append(guess)
                    except Exception:
                        pass

                # 去重并限制数量，控制耗时
                uniq = []
                seen = set()
                for g in initial_guesses:
                    key = tuple(np.round(g, 10))
                    if key not in seen:
                        seen.add(key)
                        uniq.append(g)
                initial_guesses = uniq[:24]

                best_result = None
                best_cost = np.inf

                for guess in initial_guesses:
                    try:
                        phase1_result = least_squares(objective, guess, **phase1_trial_options)
                        if phase1_result.success and phase1_result.cost < best_cost:
                            best_result = phase1_result
                            best_cost = phase1_result.cost
                    except:
                        continue

                if best_result is not None:
                    # 对最佳候选再做一轮完整第一阶段精修
                    phase1_result = least_squares(objective, best_result.x, **phase1_options)
                else:
                    phase1_result = least_squares(objective, initial_guess, **phase1_options)
            elif double_exp_model:
                # 双指数也使用多起点，降低局部最优风险
                y_range = np.max(y) - np.min(y)
                if y_range <= 0:
                    y_range = max(abs(np.max(y)), 1.0)
                x_range = abs(x[-1] - x[0]) if len(x) > 1 else max(abs(x[0]), 1.0)
                dt = abs(np.mean(np.diff(x))) if len(x) > 1 else max(abs(x[0]), 1e-9)

                tau_floor = max(dt * 0.01, 1e-12)
                tau_cap = max(x_range * 100, tau_floor * 10)

                a0, t10, c0, t20, e0 = initial_guess
                total_amp = max(a0 + c0, y_range * 0.2)
                t10 = np.clip(max(t10, tau_floor), tau_floor, tau_cap)
                t20 = np.clip(max(t20, t10 * 1.1), tau_floor, tau_cap)

                tail_n = max(8, len(y) // 10)
                y_tail = y[-tail_n:] if len(y) >= tail_n else y
                baseline_candidates = [np.min(y), np.median(y_tail), np.mean(y_tail)]
                amp_splits = [(0.8, 0.2), (0.65, 0.35), (0.5, 0.5)]
                t1_scales = [0.5, 1.0, 2.0]
                t2_scales = [0.5, 1.0, 2.0]

                initial_guesses = [initial_guess.copy()]
                for s1 in t1_scales:
                    for s2 in t2_scales:
                        t1g = np.clip(t10 * s1, tau_floor, tau_cap)
                        t2g = np.clip(t20 * s2, tau_floor, tau_cap)
                        if t1g > t2g:
                            t1g, t2g = t2g, t1g
                        t2g = max(t2g, t1g * 1.1)
                        t2g = min(t2g, tau_cap)
                        if t2g <= t1g:
                            continue
                        for w1, w2 in amp_splits:
                            for bg in baseline_candidates:
                                guess = np.array([total_amp * w1, t1g, total_amp * w2, t2g, bg], dtype=float)
                                guess = np.clip(guess, bounds_lower + 1e-14, bounds_upper - 1e-14)
                                initial_guesses.append(guess)

                # 去重并限制数量，避免时间过长
                uniq = []
                seen = set()
                for g in initial_guesses:
                    key = tuple(np.round(g, 10))
                    if key not in seen:
                        seen.add(key)
                        uniq.append(g)
                # 保留代表性候选，减少试探次数
                initial_guesses = uniq[:18]

                best_result = None
                best_cost = np.inf
                for guess in initial_guesses:
                    try:
                        phase1_result = least_squares(objective, guess, **phase1_trial_options)
                        if phase1_result.success and phase1_result.cost < best_cost:
                            best_result = phase1_result
                            best_cost = phase1_result.cost
                    except:
                        continue

                if best_result is not None:
                    phase1_result = best_result
                else:
                    phase1_result = least_squares(objective, initial_guess, **phase1_options)
            else:
                # 非单指数模型使用标准方法
                phase1_result = least_squares(objective, initial_guess, **phase1_options)

            print(f"第一阶段结果: {phase1_result.x}")
            
            # 第二阶段: 严格容差精细拟合
            phase2_options = lsq_options.copy()
            if double_exp_model:
                # 保留精修但避免过高上限导致耗时过长
                phase2_options['max_nfev'] = max(self.options.MaxFunEvals, 1200)
            phase2_result = least_squares(objective, phase1_result.x, **phase2_options)
            print(f"第二阶段结果: {phase2_result.x}")

            # 单指数兜底：若新策略失败，回退到更保守的单起点方案
            if single_exp_model:
                bad_result = (
                    (not phase2_result.success) or
                    (not np.all(np.isfinite(phase2_result.x)))
                )
                if bad_result:
                    print("单指数新策略未收敛，回退到保守拟合流程...")

                    y_abs = np.abs(y)
                    legacy_floor = max(np.percentile(y_abs, 5) * 1e-3, 1e-12)

                    def objective_legacy(params):
                        if callable(self.cancel_requested) and self.cancel_requested():
                            raise InterruptedError("用户取消拟合")
                        y_pred = func(x, *params)
                        if not np.all(np.isfinite(y_pred)):
                            return np.full_like(y, 1e10, dtype=float)
                        residuals = y - y_pred
                        residuals = np.where(np.isfinite(residuals), residuals, 1e10)
                        scale = np.sqrt(np.maximum(np.abs(y_pred), legacy_floor))
                        residuals = residuals / scale
                        if self.options.Robust != RobustMethod.OFF:
                            weights = self._robust_weight_function(residuals, self.options.Robust)
                            residuals = residuals * np.sqrt(weights)
                        if self.options.Weights is not None:
                            residuals = residuals * np.sqrt(self.options.Weights)
                        return residuals

                    legacy_phase1 = least_squares(objective_legacy, initial_guess, **phase1_options)
                    legacy_phase2_options = phase2_options.copy()
                    legacy_phase2_options['max_nfev'] = max(self.options.MaxFunEvals, 800)
                    legacy_phase2 = least_squares(objective_legacy, legacy_phase1.x, **legacy_phase2_options)

                    if legacy_phase2.success and np.all(np.isfinite(legacy_phase2.x)):
                        print("单指数回退流程收敛成功")
                        phase2_result = legacy_phase2
                    else:
                        print("单指数回退流程仍未收敛")
            
            # 保存最终结果
            fitted_params = phase2_result.x.copy()
            if single_exp_model:
                # 把平移坐标系下的振幅转换回原始x坐标系
                a_fit, tau_fit, c_fit = fitted_params
                if np.isfinite(tau_fit) and tau_fit != 0:
                    exp_arg = np.clip(x_offset / tau_fit, -700, 700)
                    a_fit = a_fit * np.exp(exp_arg)
                fitted_params = np.array([a_fit, tau_fit, c_fit], dtype=float)
            elif double_exp_model and len(fitted_params) == 5:
                a_fit, t1_fit, c_fit, t2_fit, e_fit = fitted_params
                if t1_fit > t2_fit:
                    fitted_params = np.array([c_fit, t2_fit, a_fit, t1_fit, e_fit], dtype=float)
            result.params = fitted_params
            result.success = phase2_result.success
            result.message = phase2_result.message
            result.iterations = phase2_result.nfev
            result.funcCount = phase2_result.nfev
            result.jacobian = phase2_result.jac
            
            # 计算拟合优度统计量
            y_pred = func(x_original, *result.params)
            residuals = y - y_pred
            
            sse, rsquare, dfe, adjrsquare, rmse = self._calculate_goodness_of_fit(
                y, y_pred, residuals, len(param_names))
            
            result.sse = sse
            result.rsquare = rsquare
            result.dfe = dfe
            result.adjrsquare = adjrsquare
            result.rmse = rmse
            result.residuals = residuals
            
        except InterruptedError as e:
            result.success = False
            result.message = str(e)
            result.params = initial_guess
            return result
        except Exception as e:
            result.success = False
            result.message = f"拟合失败: {str(e)}"
            # 设置初始参数作为回退值
            result.params = initial_guess
            print(f"拟合错误: {str(e)}")
            print(traceback.format_exc())
        
        return result

def run_fit_task(task, cancel_check=None):
    """在后台线程执行拟合计算（不访问UI对象）。"""
    robust_mapping = {
        'Off': RobustMethod.OFF,
        'LAR': RobustMethod.LAR,
        'Bisquare': RobustMethod.BISQUARE
    }
    algorithm_mapping = {
        'Levenberg-Marquardt': Algorithm.LEVENBERG_MARQUARDT,
        'Trust-Region': Algorithm.TRUST_REGION,
    }

    robust_method = robust_mapping.get(task.get('robust_text'), RobustMethod.OFF)
    algorithm_method = algorithm_mapping.get(task.get('algorithm_text'), Algorithm.LEVENBERG_MARQUARDT)

    fit_options = task.get('fit_options', {})

    start = task.get('start', [])
    lower = task.get('lower', [])
    upper = task.get('upper', [])
    base_lower = None
    base_upper = None
    if any(v is not None for v in lower):
        base_lower = np.array([-np.inf if v is None else v for v in lower], dtype=float)
    if any(v is not None for v in upper):
        base_upper = np.array([np.inf if v is None else v for v in upper], dtype=float)

    def _fit_once(x_fit, y_fit, start_point=None):
        fitter = MATLABCurveFitter()
        fitter.cancel_requested = cancel_check
        fitter.set_options(Robust=robust_method, Algorithm=algorithm_method, **fit_options)

        if start_point is None:
            if any(v is not None for v in start):
                fitter.options.StartPoint = start
        else:
            fitter.options.StartPoint = list(start_point)

        if base_lower is not None:
            fitter.options.Lower = np.array(base_lower, dtype=float)
        if base_upper is not None:
            fitter.options.Upper = np.array(base_upper, dtype=float)

        return fitter.fit_curve(x_fit, y_fit, task['model'])

    def _extract_characteristic_tau(model, params):
        try:
            if params is None:
                return None
            p = np.asarray(params, dtype=float)
            if model == '单指数' and len(p) >= 2:
                tau = float(p[1])
                return tau if np.isfinite(tau) and tau > 0 else None
            if model == '双指数' and len(p) >= 4:
                a1, t1, a2, t2 = float(p[0]), float(p[1]), float(p[2]), float(p[3])
                t_candidates = [t for t in [t1, t2] if np.isfinite(t) and t > 0]
                if not t_candidates:
                    return None
                w1 = abs(a1) if np.isfinite(a1) else 0.0
                w2 = abs(a2) if np.isfinite(a2) else 0.0
                w_sum = w1 + w2
                if w_sum > 0:
                    tau_eff = (w1 * t1 + w2 * t2) / w_sum
                    if np.isfinite(tau_eff) and tau_eff > 0:
                        return float(tau_eff)
                return float(max(t_candidates))
        except Exception:
            return None
        return None

    x_fit_all = np.asarray(task['x_for_fitting'])
    y_fit_all = np.asarray(task['y_for_fitting'])
    result = _fit_once(x_fit_all, y_fit_all, start_point=None)
    x_fit_used = x_fit_all
    y_fit_used = y_fit_all

    enable_second_fit = bool(task.get('enable_second_fit', False))
    tau_multiplier = float(task.get('second_fit_tau_multiplier', 5.0) or 5.0)
    if not np.isfinite(tau_multiplier) or tau_multiplier <= 0:
        tau_multiplier = 5.0

    if enable_second_fit and result.success:
        tau_ref = _extract_characteristic_tau(task['model'], result.params)
        if tau_ref is not None and tau_ref > 0:
            x_anchor = float(np.nanmin(x_fit_all))
            x_limit = x_anchor + tau_multiplier * tau_ref
            fit_mask = np.isfinite(x_fit_all) & np.isfinite(y_fit_all) & (x_fit_all >= x_anchor) & (x_fit_all <= x_limit)
            min_points = max(3, len(result.params) + 1 if result.params is not None else 3)
            if np.count_nonzero(fit_mask) >= min_points:
                x_second = x_fit_all[fit_mask]
                y_second = y_fit_all[fit_mask]
                second_result = _fit_once(x_second, y_second, start_point=result.params)
                if second_result.success:
                    result = second_result
                    x_fit_used = x_second
                    y_fit_used = y_second

    return {
        'result': result,
        'model': task['model'],
        'x_for_display': task['x_for_display'],
        'y_for_display': task['y_for_display'],
        'x_for_fitting': x_fit_used,
        'y_for_fitting': y_fit_used,
        'x_for_curve': x_fit_used if enable_second_fit else task['x_for_display']
    }

class FitWorker(QObject):
    """后台拟合worker，避免阻塞主线程事件循环。"""
    finished = pyqtSignal(object)
    failed = pyqtSignal(str)
    cancelled = pyqtSignal()

    def __init__(self, task):
        super().__init__()
        self.task = task
        self._cancel_requested = False

    def request_cancel(self):
        self._cancel_requested = True

    @QtCore.pyqtSlot()
    def run(self):
        try:
            output = run_fit_task(self.task, cancel_check=lambda: self._cancel_requested)
            if self._cancel_requested:
                self.cancelled.emit()
                return
            self.finished.emit(output)
        except InterruptedError:
            self.cancelled.emit()
        except Exception:
            self.failed.emit(traceback.format_exc())

class mainWindow(QMainWindow, main_window):
    def __init__(self, parent=None, product_names=None):
        super(mainWindow, self).__init__(parent)
        self.setupUi(self)
        self.comboBox.currentTextChanged.connect(
            self._sync_constraint_ui_for_model
        )
        self._sync_constraint_ui_for_model(self.comboBox.currentText())

        # 使用常量初始化
        self.temperature_calibration = [50,100,150,200,250,300,350,400,450,500,550,600]
        self.lifetime_calibration = [3.646040506,3.212977315,2.842532971,2.519596718,2.236712857,1.981563617,1.710492252,1.280798396,0.688913014,0.379551499,0.321295208,0.317258355]

        # 初始化设置
        self.settings = QSettings("YourCompany", "PMT_Analysis")
        self.setup_matplotlib_chinese()

        # 连接信号槽
        self._connect_signals()
        
        # 初始化变量
        self._initialize_variables()
        
        # 初始化UI组件
        self._initialize_ui()
        
        # 恢复设置
        self.restore_previous_settings()
        self._restore_widget_states()
        self._connect_auto_persist_signals()

        self.comboBox.currentTextChanged.connect(self._on_model_changed)

    def _sync_constraint_ui_for_model(self, model):
        """
        根据拟合模型，同步“系数约束”UI 状态
        """
        double_only_widgets = [
            self.lineEdit_15,  # A2 初始
            self.lineEdit_13,  # A2 下限
            self.lineEdit_18,  # A2 上限
            self.lineEdit_25,  # τ2 初始
            self.lineEdit_34,  # τ2 下限
            self.lineEdit_35,  # τ2 上限
            self.label_22,     # A2
            self.label_7 ,      # τ2
            self.label_42,
            self.label_43,
            self.lineEdit_26,
            self.lineEdit_28  
        ]
        # UI 可能已将 label_50/51 替换为 comboBox，使用可选访问避免属性错误
        label_tau2_unit = getattr(self, "label_50", None)
        if label_tau2_unit is not None:
            double_only_widgets.append(label_tau2_unit)

        if model == '单指数':
            for w in double_only_widgets:
                w.setVisible(False)
                if isinstance(w, QtWidgets.QLineEdit):
                    w.clear()
            combo_tau2 = getattr(self, "comboBox_12", None)
            if combo_tau2 is not None:
                combo_tau2.setVisible(False)

        else:  # 双指数
            for w in double_only_widgets:
                w.setVisible(True)
            combo_tau2 = getattr(self, "comboBox_12", None)
            if combo_tau2 is not None:
                combo_tau2.setVisible(True)


    def _on_model_changed(self, text):
        self.current_model_type = text
        self._sync_constraint_ui_with_model(text)


    def _connect_signals(self):
        """集中连接所有信号槽"""
        # 现有连接...
        self.treeWidget.currentItemChanged.connect(self._on_tree_current_item_changed)
        self.comboBox_10.currentIndexChanged.connect(self.comboBox_10_changed)
        self.comboBox.currentIndexChanged.connect(self.fit_curve_model)
        
        # 文件夹设置
        self.pushButton.clicked.connect(self.set_work_directory)
        self.pushButton_2.clicked.connect(self.select_target_files)
        self.pushButton_3.clicked.connect(self.set_download_directory)
        
        # 下载按钮
        self.pushButton_5.clicked.connect(self.save_all)
        self.pushButton_7.clicked.connect(self.save_one)
        
        # 数据筛选
        self.comboBox_3.currentTextChanged.connect(self.change_range)
        self.comboBox_4.currentTextChanged.connect(self.change_range)
        
        # 单位设置
        self.comboBox_5.currentTextChanged.connect(self.change_time_unit)
        self.comboBox_6.currentTextChanged.connect(self.change_time_unit)
        self.comboBox_7.currentTextChanged.connect(self.change_time_unit)
        
        # 寿命、温度计算
        self.pushButton_6.clicked.connect(self.calculate_decay)
        
        # 重置初始值
        self.pushButton_reset_start.clicked.connect(self.reset_start_point)
        
        # 新增：导入标定数据
        self.pushButton_4.clicked.connect(self.import_calibration_data)

        # 氧化参数计算（时间输入变化时实时更新）
        self.lineEdit_39.textChanged.connect(self._on_oxidation_time_changed)

        # τ1/τ2 显示单位切换（若UI中存在 comboBox_11/12）
        combo_tau1 = getattr(self, "comboBox_11", None)
        combo_tau2 = getattr(self, "comboBox_12", None)
        if combo_tau1 is not None:
            combo_tau1.currentTextChanged.connect(self.change_tau_unit)
        if combo_tau2 is not None:
            combo_tau2.currentTextChanged.connect(self.change_tau_unit)

    def _on_tree_current_item_changed(self, current, previous):
        """切换文件时，使用与手动计算一致的动画流程"""
        if current is None:
            return
        self.calculate_decay(current, show_progress=True)
    
    def import_calibration_data(self):
        """导入标定数据 - CSV格式，包含'温度,寿命'列，并在新窗口显示图像"""
        try:
            # 获取初始目录
            initial_dir = self.pushButton.text() if self.pushButton.text() else QDir.homePath()
            
            # 打开文件选择对话框
            file_path, _ = QFileDialog.getOpenFileName(
                self,
                "导入标定数据",
                initial_dir,
                "CSV Files (*.csv);;All Files (*.*)"
            )
            
            if not file_path:
                return
                
            # 读取CSV文件
            df = pd.read_csv(file_path)
            
            # 检查必要的列
            required_columns = ['温度', '寿命']
            if not all(col in df.columns for col in required_columns):
                QMessageBox.warning(
                    self, 
                    "格式错误", 
                    f"CSV文件必须包含列: {', '.join(required_columns)}\n"
                    f"当前文件的列: {', '.join(df.columns.tolist())}"
                )
                return
            
            # 提取数据（单位保持为秒s）
            temperatures = df['温度'].values
            lifetimes = df['寿命'].values  # 单位：秒(s)
            
            # 验证数据
            if len(temperatures) != len(lifetimes):
                QMessageBox.warning(self, "数据错误", "温度和寿命数据长度不一致")
                return
                
            if len(temperatures) == 0:
                QMessageBox.warning(self, "数据错误", "没有找到有效数据")
                return
            
            # 创建标定数据预览窗口（单位保持为秒s）
            self.show_calibration_preview(temperatures, lifetimes, file_path)
            
        except Exception as e:
            QMessageBox.critical(self, "导入错误", f"导入标定数据时出错:\n{str(e)}")
            print(f"导入标定数据错误: {str(e)}")
            traceback.print_exc()
            
    def show_calibration_preview(self, temperatures, lifetimes, file_path):
        """显示标定数据预览窗口"""
        # 创建新窗口
        self.calibration_window = QtWidgets.QDialog(self)
        self.calibration_window.setWindowTitle("标定数据预览")
        self.calibration_window.setMinimumSize(800, 600)
        
        # 创建布局
        layout = QtWidgets.QVBoxLayout(self.calibration_window)
        
        # 添加文件信息标签
        file_info = QtWidgets.QLabel(f"文件: {os.path.basename(file_path)}")
        file_info.setStyleSheet("font-weight: bold; font-size: 12pt;")
        layout.addWidget(file_info)
        
        # 添加数据统计标签（使用科学计数法显示，单位：秒）
        stats_text = (f"数据点数: {len(temperatures)} | "
                    f"温度范围: {min(temperatures):.1f} - {max(temperatures):.1f} °C | "
                    f"寿命范围: {min(lifetimes):.5e} - {max(lifetimes):.5e} s")
        stats_label = QtWidgets.QLabel(stats_text)
        stats_label.setStyleSheet("font-size: 10pt;")
        layout.addWidget(stats_label)
        
        # 创建matplotlib图形
        figure = Figure(figsize=(8, 3))
        canvas = FC(figure)
        toolbar = NavigationToolbar(canvas, self.calibration_window)
        
        # 绘制标定曲线
        ax = figure.add_subplot(111)
        
        # 对数据进行排序以便连线
        sorted_indices = np.argsort(lifetimes)
        sorted_lifetimes = lifetimes[sorted_indices]
        sorted_temperatures = temperatures[sorted_indices]
        
        # 绘制散点图
        ax.scatter(lifetimes, temperatures, color='blue', s=50, alpha=0.7, label='标定数据点')
        
        # 单纯连线，不进行拟合
        ax.plot(sorted_lifetimes, sorted_temperatures, 'r-', linewidth=2, label='数据连线')
        
        ax.set_xlabel('寿命 (s)')  # 修改为单位：秒
        ax.set_ylabel('温度 (°C)')
        ax.set_title('温度-寿命标定曲线')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # 设置x轴为科学计数法格式
        ax.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
        ax.xaxis.set_major_formatter(mpl.ticker.FormatStrFormatter('%.1e'))
        
        figure.tight_layout()
        
        # 添加matplotlib组件到布局
        layout.addWidget(toolbar)
        layout.addWidget(canvas)
        
        # 添加数据表格
        table_label = QtWidgets.QLabel("标定数据表:")
        table_label.setStyleSheet("font-weight: bold; margin-top: 10px;")
        layout.addWidget(table_label)
        
        # 创建表格
        table = QtWidgets.QTableWidget()
        table.setRowCount(len(temperatures))
        table.setColumnCount(2)
        table.setHorizontalHeaderLabels(['温度 (°C)', '寿命 (s)'])  # 修改为单位：秒
        
        # 填充表格数据（使用科学计数法）
        for i, (temp, life) in enumerate(zip(temperatures, lifetimes)):
            table.setItem(i, 0, QtWidgets.QTableWidgetItem(f"{temp:.2f}"))
            table.setItem(i, 1, QtWidgets.QTableWidgetItem(f"{life:.5e}"))
        
        table.setMaximumHeight(500)
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        layout.addWidget(table)
        
        # 添加按钮区域
        button_layout = QtWidgets.QHBoxLayout()
        
        # 确认导入按钮
        confirm_button = QtWidgets.QPushButton("确认导入")
        confirm_button.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }")
        confirm_button.clicked.connect(lambda: self.confirm_calibration_import(temperatures, lifetimes))
        
        # 取消按钮
        cancel_button = QtWidgets.QPushButton("取消")
        cancel_button.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-weight: bold; padding: 8px; }")
        cancel_button.clicked.connect(self.calibration_window.reject)
        
        button_layout.addWidget(confirm_button)
        button_layout.addWidget(cancel_button)
        layout.addLayout(button_layout)
        
        # 显示窗口
        self.calibration_window.exec_()
    
    def confirm_calibration_import(self, temperatures, lifetimes):
        """确认导入标定数据"""
        try:
            # 更新标定数据（单位：秒s）
            self.temperature_calibration = temperatures.tolist()
            self.lifetime_calibration = lifetimes.tolist()
            
            # 关闭预览窗口
            if self.calibration_window:
                self.calibration_window.accept()
                self.calibration_window = None
            
            # 显示成功信息（使用科学计数法，单位：秒）
            QMessageBox.information(
                self, 
                "导入成功", 
                f"成功导入 {len(temperatures)} 个标定数据点\n"
                f"温度范围: {min(temperatures):.2f} - {max(temperatures):.2f} °C\n"
                f"寿命范围: {min(lifetimes):.5e} - {max(lifetimes):.5e} s"
            )
            
            # 打印导入信息（使用科学计数法，单位：秒）
            print(f"导入标定数据: {len(temperatures)} 个点")
            print(f"温度: {temperatures}")
            print(f"寿命 (s): [{min(lifetimes):.5e}, {max(lifetimes):.5e}]")
            
        except Exception as e:
            QMessageBox.critical(self, "导入错误", f"确认导入时出错:\n{str(e)}")
            print(f"确认导入标定数据错误: {str(e)}")

    def _initialize_variables(self):
        """初始化实例变量"""
        self.result = None
        self.file_path = None
        self.img_directory = None
        self.download_directory = None
        self.file_data_cache = {}
        self.lifetime = None
        self.temperature_measure = None
        self.temperature = None
        self.current_model_type = None
        
        # 时间单位
        self.time_unit_5 = 1
        self.time_unit_6 = 1
        self.time_unit_7 = 1
        self.tau_unit_11 = 1
        self.tau_unit_12 = 1
        
        # 新增：标定数据相关变量
        self.temperature_calibration = []
        self.lifetime_calibration = []
        self.calibration_window = None

        # 异步拟合相关
        self._fit_thread = None
        self._fit_worker = None
        self._fit_request_id = 0
        self._fit_progress_dialog = None

    def _initialize_ui(self):
        """初始化UI组件"""
        self.lineEdit_8.setEnabled(False)
        self.lineEdit_3.setEnabled(True)
        self.lineEdit_4.setEnabled(False)
        self._setup_secondary_fit_controls()
        
        # 初始化treeWidget
        self.treeWidget.setHeaderLabels(["文件列表"])
        self.treeWidget.setColumnCount(1)

        # 完全隐藏氧化参数区域
        self._hide_oxidation_section()
        # 完全隐藏温度显示区域
        self._hide_temperature_section()
        # 初始化单位状态
        self.change_time_unit()
        self.change_tau_unit()
        
        # 初始化matplotlib图形
        self._setup_matplotlib_canvases()

    def _setup_secondary_fit_controls(self):
        """在“数据筛选”区域添加二次拟合控件。"""
        self.checkBox_second_fit = QtWidgets.QCheckBox(self.page_2)
        self.checkBox_second_fit.setObjectName("checkBox_second_fit")
        self.checkBox_second_fit.setText("二次拟合")

        self.label_second_fit_tau = QtWidgets.QLabel(self.page_2)
        self.label_second_fit_tau.setObjectName("label_second_fit_tau")
        self.label_second_fit_tau.setText("二次时间 n")

        self.lineEdit_second_fit_tau = QtWidgets.QLineEdit(self.page_2)
        self.lineEdit_second_fit_tau.setObjectName("lineEdit_second_fit_tau")
        self.lineEdit_second_fit_tau.setText("5")
        self.lineEdit_second_fit_tau.setMaximumWidth(80)

        self.label_second_fit_tau_unit = QtWidgets.QLabel(self.page_2)
        self.label_second_fit_tau_unit.setObjectName("label_second_fit_tau_unit")
        self.label_second_fit_tau_unit.setText("τ")

        second_fit_layout = QtWidgets.QHBoxLayout()
        second_fit_layout.setContentsMargins(0, 0, 0, 0)
        second_fit_layout.addWidget(self.checkBox_second_fit)
        second_fit_layout.addWidget(self.label_second_fit_tau)
        second_fit_layout.addWidget(self.lineEdit_second_fit_tau)
        second_fit_layout.addWidget(self.label_second_fit_tau_unit)
        second_fit_layout.addStretch(1)
        self.gridLayout_3.addLayout(second_fit_layout, 6, 0, 1, 3)

        self.checkBox_second_fit.toggled.connect(self._on_secondary_fit_toggled)
        self._on_secondary_fit_toggled(False)

    def _on_secondary_fit_toggled(self, checked):
        enabled = bool(checked)
        self.label_second_fit_tau.setEnabled(enabled)
        self.lineEdit_second_fit_tau.setEnabled(enabled)
        self.label_second_fit_tau_unit.setEnabled(enabled)

    def _get_secondary_fit_settings(self):
        """读取二次拟合设置。"""
        checkbox = getattr(self, "checkBox_second_fit", None)
        tau_edit = getattr(self, "lineEdit_second_fit_tau", None)
        enabled = checkbox is not None and checkbox.isChecked()
        tau_text = tau_edit.text() if tau_edit is not None else ""
        tau_n = self._safe_float_conversion(tau_text, 5.0)
        if not np.isfinite(tau_n) or tau_n <= 0:
            tau_n = 5.0
        return enabled, float(tau_n)

    def _hide_oxidation_section(self):
        """完全隐藏氧化参数相关控件。"""
        widgets = [
            self.label, self.label_47, self.label_48, self.label_49,
            self.label_52, self.label_53, self.label_54, self.label_55,
            self.lineEdit_39, self.lineEdit_40, self.lineEdit_41, self.lineEdit_42
        ]

        for w in widgets:
            w.setVisible(False)

        # 清空内容，避免残留
        self.lineEdit_39.clear()
        self.lineEdit_40.clear()
        self.lineEdit_41.clear()
        self.lineEdit_42.clear()

    def _hide_temperature_section(self):
        """完全隐藏温度显示相关控件。"""
        self.groupBox_6.setVisible(False)
        self.textEdit.clear()
        self.pushButton_4.setVisible(False)

    def _setup_matplotlib_canvases(self):
        """设置matplotlib画布 - 最小化工具栏版本"""
        # 第一个图形
        self.figure = Figure(figsize=(5, 4))  # 设置图形尺寸
        self.figure.subplots_adjust(left=0.12, bottom=0.15, right=0.95, top=0.92)
        self.canvas = FC(self.figure)
        self.toolbar = self._create_minimal_toolbar(self.canvas)
        self._add_canvas_to_layout(self.graphicsView, self.toolbar, self.canvas)
        
        # 第二个图形
        self.figure2 = Figure(figsize=(5, 4))
        self.figure2.subplots_adjust(left=0.12, bottom=0.15, right=0.95, top=0.92)
        self.canvas2 = FC(self.figure2)
        self.toolbar2 = self._create_minimal_toolbar(self.canvas2)
        self._add_canvas_to_layout(self.graphicsView_2, self.toolbar2, self.canvas2)
        
        # 第三个图形
        self.figure3 = Figure(figsize=(5, 4))
        self.figure3.subplots_adjust(left=0.12, bottom=0.15, right=0.95, top=0.92)
        self.canvas3 = FC(self.figure3)
        self.toolbar3 = self._create_minimal_toolbar(self.canvas3)
        self._add_canvas_to_layout(self.graphicsView_3, self.toolbar3, self.canvas3)

    def _create_minimal_toolbar(self, canvas):
        """创建最小化工具栏 - 只保留关键功能"""
        toolbar = NavigationToolbar(canvas, self)
        
        # 移除不必要的按钮
        actions_to_remove = []
        for action in toolbar.actions():
            text = action.text().lower() if action.text() else ""
            if any(word in text for word in ['subplots', 'customize', 'save']):
                actions_to_remove.append(action)
        
        for action in actions_to_remove:
            toolbar.removeAction(action)
        
        # 设置小尺寸样式
        toolbar.setStyleSheet("""
            QToolBar {
                background-color: #f0f0f0;
                spacing: 1px;
                padding: 0px;
                margin: 0px;
                border: none;
            }
            QToolButton {
                background-color: transparent;
                border: 1px solid transparent;
                border-radius: 2px;
                padding: 1px;
                margin: 0px;
            }
            QToolButton:hover {
                background-color: #d0d0d0;
                border: 1px solid #a0a0a0;
            }
            QToolButton:pressed {
                background-color: #b0b0b0;
            }
        """)
        
        # 设置固定小尺寸
        toolbar.setFixedHeight(22)
        
        # 设置小图标
        for action in toolbar.actions():
            widget = toolbar.widgetForAction(action)
            if widget and hasattr(widget, 'setIconSize'):
                widget.setIconSize(QtCore.QSize(14, 14))
                widget.setFixedSize(20, 18)
        
        return toolbar

    def _add_canvas_to_layout(self, graphics_view, toolbar, canvas):
        """将matplotlib组件添加到布局 - 优化空间利用"""
        # 清除现有布局
        if graphics_view.layout() is not None:
            QtWidgets.QWidget().setLayout(graphics_view.layout())
        
        layout = QtWidgets.QVBoxLayout(graphics_view)
        layout.setContentsMargins(1, 1, 1, 1)  # 最小边距
        layout.setSpacing(1)  # 最小间距
        
        if toolbar:
            layout.addWidget(toolbar)
        
        layout.addWidget(canvas)
        
        # 设置graphicsView的尺寸策略，优先给图形更多空间
        graphics_view.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

    def _safe_float_conversion(self, text, default=0.0):
        """安全的浮点数转换"""
        try:
            return float(text) if text else default
        except ValueError:
            return default

    def _safe_int_conversion(self, text, default=0):
        """安全的整数转换"""
        try:
            return int(text) if text else default
        except ValueError:
            return default

    def get_fit_options(self):
        """安全获取拟合选项"""
        return {
            'DiffMinChange': self._safe_float_conversion(self.lineEdit_19.text(), DEFAULT_FIT_OPTIONS['DiffMinChange']),
            'DiffMaxChange': self._safe_float_conversion(self.lineEdit_20.text(), DEFAULT_FIT_OPTIONS['DiffMaxChange']),
            'MaxIter': self._safe_int_conversion(self.lineEdit_21.text(), DEFAULT_FIT_OPTIONS['MaxIter']),
            'MaxFunEvals': self._safe_int_conversion(self.lineEdit_22.text(), DEFAULT_FIT_OPTIONS['MaxFunEvals']),
            'TolFun': self._safe_float_conversion(self.lineEdit_23.text(), DEFAULT_FIT_OPTIONS['TolFun']),
            'TolX': self._safe_float_conversion(self.lineEdit_24.text(), DEFAULT_FIT_OPTIONS['TolX'])
        }

    def _get_file_filters(self):
        """获取文件过滤器"""
        return (
            "Supported Files (*.xlsx *.xls *.txt *.csv);;"
            "Excel Files (*.xlsx *.xls);;"
            "Text Files (*.txt);;"
            "CSV Files (*.csv);;"
            "All Files (*.*)"
        )

    def _validate_file_extension(self, file_path):
        """验证文件扩展名"""
        return file_path.lower().endswith(('.xlsx', '.xls', '.txt', '.csv'))

    def _create_tree_item(self, file_path):
        """创建树形项目"""
        file_name = os.path.basename(file_path)
        item = QTreeWidgetItem(self.treeWidget)
        item.setText(0, file_name)
        item.setData(0, QtCore.Qt.UserRole, file_path)
        return item

    def setup_matplotlib_chinese(self):
        """配置matplotlib支持中文显示"""
        # 设置中文字体
        mpl.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'WenQuanYi Zen Hei']  # 指定默认字体
        mpl.rcParams['axes.unicode_minus'] = False  # 解决保存图像时负号'-'显示为方块的问题

    def change_range(self):
        comboBox_3 = self.comboBox_3.currentText()
        comboBox_4 = self.comboBox_4.currentText()
        if comboBox_3 == '最小值':
            self.lineEdit_3.setText(self.comboBox_3.currentText())
            self.lineEdit_3.setEnabled(False)
        else:
            self.lineEdit_3.setText('')
            self.lineEdit_3.setEnabled(True)
        if comboBox_4 == '最大值':
            self.lineEdit_4.setText(self.comboBox_4.currentText())
            self.lineEdit_4.setEnabled(False)
        else:
            self.lineEdit_4.setText('')
            self.lineEdit_4.setEnabled(True)

    def change_time_unit(self):
        text_5 = self.comboBox_5.currentText()
        text_6 = self.comboBox_6.currentText()
        text_7 = self.comboBox_7.currentText()
        self.time_unit_5 = UNIT_MAPPING[text_5]
        self.time_unit_6 = UNIT_MAPPING[text_6]
        self.time_unit_7 = UNIT_MAPPING[text_7]

    def change_tau_unit(self):
        """更新 τ1/τ2 的显示单位并刷新结果显示。"""
        combo_tau1 = getattr(self, "comboBox_11", None)
        combo_tau2 = getattr(self, "comboBox_12", None)

        if combo_tau1 is not None:
            self.tau_unit_11 = UNIT_MAPPING.get(combo_tau1.currentText(), 1)
        else:
            self.tau_unit_11 = 1

        if combo_tau2 is not None:
            self.tau_unit_12 = UNIT_MAPPING.get(combo_tau2.currentText(), 1)
        else:
            self.tau_unit_12 = 1

        self._refresh_tau_display_from_result()

    def _refresh_tau_display_from_result(self):
        """根据当前拟合结果和单位设置刷新 τ1/τ2 显示。"""
        if not (hasattr(self, 'result') and self.result and self.result.params is not None):
            return

        try:
            model = self.current_model_type or self.comboBox.currentText()
            if model == '单指数' and len(self.result.params) >= 2:
                tau1 = float(self.result.params[1])
                self.lineEdit.setText(f"{tau1 * self.tau_unit_11:.5e}")
                self.lineEdit_28.clear()
            elif model == '双指数' and len(self.result.params) >= 4:
                t1 = float(self.result.params[1])
                t2 = float(self.result.params[3])
                self.lineEdit.setText(f"{t1 * self.tau_unit_11:.5e}")
                self.lineEdit_28.setText(f"{t2 * self.tau_unit_12:.5e}")
        except Exception as e:
            print(f"刷新τ显示单位时出错: {str(e)}")

    def _get_time_thresholds_in_seconds(self):
        """
        读取时间阈值并转换到秒(s)。
        lineEdit_2 对应“大于”(comboBox_5)
        lineEdit_5 对应“小于”(comboBox_6)
        """
        x_min = None
        x_max = None

        if self.lineEdit_2.text():
            x_min = float(self.lineEdit_2.text()) / float(self.time_unit_5)

        if self.lineEdit_5.text():
            x_max = float(self.lineEdit_5.text()) / float(self.time_unit_6)

        return x_min, x_max
    
    def fit_curve_model(self):
        text = self.comboBox.currentText()
        if text == '双指数':
            self.label_21.setText('I(t) = A1exp(-t/τ1) + A2exp(-t/τ2) + I0')
        if text == '单指数':
            self.label_21.setText('I(t) = Aexp(-t/τ) + I0')

    def comboBox_10_changed(self):
        text = self.comboBox_10.currentText()
        if text == '自动':
            self.lineEdit_8.setText('当整行都为数值时')
            self.lineEdit_8.setEnabled(False)
        if text == '行':
            self.lineEdit_8.setText('1')
            self.lineEdit_8.setEnabled(True)
    
    def restore_previous_settings(self):
        """恢复上一次的工作路径和文件列表"""
        # 恢复工作路径
        last_work_dir = self.settings.value("last_work_directory", "")
        if last_work_dir and os.path.exists(last_work_dir):
            self.pushButton.setText(last_work_dir)
        last_download_dir = self.settings.value("last_download_directory", "")
        if last_download_dir and os.path.exists(last_download_dir):
            self.pushButton_3.setText(last_download_dir)
        last_img_dir = self.settings.value("last_img_directory", "")

        
        # 恢复文件列表
        file_list = self.settings.value("last_file_list", [])
        if file_list:
            for file_path in file_list:
                if os.path.exists(file_path):
                    file_name = os.path.basename(file_path)
                    item = self._create_tree_item(file_path)
            
            # 更新按钮文本显示已选择文件数量
            selected_count = self.treeWidget.topLevelItemCount()
            self.pushButton_2.setText(f"已选择 {selected_count} 个文件")

    def _connect_auto_persist_signals(self):
        """对可编辑参数启用自动保存。"""
        for widget in self.findChildren(QtWidgets.QLineEdit):
            widget.editingFinished.connect(self.save_current_settings)
        for widget in self.findChildren(QtWidgets.QComboBox):
            widget.currentIndexChanged.connect(self.save_current_settings)
        for widget in self.findChildren(QtWidgets.QSpinBox):
            widget.valueChanged.connect(self.save_current_settings)
        for widget in self.findChildren(QtWidgets.QDoubleSpinBox):
            widget.valueChanged.connect(self.save_current_settings)
        for widget in self.findChildren(QtWidgets.QCheckBox):
            widget.toggled.connect(self.save_current_settings)
        for widget in self.findChildren(QtWidgets.QToolBox):
            widget.currentChanged.connect(self.save_current_settings)

    def _restore_widget_states(self):
        """恢复通用控件状态（除文件路径/文件列表外）。"""
        try:
            self.settings.beginGroup("ui_state")
            for widget in self.findChildren(QtWidgets.QLineEdit):
                key = f"lineedit/{widget.objectName()}"
                if self.settings.contains(key):
                    widget.setText(self.settings.value(key, "", type=str))
            for widget in self.findChildren(QtWidgets.QComboBox):
                key = f"combobox/{widget.objectName()}"
                if self.settings.contains(key):
                    idx = self.settings.value(key, 0, type=int)
                    if 0 <= idx < widget.count():
                        widget.setCurrentIndex(idx)
            for widget in self.findChildren(QtWidgets.QSpinBox):
                key = f"spinbox/{widget.objectName()}"
                if self.settings.contains(key):
                    widget.setValue(self.settings.value(key, widget.value(), type=int))
            for widget in self.findChildren(QtWidgets.QDoubleSpinBox):
                key = f"doublespin/{widget.objectName()}"
                if self.settings.contains(key):
                    widget.setValue(self.settings.value(key, widget.value(), type=float))
            for widget in self.findChildren(QtWidgets.QCheckBox):
                key = f"checkbox/{widget.objectName()}"
                if self.settings.contains(key):
                    widget.setChecked(self.settings.value(key, widget.isChecked(), type=bool))
            for widget in self.findChildren(QtWidgets.QToolBox):
                key = f"toolbox/{widget.objectName()}"
                if self.settings.contains(key):
                    idx = self.settings.value(key, widget.currentIndex(), type=int)
                    if 0 <= idx < widget.count():
                        widget.setCurrentIndex(idx)
        except Exception as e:
            print(f"恢复控件状态时出错: {str(e)}")
        finally:
            self.settings.endGroup()

    def set_download_directory(self):
        """打开文件夹选择对话框并更新lineEdit内容"""
        try:
            initial_dir = self.pushButton_3.text() or QDir.homePath()
            folder_path = QFileDialog.getExistingDirectory(
                self,
                "选择数据保存路径",
                initial_dir,
                QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks
            )

            
            if folder_path:
                self.pushButton_3.setText(folder_path)
                if self.validate_folder_path(folder_path):
                    print(f"有效路径: {folder_path}")
                    # 保存设置
                    self.save_current_settings()
                else:
                    print("警告：路径不可访问")
            
            self.download_directory = self.pushButton_3.text()
        except Exception as e:
            QMessageBox.critical(self, "错误", f"设置保存目录时出错:\n{str(e)}")

    def set_img_directory(self):
        """打开文件夹选择对话框"""
        try:
            initial_dir = self.pushButton_4.text() or QDir.homePath()
            folder_path = QFileDialog.getExistingDirectory(
                self,
                "选择图片保存路径",
                initial_dir,
                QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks
            )

            if folder_path:
                self.pushButton_4.setText(folder_path)
                if self.validate_folder_path(folder_path):
                    print(f"有效路径: {folder_path}")
                    # 保存设置
                    self.save_current_settings()
                else:
                    print("警告：路径不可访问")
            self.img_directory = self.pushButton_4.text()
        except Exception as e:
            QMessageBox.critical(self, "错误", f"设置图片目录时出错:\n{str(e)}")
    
    def save_current_settings(self):
        """保存当前的工作路径和文件列表到设置"""
        try:
            # 保存工作路径
            work_dir = self.pushButton.text()
            if work_dir and os.path.exists(work_dir):
                self.settings.setValue("last_work_directory", work_dir)
            download_dir = self.pushButton_3.text()
            if download_dir and os.path.exists(download_dir):
                self.settings.setValue("last_download_directory", download_dir)        

            
            # 保存文件列表
            file_list = []
            for i in range(self.treeWidget.topLevelItemCount()):
                item = self.treeWidget.topLevelItem(i)
                file_path = item.data(0, QtCore.Qt.UserRole)
                if file_path and os.path.exists(file_path):
                    file_list.append(file_path)
            
            self.settings.setValue("last_file_list", file_list)
            self.settings.beginGroup("ui_state")
            for widget in self.findChildren(QtWidgets.QLineEdit):
                self.settings.setValue(f"lineedit/{widget.objectName()}", widget.text())
            for widget in self.findChildren(QtWidgets.QComboBox):
                self.settings.setValue(f"combobox/{widget.objectName()}", widget.currentIndex())
            for widget in self.findChildren(QtWidgets.QSpinBox):
                self.settings.setValue(f"spinbox/{widget.objectName()}", widget.value())
            for widget in self.findChildren(QtWidgets.QDoubleSpinBox):
                self.settings.setValue(f"doublespin/{widget.objectName()}", widget.value())
            for widget in self.findChildren(QtWidgets.QCheckBox):
                self.settings.setValue(f"checkbox/{widget.objectName()}", widget.isChecked())
            for widget in self.findChildren(QtWidgets.QToolBox):
                self.settings.setValue(f"toolbox/{widget.objectName()}", widget.currentIndex())
            self.settings.endGroup()
        except Exception as e:
            print(f"保存设置时出错: {str(e)}")

    def apply_external_context(self, work_dir: str, save_dir: Optional[str] = None, target_files: Optional[List[str]] = None):
        """供外部主程序调用：设置工作目录/目标文件/保存目录。"""
        try:
            work_dir = (work_dir or "").strip()
            save_dir = (save_dir or work_dir or "").strip()
            if work_dir and os.path.isdir(work_dir):
                self.pushButton.setText(work_dir)
            if save_dir and os.path.isdir(save_dir):
                self.pushButton_3.setText(save_dir)

            files = list(target_files or [])
            if not files and work_dir and os.path.isdir(work_dir):
                files = [
                    os.path.join(work_dir, fn)
                    for fn in sorted(os.listdir(work_dir))
                    if fn.lower().endswith(".csv")
                ]
            self.treeWidget.clear()
            for file_path in files:
                if os.path.isfile(file_path):
                    self._create_tree_item(file_path)
            self.pushButton_2.setText(f"已选择 {self.treeWidget.topLevelItemCount()} 个文件")
            self.save_current_settings()
        except Exception as e:
            QMessageBox.critical(self, "错误", f"应用外部参数时出错:\n{str(e)}")

    def auto_run_save_all(self):
        """外部调用：自动启动“全部文件结果保存”流程。"""
        QTimer.singleShot(0, self.save_all)

    def set_work_directory(self):
        """打开文件夹选择对话框并更新lineEdit内容"""
        try:
            initial_dir = self.pushButton.text() or QDir.homePath()
            
            folder_path = QFileDialog.getExistingDirectory(
                self,
                "选择工作文件夹",
                initial_dir,
                QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks
            )
            
            if folder_path:
                self.pushButton.setText(folder_path)
                if self.validate_folder_path(folder_path):
                    print(f"有效路径: {folder_path}")
                    # 保存设置
                    self.save_current_settings()
                else:
                    print("警告：路径不可访问")
        except Exception as e:
            QMessageBox.critical(self, "错误", f"设置工作目录时出错:\n{str(e)}")
                
    def save_one(self):
        """保存当前文件的衰减计算结果到txt文件"""
        
        try:
            # 检查是否有选中的文件
            current_item = self.treeWidget.currentItem()
            if current_item is None:
                QMessageBox.warning(self, "警告", "请先选择一个文件")
                return
            
            # 检查保存目录是否设置
            self.download_directory = self.pushButton_3.text()

            if not self.download_directory or not os.path.exists(self.download_directory):
                QMessageBox.warning(self, "警告", "请先设置有效的保存目录")
                return
            
            file_path = current_item.data(0, QtCore.Qt.UserRole)
            file_name = os.path.basename(file_path)
            
            # 创建进度对话框
            progress = self.create_progress_dialog("处理进度", "正在处理文件...", 1)
            self.update_progress(progress, 0, f"正在处理: {file_name}")
            
            print(f"文件路径: {file_path}")

            # 加载文件数据
            if not self.load_file_data(file_path):
                QMessageBox.warning(self, "错误", f"无法加载文件数据: {file_name}")
                progress.close()
                return
            
            # 调用calculate_decay，不显示内部进度对话框
            success = self.calculate_decay(current_item, show_progress=False)
            if not success:
                QMessageBox.warning(self, "错误", f"处理文件 {file_name} 失败")
                progress.close()
                return
            
            # 如果有标定数据，计算温度
            if (hasattr(self, 'temperature_calibration') and self.temperature_calibration and
                hasattr(self, 'lifetime_calibration') and self.lifetime_calibration):
                try:
                    avg_lifetime = self.calculate_average_lifetime()
                    if avg_lifetime is not None:
                        self.calculate_temperature_by_interpolation(avg_lifetime)
                except Exception as e:
                    print(f"保存前计算温度时出错: {str(e)}")
            
            # 收集拟合结果
            result_data = self._create_result_data(file_name)
            if not result_data:
                QMessageBox.warning(self, "警告", "当前文件拟合失败，无法保存结果")
                progress.close()
                return

            # 完成进度条
            self.update_progress(progress, 1, "正在保存结果...")
            
            # 设置输出文件名
            output_file = self._get_output_filename(file_name, self.download_directory)
            
            # 写入文件
            with open(output_file, 'w', encoding='utf-8') as f:
                # 写入表头
                header = self._generate_header(result_data)
                f.write(header + '\n')
                
                # 写入数据
                line = self._format_result_line(result_data)
                f.write(line + '\n')
            
            progress.close()
            
            QMessageBox.information(self, "完成", f"结果已保存到: {output_file}")
            print(f"结果已保存到: {output_file}")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存结果时出错: {str(e)}")
            traceback.print_exc()
        
    def save_all(self):
        """保存所有文件的衰减计算结果到txt文件 - 详细进度版本"""
        
        try:
            # 检查是否有文件需要处理
            file_count = self.treeWidget.topLevelItemCount()
            if file_count == 0:
                QMessageBox.warning(self, "警告", "没有可处理的文件")
                return
            
            # 检查保存目录是否设置
            self.download_directory = self.pushButton_3.text()

            if not self.download_directory or not os.path.exists(self.download_directory):
                QMessageBox.warning(self, "警告", "请先设置有效的保存目录")
                return
            
            # 创建自定义进度对话框
            progress_dialog = QtWidgets.QDialog(self)
            progress_dialog.setWindowTitle("处理进度")
            progress_dialog.setFixedSize(400, 150)
            progress_dialog.setModal(True)
            progress_dialog._cancelled = False
            
            layout = QtWidgets.QVBoxLayout(progress_dialog)
            
            # 当前文件标签
            current_file_label = QtWidgets.QLabel("准备开始...")
            current_file_label.setAlignment(QtCore.Qt.AlignCenter)
            layout.addWidget(current_file_label)
            
            # 进度条
            progress_bar = QtWidgets.QProgressBar()
            progress_bar.setRange(0, file_count)
            layout.addWidget(progress_bar)
            
            # 进度文本
            progress_text = QtWidgets.QLabel(f"0/{file_count}")
            progress_text.setAlignment(QtCore.Qt.AlignCenter)
            layout.addWidget(progress_text)
            
            # 取消按钮
            cancel_button = QtWidgets.QPushButton("取消")
            def _cancel_save_all():
                progress_dialog._cancelled = True
                progress_dialog.reject()
            cancel_button.clicked.connect(_cancel_save_all)
            layout.addWidget(cancel_button)
            
            progress_dialog.show()
            
            # 创建结果列表
            all_results = []
            cancelled = False
            
            # 遍历所有文件
            for i in range(file_count):
                item = self.treeWidget.topLevelItem(i)
                file_path = item.data(0, QtCore.Qt.UserRole)
                file_name = os.path.basename(file_path)
                self.file_path = file_path

                # 更新进度显示
                current_file_label.setText(f"正在处理: {file_name}")
                progress_bar.setValue(i)
                progress_text.setText(f"{i+1}/{file_count}")
                
                QtWidgets.QApplication.processEvents()
                
                # 检查是否取消（QDialog 无 wasCanceled）
                if getattr(progress_dialog, "_cancelled", False):
                    cancelled = True
                    break

                print(f"文件路径: {self.file_path}")

                # 加载文件数据
                if not self.load_file_data(self.file_path):
                    print(f"跳过无法加载的文件: {file_name}")
                    continue
            
                # 调用calculate_decay并传递item参数，不显示内部进度对话框
                success = self.calculate_decay(item, show_progress=False)
                if not success:
                    print(f"处理文件 {file_name} 失败")
                    continue
                
                # 如果有标定数据，计算温度
                avg_lifetime = None
                if (hasattr(self, 'temperature_calibration') and self.temperature_calibration and
                    hasattr(self, 'lifetime_calibration') and self.lifetime_calibration):
                    try:
                        avg_lifetime = self.calculate_average_lifetime()
                        if avg_lifetime is not None:
                            self.calculate_temperature_by_interpolation(avg_lifetime)
                    except Exception as e:
                        print(f"保存前计算温度时出错: {str(e)}")
                
                if avg_lifetime is not None:
                    print(f"平均寿命: {avg_lifetime}")
                
                # 收集拟合结果
                result_data = self._create_result_data(file_name)
                if result_data:
                    all_results.append(result_data)
                    print(f"成功收集文件 {file_name} 的拟合结果")
                else:
                    print(f"收集文件 {file_name} 的拟合结果时出错")
                    continue
            
            progress_dialog.close()
            
            # 如果用户取消了操作
            if cancelled:
                QMessageBox.information(self, "提示", "操作已取消")
                return
            
            # 如果没有结果，则返回
            if not all_results:
                QMessageBox.warning(self, "警告", "没有可保存的结果")
                return
            
            # 设置输出文件名（添加时间后缀）
            timestamp = QtCore.QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
            output_file = os.path.join(self.download_directory, f"decay_results_{timestamp}.txt")
            
            # 写入文件
            with open(output_file, 'w', encoding='utf-8') as f:
                # 写入表头
                header = self._generate_header(all_results[0])
                f.write(header + '\n')
                
                # 写入数据
                for result in all_results:
                    line = self._format_result_line(result)
                    f.write(line + '\n')
            
            QMessageBox.information(self, "完成", f"结果已保存到: {output_file}")
            print(f"结果已保存到: {output_file}")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存结果时出错: {str(e)}")
            traceback.print_exc()

    def _create_result_data(self, file_name):
        """创建结果数据结构"""
        if not (hasattr(self, 'result') and self.result and self.result.success):
            return None
        
        # 计算平均寿命
        avg_lifetime = None
        if hasattr(self, 'result') and self.result and self.result.success:
            try:
                avg_lifetime = self.calculate_average_lifetime()
            except Exception as e:
                print(f"计算平均寿命时出错: {str(e)}")
                avg_lifetime = None
        
        # 使用已计算的温度（如果存在）
        temperature = None
        if hasattr(self, 'temperature'):
            temperature = self.temperature
        
        result_data = {
            'file_name': file_name,
            'model_type': self.comboBox.currentText(),
            'params': self.result.params.copy() if self.result.params is not None else [],
            'param_names': self.result.param_names.copy() if self.result.param_names is not None else [],
            'sse': self.result.sse,
            'rsquare': self.result.rsquare,
            'adjrsquare': self.result.adjrsquare,
            'rmse': self.result.rmse,
            'success': self.result.success,
            'avg_lifetime': avg_lifetime,  # 新增平均寿命
            'temperature': temperature     # 温度
        }
        
        return result_data

    def _get_output_filename(self, base_name, directory):
        """生成输出文件名"""
        timestamp = QtCore.QDateTime.currentDateTime().toString("yyyyMMdd_hhmmss")
        safe_name = os.path.splitext(base_name)[0]
        return os.path.join(directory, f"decay_result_{safe_name}_{timestamp}.txt")

    def _generate_header(self, sample_result):
        """生成表头"""
        header_parts = ["文件名", "模型类型"]
        
        # 添加系数估计的列名
        if sample_result['param_names']:
            for param_name in sample_result['param_names']:
                header_parts.append(f"系数_{param_name}")
        
        # 添加拟合优度统计的列名
        header_parts.extend(["SSE", "R平方", "调整R平方", "RMSE", "拟合成功"])
        
        # 添加平均寿命和温度列（最后两栏）
        header_parts.append("平均寿命(s)")
        
        # 只要有标定数据就添加温度列
        if (hasattr(self, 'temperature_calibration') and self.temperature_calibration and
            hasattr(self, 'lifetime_calibration') and self.lifetime_calibration):
            header_parts.append("温度(°C)")
        
        return "\t".join(header_parts)

    def _format_result_line(self, result):
        """格式化单行结果，使用科学计数法"""
        line_parts = [
            result['file_name'],
            result['model_type']
        ]
        
        # 添加系数估计的值（使用科学计数法）
        if result['params'] is not None:
            for param_value in result['params']:
                line_parts.append(f"{param_value:.5e}")
        
        # 添加拟合优度统计的值（使用科学计数法）
        line_parts.extend([
            f"{result['sse']:.5e}",
            f"{result['rsquare']:.5e}",
            f"{result['adjrsquare']:.5e}",
            f"{result['rmse']:.5e}",
            "是" if result['success'] else "否"  # 拟合成功状态
        ])
        
        # 添加平均寿命（使用科学计数法）
        if result.get('avg_lifetime') is not None:
            line_parts.append(f"{result['avg_lifetime']:.5e}")
        else:
            line_parts.append("N/A")
        
        # 添加温度值（如果有标定数据就包含这一列）
        if (hasattr(self, 'temperature_calibration') and self.temperature_calibration and
            hasattr(self, 'lifetime_calibration') and self.lifetime_calibration):
            if result.get('temperature') is not None:
                line_parts.append(f"{result['temperature']:.2f}")  # 温度保持2位小数
            else:
                line_parts.append("N/A")
        
        return "\t".join(line_parts)

    def select_target_files(self):
        """打开文件选择对话框，选择Excel/TXT/CSV文件并添加到treeWidget"""
        try:
            initial_dir = self.pushButton.text() if self.pushButton.text() else QDir.homePath()
            
            file_paths, _ = QFileDialog.getOpenFileNames(
                self,
                "选择目标文件",
                initial_dir,
                self._get_file_filters()
            )
            
            if file_paths:
                # 清空现有内容（可选，根据需要决定是否保留之前的内容）
                self.treeWidget.clear()
                
                for file_path in file_paths:
                    if self._validate_file_extension(file_path):
                        self._create_tree_item(file_path)
                    else:
                        QMessageBox.warning(self, "警告", f"忽略不支持的文件类型: {os.path.basename(file_path)}")
                
                # 更新按钮文本显示已选择文件数量
                selected_count = self.treeWidget.topLevelItemCount()
                self.pushButton_2.setText(f"已选择 {selected_count} 个文件")
                
                # 保存设置
                self.save_current_settings()
        except Exception as e:
            QMessageBox.critical(self, "错误", f"选择文件时出错:\n{str(e)}")
        
    def _load_excel_data(self, file_path, column1, column2, data_start_row=None):
        """加载Excel数据"""
        try:
            if data_start_row is not None:
                data = pd.read_excel(file_path, skiprows=data_start_row).iloc[:, [column1, column2]]
            else:
                data = pd.read_excel(file_path).iloc[:, [column1, column2]]
            
            return self._clean_dataframe(data)
        except Exception as e:
            print(f"Excel读取失败: {e}")
            return None

    def _load_text_data_auto(self, file_path, column1, column2):
        """自动加载文本数据"""
        try:
            # 使用pandas读取，指定数据类型为数值
            df = pd.read_csv(file_path, header=None, dtype=str)  # 先读为字符串
            
            # 查找数据开始行
            data_start_row = 0
            for i in range(len(df)):
                try:
                    val1 = df.iloc[i, column1]
                    val2 = df.iloc[i, column2]
                    if pd.notna(val1) and pd.notna(val2) and val1.strip() and val2.strip():
                        # 尝试转换为浮点数
                        float(val1)
                        float(val2)
                        data_start_row = i
                        break
                except (ValueError, TypeError):
                    continue
            
            # 重新读取数据并转换为数值
            data = pd.read_csv(file_path, header=None, skiprows=data_start_row, 
                            usecols=[column1, column2], dtype=str)
            
            return self._clean_dataframe(data)
            
        except Exception as e:
            print(f"Pandas读取失败，尝试手动解析: {e}")
            return None

    def _clean_dataframe(self, df):
        """清理数据框"""
        df = df.apply(pd.to_numeric, errors='coerce')
        df = df.dropna()
        return df.values if not df.empty else None

    def load_file_data(self, file_path):
        """加载文件数据到缓存，确保数据为数值类型"""
        try:
            # 验证列设置
            try:
                column1 = int(self.lineEdit_9.text()) - 1
                column2 = int(self.lineEdit_10.text()) - 1
                if column1 < 0 or column2 < 0:
                    QMessageBox.warning(self, "警告", "列号必须为正整数")
                    return False
            except ValueError:
                QMessageBox.warning(self, "警告", "列号必须为有效数字")
                return False
            
            if self.comboBox_10.currentText() == '自动':
                if file_path.lower().endswith(('.txt', '.csv')):
                    data = self._load_text_data_auto(file_path, column1, column2)
                    if data is None:
                        data = self._load_file_manually(file_path, column1, column2, None)
                
                elif file_path.lower().endswith(('.xlsx', '.xls')):
                    data = self._load_excel_data(file_path, column1, column2)
            
            elif self.comboBox_10.currentText() == '行':
                try:
                    data_start_row = int(self.lineEdit_8.text()) - 1
                    if data_start_row < 0:
                        QMessageBox.warning(self, "警告", "起始行必须为正整数")
                        return False
                except ValueError:
                    QMessageBox.warning(self, "警告", "起始行必须为有效数字")
                    return False
                    
                if file_path.lower().endswith(('.txt', '.csv')):
                    data = self._load_file_manually(file_path, column1, column2, data_start_row)
                elif file_path.lower().endswith(('.xlsx', '.xls')):
                    data = self._load_excel_data(file_path, column1, column2, data_start_row)
            
            if data is not None:
                print(f"加载数据形状: {data.shape}, 数据类型: {data.dtype}")
                self.file_data_cache[file_path] = data
                return True
            else:
                QMessageBox.warning(self, "警告", f"无法从文件 {os.path.basename(file_path)} 中读取有效数据")
                return False
                
        except Exception as e:
            error_msg = f"加载文件 {file_path} 出错: {str(e)}"
            print(error_msg)
            QMessageBox.critical(self, "错误", error_msg)
            traceback.print_exc()
            return False
         
    def _load_file_manually(self, file_path, column1, column2, data_start_row):
        """手动解析文件，确保数据类型正确"""
        try:
            valid_data = []
            
            with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()
            
            start_processing = False
            line_count = 0
            
            for line in lines:
                line_count += 1
                
                # 如果指定了起始行，检查是否到达起始行
                if data_start_row is not None:
                    if line_count <= data_start_row:
                        continue
                    start_processing = True
                else:
                    # 自动模式：检查是否是数据行
                    if not start_processing:
                        if self._is_data_line(line, column1, column2):
                            start_processing = True
                        else:
                            continue
                
                if start_processing:
                    # 处理数据行
                    processed_line = self._process_data_line(line, column1, column2)
                    if processed_line is not None:
                        valid_data.append(processed_line)
            
            if not valid_data:
                print(f"文件 {file_path} 中没有找到有效数据")
                return None
            
            # 转换为numpy数组并确保为浮点类型
            data = np.array(valid_data, dtype=float)
            print(f"手动加载数据形状: {data.shape}, 数据类型: {data.dtype}")
            
            return data
            
        except Exception as e:
            print(f"手动解析文件 {file_path} 出错: {str(e)}")
            return None

    def _is_data_line(self, line, column1, column2):
        """检查行是否是有效的数据行"""
        line = line.strip()
        if not line:
            return False
        
        parts = line.split(',')
        if len(parts) <= max(column1, column2):
            return False
        
        try:
            # 尝试转换两个列的值
            val1 = parts[column1].strip()
            val2 = parts[column2].strip()
            
            # 检查是否为空字符串
            if not val1 or not val2:
                return False
            
            float(val1)
            float(val2)
            return True
            
        except (ValueError, IndexError):
            return False

    def _process_data_line(self, line, column1, column2):
        """处理单行数据，返回[x, y]或None，确保数值类型"""
        line = line.strip()
        if not line:
            return None
        
        # 支持多种分隔符：逗号、制表符、空格
        separators = [',', '\t', ' ']
        parts = None
        
        for sep in separators:
            temp_parts = line.split(sep)
            if len(temp_parts) > max(column1, column2):
                parts = [p.strip() for p in temp_parts if p.strip()]  # 移除空字符串
                break
        
        if parts is None or len(parts) <= max(column1, column2):
            return None
        
        try:
            val1 = parts[column1]
            val2 = parts[column2]
            
            # 跳过空值
            if not val1 or not val2:
                return None
            
            # 移除可能的引号或其他非数字字符
            val1 = val1.replace('"', '').replace("'", "").strip()
            val2 = val2.replace('"', '').replace("'", "").strip()
            
            x_val = float(val1)
            y_val = float(val2)
            
            return [x_val, y_val]
            
        except (ValueError, IndexError) as e:
            # 打印无法转换的值以便调试
            print(f"无法转换的数据: '{parts[column1]}', '{parts[column2]}' - 错误: {e}")
            return None

    def create_progress_dialog(self, title, message, maximum=0):
        """创建进度对话框 - 修复动画显示"""
        progress = QtWidgets.QProgressDialog(message, "取消", 0, maximum, self)
        progress.setWindowTitle(title)
        progress.setWindowModality(QtCore.Qt.WindowModal)
        progress.setMinimumDuration(0)  # 立即显示
        
        if maximum == 0:  # 不确定进度
            progress.setCancelButton(None)  # 移除取消按钮
            
        # 强制显示
        progress.show()
        progress.setValue(0)
        QtWidgets.QApplication.processEvents()
        
        return progress

    def update_progress(self, progress, value, message=None):
        """更新进度 - 修复动画显示"""
        if message:
            progress.setLabelText(message)
        if value >= 0:
            progress.setValue(value)
        
        # 强制重绘以确保动画显示
        progress.repaint()
        QtWidgets.QApplication.processEvents()
        
        return not progress.wasCanceled()

    def data_range(self, x, y_data):
        """数据范围筛选"""
        try:
            # 确保输入是numpy数组
            x = np.asarray(x)
            y_data = np.asarray(y_data)
            
            # 初始化筛选条件（默认不筛选）
            x_mask = np.ones_like(x, dtype=bool)
            y_mask = np.ones_like(y_data, dtype=bool)

            # 处理时间范围筛选（x轴范围，统一换算到秒）
            x_min, x_max = self._get_time_thresholds_in_seconds()
            if x_min is not None:
                x_mask &= (x >= x_min)

            if x_max is not None:
                x_mask &= (x <= x_max)
            
            # 应用时间筛选后的数据
            x_filtered = x[x_mask]
            y_filtered = y_data[x_mask]
            
            # 如果没有数据通过时间筛选，直接返回空数组
            if len(x_filtered) == 0:
                return np.array([]), np.array([])
            
            # 计算vrange（基于时间筛选后的数据）
            vrange = np.max(y_filtered) - np.min(y_filtered)
            
            # 处理最大值条件 - 仅保留从最大值点开始的数据（不移动x轴）
            if self.lineEdit_4.text():
                if self.lineEdit_4.text().lower() == '最大值':
                    max_idx = np.argmax(y_filtered)
                    x_filtered = x_filtered[max_idx:]
                    y_filtered = y_filtered[max_idx:]
                    # 已删除时间偏移调整代码
            
            # 处理最小值条件 - 保留到最小值点为止的数据
            if self.lineEdit_3.text():
                if self.lineEdit_3.text().lower() == '最小值':
                    min_idx = np.argmin(y_filtered)
                    x_filtered = x_filtered[:min_idx+1]
                    y_filtered = y_filtered[:min_idx+1]
            
            # 处理百分比范围条件
            if self.lineEdit_4.text() and self.lineEdit_4.text().lower() != '最大值':
                try:
                    y_max = float(self.lineEdit_4.text()) * vrange * 0.01
                    mask = (y_filtered <= y_max)
                    x_filtered = x_filtered[mask]
                    y_filtered = y_filtered[mask]
                except ValueError:
                    pass
            
            if self.lineEdit_3.text() and self.lineEdit_3.text().lower() != '最小值':
                try:
                    y_min = float(self.lineEdit_3.text()) * vrange * 0.01
                    mask = (y_filtered >= y_min)
                    x_filtered = x_filtered[mask]
                    y_filtered = y_filtered[mask]
                except ValueError:
                    pass
            
            # 保留您最初的调试信息
            print('def data_range')
            if len(y_filtered) > 0:
                print('y 起始值:', y_filtered[0])
            if len(x_filtered) > 0:
                print('x 起始值:', x_filtered[0])

            return x_filtered, y_filtered
        except Exception as e:
            print(f"数据范围筛选时出错: {str(e)}")
            return np.array([]), np.array([])

    def calculate_decay(self, item=None, show_progress=True):
        """计算衰减参数 - 支持有参数和无参数调用
        show_progress: 是否显示进度对话框，批量处理时不显示
        """
        progress = None
        if show_progress:
            # 创建进度对话框 - 仅当需要显示时创建
            progress = QtWidgets.QProgressDialog("正在计算衰减参数...", "取消", 0, 0, self)
            progress.setWindowTitle("处理中")
            progress.setWindowModality(QtCore.Qt.WindowModal)
            progress.setMinimumDuration(0)  # 立即显示
            progress.show()
            
            # 强制显示进度对话框
            progress.setValue(0)
            
            # 处理事件，确保界面更新
            QtWidgets.QApplication.processEvents()
        
        try:
            # 如果没有传递item参数，使用当前选中的项目
            if item is None:
                current_item = self.treeWidget.currentItem()
                if current_item is None:
                    # 如果没有选中的项目，尝试使用第一个项目
                    if self.treeWidget.topLevelItemCount() > 0:
                        current_item = self.treeWidget.topLevelItem(0)
                    else:
                        QMessageBox.warning(self, "警告", "没有可处理的文件")
                        if progress:
                            progress.close()
                        return False
                file_path = current_item.data(0, QtCore.Qt.UserRole)
            else:
                # 确保item是QTreeWidgetItem类型，而不是布尔值
                if isinstance(item, bool):
                    # 如果是布尔值，使用当前选中的项目
                    current_item = self.treeWidget.currentItem()
                    if current_item is None:
                        if self.treeWidget.topLevelItemCount() > 0:
                            current_item = self.treeWidget.topLevelItem(0)
                        else:
                            QMessageBox.warning(self, "警告", "没有可处理的文件")
                            if progress:
                                progress.close()
                            return False
                    file_path = current_item.data(0, QtCore.Qt.UserRole)
                else:
                    file_path = item.data(0, QtCore.Qt.UserRole)
            
            file_name = os.path.basename(file_path)
            
            if progress:
                # 更新进度显示
                progress.setLabelText(f"正在处理: {file_name}\n请稍候...")
                progress.repaint()  # 强制重绘
                QtWidgets.QApplication.processEvents()
            
            print(f"处理文件: {file_path}")
            
            self.file_path = file_path

            if progress:
                progress.setLabelText("清理图形...")
                progress.repaint()
                QtWidgets.QApplication.processEvents()
            
            self.clear_figures()
            
            # 重置相关变量
            self.result = None
            self.lifetime = None

            if progress:
                progress.setLabelText("加载文件数据...")
                progress.repaint()
                QtWidgets.QApplication.processEvents()
            
            if file_path in self.file_data_cache:
                del self.file_data_cache[file_path]
                
            if not self.load_file_data(file_path):
                QMessageBox.warning(self, "错误", f"无法加载文件数据: {os.path.basename(file_path)}")
                if progress:
                    progress.close()
                return False
            
            data = self.file_data_cache[file_path]
            print(f"数据形状: {data.shape}, 数据类型: {data.dtype}")
            
            # 数据验证
            if len(data.shape) != 2 or data.shape[1] < 2:
                QMessageBox.warning(self, "错误", f"数据格式不正确，期望至少2列，实际形状: {data.shape}")
                if progress:
                    progress.close()
                return False
            
            if len(data) == 0:
                QMessageBox.warning(self, "错误", "文件为空或没有有效数据")
                if progress:
                    progress.close()
                return False
            
            # 确保数据为数值类型
            if data.dtype.kind not in 'buifc':  # 检查是否为数值类型
                data = data.astype(float)
            
            t_data = data[:, 0]  # 第一列作为x轴数据
            y_data = data[:, 1]  # 第二列作为y轴数据
            
            # 安全的打印语句
            try:
                print(f"数据范围 - x: [{np.min(t_data):.3f}, {np.max(t_data):.3f}], y: [{np.min(y_data):.3f}, {np.max(y_data):.3f}]")
            except:
                print(f"数据范围 - x: [{np.min(t_data)}, {np.max(t_data)}], y: [{np.min(y_data)}, {np.max(y_data)}]")
            
            if progress:
                progress.setLabelText("绘制原始数据...")
                progress.repaint()
                QtWidgets.QApplication.processEvents()
            
            ax = self.figure.add_subplot(111)
            x_label = self.comboBox_7.currentText() if self.comboBox.currentText() else "X轴"
            
            ax.plot(t_data, y_data, 'b-', linewidth=1)
            ax.set_xlabel(f'时间/{x_label}')
            ax.set_ylabel("相对强度")
            ax.set_title(f'原始数据 - {os.path.basename(file_path)}')
            ax.grid(True)
            
            self.figure.tight_layout()
            self.canvas.draw()
            
            if progress:
                progress.setLabelText("进行曲线拟合...\n这可能需要一些时间")
                progress.repaint()
                QtWidgets.QApplication.processEvents()
            
            if show_progress:
                return self._start_async_fitting(data, file_path, progress)

            self._perform_fitting(data, file_path)
            if progress:
                progress.close()
            return True
            
        except Exception as e:
            if progress:
                progress.close()
            error_msg = f"处理文件时出错: {str(e)}"
            print(error_msg)
            traceback.print_exc()
            QMessageBox.critical(self, "错误", error_msg)
            return False

    def _set_fitting_ui_busy(self, busy):
        """异步拟合期间，限制可能触发重复拟合的交互。"""
        self.pushButton_6.setEnabled(not busy)
        self.treeWidget.setEnabled(not busy)

    def _start_async_fitting(self, data, file_path, progress):
        """启动后台拟合线程（仅用于前台交互流程）。"""
        try:
            task = self._build_fit_task(data, file_path)
            if task is None:
                if progress:
                    progress.close()
                return False

            self._fit_request_id += 1
            request_id = self._fit_request_id
            self._fit_progress_dialog = progress

            self._set_fitting_ui_busy(True)

            thread = QtCore.QThread(self)
            worker = FitWorker(task)
            worker.moveToThread(thread)

            thread.started.connect(worker.run)
            worker.finished.connect(lambda output, rid=request_id: self._on_async_fitting_finished(rid, output))
            worker.failed.connect(lambda err, rid=request_id: self._on_async_fitting_failed(rid, err))
            worker.cancelled.connect(lambda rid=request_id: self._on_async_fitting_cancelled(rid))

            worker.finished.connect(thread.quit)
            worker.failed.connect(thread.quit)
            worker.cancelled.connect(thread.quit)
            worker.finished.connect(worker.deleteLater)
            worker.failed.connect(worker.deleteLater)
            worker.cancelled.connect(worker.deleteLater)
            thread.finished.connect(thread.deleteLater)

            self._fit_thread = thread
            self._fit_worker = worker
            if progress is not None:
                progress.canceled.connect(self._cancel_async_fitting)
            thread.start()
            return True
        except Exception as e:
            if progress:
                progress.close()
            self._set_fitting_ui_busy(False)
            error_msg = f"启动后台拟合失败: {str(e)}"
            print(error_msg)
            traceback.print_exc()
            QMessageBox.critical(self, "错误", error_msg)
            return False

    def _finish_async_fitting_ui(self):
        """异步拟合完成后恢复UI状态并关闭进度框。"""
        self._set_fitting_ui_busy(False)
        if self._fit_progress_dialog:
            self._fit_progress_dialog.close()
            self._fit_progress_dialog = None

    def _on_async_fitting_finished(self, request_id, fit_output):
        """后台拟合完成回调（主线程）。"""
        try:
            # 忽略过期结果，避免旧任务覆盖新选择
            if request_id != self._fit_request_id:
                return
            self._apply_fit_output(fit_output)
        finally:
            if request_id == self._fit_request_id:
                self._fit_thread = None
                self._fit_worker = None
                self._finish_async_fitting_ui()

    def _on_async_fitting_failed(self, request_id, error_trace):
        """后台拟合失败回调（主线程）。"""
        try:
            if request_id != self._fit_request_id:
                return
            print("后台拟合异常:")
            print(error_trace)
            QMessageBox.critical(self, "错误", "拟合计算时出错，请查看控制台日志")
        finally:
            if request_id == self._fit_request_id:
                self._fit_thread = None
                self._fit_worker = None
                self._finish_async_fitting_ui()

    def _on_async_fitting_cancelled(self, request_id):
        """后台拟合取消回调（主线程）。"""
        try:
            if request_id != self._fit_request_id:
                return
            print("拟合已取消")
        finally:
            if request_id == self._fit_request_id:
                self._fit_thread = None
                self._fit_worker = None
                self._finish_async_fitting_ui()

    def _cancel_async_fitting(self):
        """用户关闭/取消进度动画时，请求后台拟合停止。"""
        if self._fit_progress_dialog:
            self._fit_progress_dialog.setLabelText("正在取消拟合...")
            self._fit_progress_dialog.repaint()
            QtWidgets.QApplication.processEvents()
        if self._fit_worker is not None:
            self._fit_worker.request_cancel()

    def _build_fit_task(self, data, file_path):
        """从当前UI状态构建拟合任务数据。"""
        # =========================
        # 0️⃣ 原始数据
        # =========================
        Origional_x = data[:, 0]
        Origional_y_data = data[:, 1]

        # =========================
        # 1️⃣ 当前模型
        # =========================
        model = self.comboBox.currentText()
        self.current_model_type = model
        print(f"使用模型: {model}")

        # =========================
        # 2️⃣ 数据筛选 - 保存时间范围内的数据作为背景
        # =========================
        x_for_display = np.asarray(Origional_x)
        y_for_display = np.asarray(Origional_y_data)

        x_min, x_max = self._get_time_thresholds_in_seconds()
        if x_min is not None:
            mask = x_for_display >= x_min
            x_for_display = x_for_display[mask]
            y_for_display = y_for_display[mask]

        if x_max is not None:
            mask = x_for_display <= x_max
            x_for_display = x_for_display[mask]
            y_for_display = y_for_display[mask]

        x, y_data = self.data_range(Origional_x, Origional_y_data)
        if len(x) < 2:
            QMessageBox.warning(self, "警告", "筛选后的数据点不足")
            return None

        # =========================
        # 3️⃣ 智能初始值选择
        # =========================
        temp_fitter = MATLABCurveFitter()
        smart_p0 = temp_fitter._get_default_start_point(model, x, y_data)

        if model == '单指数':
            current_start = self._read_param_block([
                self.lineEdit_6,
                self.lineEdit_14,
                self.lineEdit_36
            ])
        else:
            current_start = self._read_param_block([
                self.lineEdit_6,
                self.lineEdit_14,
                self.lineEdit_15,
                self.lineEdit_25,
                self.lineEdit_36
            ])

        print(f"为当前数据计算智能初始值: {smart_p0}")
        self._auto_fill_start_point(model, smart_p0)

        is_user_set_values = any(v is not None for v in current_start)
        if not is_user_set_values:
            print(f"UI初始值为空，使用智能初始值: {smart_p0}")
        else:
            if self._should_update_start_point(current_start, smart_p0, x, y_data):
                print(f"检测到当前初始值不适合新数据，使用智能初始值: {smart_p0}")
            else:
                print(f"使用用户设置的初始值: {current_start}")

        # =========================
        # 4️⃣ 从 UI 读取 Start / Lower / Upper
        # =========================
        if model == '单指数':
            start = self._read_param_block([self.lineEdit_6, self.lineEdit_14, self.lineEdit_36])
            lower = self._read_param_block([self.lineEdit_11, self.lineEdit_12, self.lineEdit_37])
            upper = self._read_param_block([self.lineEdit_16, self.lineEdit_17, self.lineEdit_38])
        else:
            start = self._read_param_block([
                self.lineEdit_6, self.lineEdit_14, self.lineEdit_15, self.lineEdit_25, self.lineEdit_36
            ])
            lower = self._read_param_block([
                self.lineEdit_11, self.lineEdit_12, self.lineEdit_13, self.lineEdit_34, self.lineEdit_37
            ])
            upper = self._read_param_block([
                self.lineEdit_16, self.lineEdit_17, self.lineEdit_18, self.lineEdit_35, self.lineEdit_38
            ])
        enable_second_fit, second_fit_tau_multiplier = self._get_secondary_fit_settings()

        return {
            'file_path': file_path,
            'model': model,
            'x_for_display': x_for_display,
            'y_for_display': y_for_display,
            'x_for_fitting': x,
            'y_for_fitting': y_data,
            'start': start,
            'lower': lower,
            'upper': upper,
            'enable_second_fit': enable_second_fit,
            'second_fit_tau_multiplier': second_fit_tau_multiplier,
            'fit_options': self.get_fit_options(),
            'robust_text': self.comboBox_8.currentText(),
            'algorithm_text': self.comboBox_9.currentText()
        }

    def _apply_fit_output(self, fit_output):
        """将拟合输出回填到UI。"""
        result = fit_output['result']
        self.result = result

        if result.success:
            fitter = MATLABCurveFitter()
            self._handle_fit_result(
                result,
                fit_output['model'],
                fit_output['x_for_display'],
                fit_output['y_for_display'],
                fit_output['x_for_fitting'],
                fit_output['y_for_fitting'],
                fit_output.get('x_for_curve'),
                fitter
            )
        else:
            QMessageBox.warning(self, "警告", f"拟合失败: {result.message}")
    
    def _sync_constraint_ui_with_model(self, model):
        """
        不依赖 layout 名字，通过控件反查所在布局行，彻底隐藏 A2 / τ2
        """

        # 任选一个双指数控件，反查 layout
        ref_widget = self.lineEdit_15  # A2 初始值，一定存在
        layout = ref_widget.parentWidget().layout()

        if layout is None:
            print("⚠️ 未找到系数约束的 layout")
            return

        # 找 A2 / τ2 所在的行号（只算一次也行，这里直接算）
        rows_to_hide = set()

        for w in [
            self.lineEdit_15, self.lineEdit_13, self.lineEdit_18,  # A2
            self.lineEdit_25, self.lineEdit_34, self.lineEdit_35   # τ2
        ]:
            idx = layout.indexOf(w)
            if idx >= 0:
                row, col, rowspan, colspan = layout.getItemPosition(idx)
                rows_to_hide.add(row)

        if model == '单指数':
            for row in rows_to_hide:
                layout.setRowVisible(row, False)

            # 清空双指数参数，防止残留
            for le in [
                self.lineEdit_15, self.lineEdit_13, self.lineEdit_18,
                self.lineEdit_25, self.lineEdit_34, self.lineEdit_35
            ]:
                le.clear()

        elif model == '双指数':
            for row in rows_to_hide:
                layout.setRowVisible(row, True)



    def _perform_fitting(self, data, file_path):
        """执行拟合计算的内部方法（支持自动初始值 + UI 约束）"""
        try:
            task = self._build_fit_task(data, file_path)
            if task is None:
                return
            fit_output = run_fit_task(task)
            self._apply_fit_output(fit_output)

        except Exception as e:
            error_msg = f"拟合计算时出错: {str(e)}"
            print(error_msg)
            traceback.print_exc()
            QMessageBox.critical(self, "错误", error_msg)


    def _setup_fitter_options(self, fitter):
        """设置拟合器选项"""
        try:
            robust_mapping = {
                'Off': RobustMethod.OFF,
                'LAR': RobustMethod.LAR,
                'Bisquare': RobustMethod.BISQUARE
            }

            algorithm_mapping = {
                'Levenberg-Marquardt': Algorithm.LEVENBERG_MARQUARDT,
                'Trust-Region': Algorithm.TRUST_REGION,
            }

            robust_method = robust_mapping.get(self.comboBox_8.currentText(), RobustMethod.OFF)
            algorithm_method = algorithm_mapping.get(self.comboBox_9.currentText(), Algorithm.LEVENBERG_MARQUARDT)
            
            fit_options = self.get_fit_options()
            
            fitter.set_options(
                Robust=robust_method,
                Algorithm=algorithm_method,
                **fit_options
            )
            
            return fitter
        except Exception as e:
            print(f"设置拟合器选项时出错: {str(e)}")
            return fitter

    def _handle_fit_result(self, result, models_to_test, x_for_display, y_for_display, x_for_fitting, y_for_fitting, x_for_curve, fitter):
        """处理拟合结果"""
        try:
            # 确保模型类型信息被保存
            self.current_model_type = models_to_test
            
            if models_to_test == '单指数':
                a, b, c = result.params
                lifetime = b if b != 0 else float('inf')
                self.lifetime = lifetime
                self.lineEdit_27.setText(f"{a:.5e}")   # A1
                self.lineEdit.setText(f"{lifetime * self.tau_unit_11:.5e}")  # τ1（按单位显示）
                self.lineEdit_29.setText(f"{c:.5e}")   # I0
                self.lineEdit_26.clear()               # A2
                self.lineEdit_28.clear()               # τ2
                print(f"单指数拟合结果: lifetime = {lifetime:.5e}")

            elif models_to_test == '双指数':
                # 修复参数排序逻辑，确保 t1 < t2
                a, t1, c, t2, e = result.params
                
                # 如果 t2 < t1，交换两个分量的参数
                if t2 < t1:
                    # 交换快速分量和慢速分量
                    a, c = c, a  # 交换振幅
                    t1, t2 = t2, t1  # 交换时间常数
                    print(f"参数已交换: t1={t1:.5e}, t2={t2:.5e}")
                
                # 更新界面显示（确保 t1 < t2）
                self.lineEdit_27.setText(f"{a:.5e}")  # 快速分量振幅
                self.lineEdit.setText(f"{t1 * self.tau_unit_11:.5e}")    # 快速衰减时间常数 (t1, 按单位显示)
                self.lineEdit_26.setText(f"{c:.5e}")  # 慢速分量振幅
                self.lineEdit_28.setText(f"{t2 * self.tau_unit_12:.5e}") # 慢速衰减时间常数 (t2, 按单位显示)
                self.lineEdit_29.setText(f"{e:.5e}")  # 基线
                
                # 同时更新result.params以确保保存时使用正确的顺序
                result.params = [a, t1, c, t2, e]
                
                print(f"双指数拟合结果: t1 = {t1:.5e}, t2 = {t2:.5e}, 振幅1 = {a:.5e}, 振幅2 = {c:.5e}")

            # 更新统计信息
            self.lineEdit_30.setText(f"{result.sse:.5f}")
            self.lineEdit_7.setText(f"{result.rsquare:.5f}")
            self.lineEdit_31.setText(f"{result.adjrsquare:.5f}")
            self.lineEdit_32.setText(f"{result.rmse:.5f}")
            self.lineEdit_33.setText(f"{'是' if result.success else '否'}")

            # 绘制结果 - 在显示数据范围上绘制拟合曲线
            model_def = fitter._get_model_definition(models_to_test)
            if model_def:
                curve_x = x_for_display if x_for_curve is None else np.asarray(x_for_curve)
                y_pred_full = model_def['function'](curve_x, *result.params)
                self.fig1fig2_plot(x_for_display, y_for_display, x_for_fitting, y_for_fitting, y_pred_full, models_to_test, curve_x)
            
            print("拟合完成:", result)
            
        except Exception as e:
            error_msg = f"处理拟合结果时出错: {str(e)}"
            print(error_msg)
            traceback.print_exc()

    def fig1fig2_plot(self, Origional_x, Origional_y_data, x, y_data, y_pred, models_to_test, x_pred=None):
        """绘制拟合结果图形
        
        参数:
            x_pred: 用于绘制拟合曲线的x坐标（如果为None，使用x）
        """
        try:
            # 如果没有提供x_pred，则使用拟合数据的x坐标
            if x_pred is None:
                x_pred = x
            
            # 绘制图形
            self.figure2.clear()
            ax2 = self.figure2.add_subplot(111)

            # 绘制所有原始数据点
            ax2.scatter(Origional_x, Origional_y_data, alpha=0.3, color='gray', s=5, label='原始数据')
            # 高亮显示用于拟合的数据点
            ax2.scatter(x, y_data, alpha=0.8, color='blue', s=10, label='拟合范围数据')
            
            # 绘制完整拟合曲线
            ax2.plot(x_pred, y_pred, 'r-', label=f'{models_to_test}拟合', linewidth=2)
            ax2.legend()
            ax2.set_title(f'{models_to_test}模型拟合结果')
            ax2.set_xlabel('时间')
            ax2.set_ylabel("相对强度")
            ax2.grid(True, alpha=0.3)
            
            # 自动调整布局
            self.figure2.tight_layout()
            self.canvas2.draw()

            # 绘制残差（仅在拟合范围内）
            self.figure3.clear()
            ax3 = self.figure3.add_subplot(111)
            
            ax3.plot(x, self.result.residuals, 'ro-', alpha=0.6, markersize=2)
            ax3.axhline(y=0, color='k', linestyle='--')
            ax3.set_title('残差图')
            ax3.set_xlabel('x')
            ax3.set_ylabel('残差')
            ax3.grid(True, alpha=0.3)
            
            # 自动调整布局
            self.figure3.tight_layout()
            self.canvas3.draw()
            
            self.calculate_lifetime()
        except Exception as e:
            print(f"绘图时出错: {str(e)}")

    def _is_start_point_empty(self, model):
        """判断初始值输入框是否全空（避免覆盖用户手动输入）"""
        if model == '双指数':
            edits = [self.lineEdit_6, self.lineEdit_14, self.lineEdit_15, self.lineEdit_25, self.lineEdit_36]
        elif model == '单指数':
            # 单指数参数：a, b(τ), c(I0) —— 你按你UI实际对应改一下
            edits = [self.lineEdit_6, self.lineEdit_14, self.lineEdit_36]
        else:
            return True
        return all(le.text().strip() == "" for le in edits)

    def _should_update_start_point(self, current_start, smart_start, x, y):
        """判断是否应该更新初始值 - 在切换文件时更积极地更新"""
        # 如果UI为空，总是更新
        if all(v is None for v in current_start):
            return True

        # 如果智能初始值明显更好，建议更新
        try:
            # 计算当前初始值的"合理性"评分
            current_score = self._evaluate_start_point_quality(current_start, x, y)
            smart_score = self._evaluate_start_point_quality(smart_start, x, y)

            # 如果智能初始值明显更好（评分差距>10%，比原来更敏感），更新
            if smart_score > current_score * 1.1:
                return True

            # 如果当前初始值包含不合理的参数（负值、极小值等），更新
            for val in current_start:
                if val is not None and (val <= 0 or val < 1e-8):
                    return True

            # 切换文件时，如果参数差异较大，也更新
            # 检查参数差异 - 进一步降低阈值以更敏感地检测变化
            significant_diff = False
            for curr, smart in zip(current_start, smart_start):
                if curr is not None and smart is not None:
                    # 如果参数差异超过20%（进一步降低阈值），认为需要更新
                    if abs(curr - smart) / max(abs(smart), 1e-10) > 0.2:
                        significant_diff = True
                        break

            if significant_diff:
                return True

        except:
            # 如果评估失败，保守起见更新
            return True

        return False

    def _evaluate_start_point_quality(self, start_point, x, y):
        """评估初始值的质量（基于物理合理性和数据匹配度）"""
        if not start_point or all(v is None for v in start_point):
            return 0

        score = 0
        try:
            # 基本合理性检查
            for val in start_point:
                if val is not None:
                    if val > 0 and val < 1e10:  # 合理范围
                        score += 10
                    elif val <= 0:  # 负值或零
                        score -= 50

            # 数据范围匹配度
            y_range = np.max(y) - np.min(y)
            x_range = x[-1] - x[0]

            # 检查振幅参数是否在合理范围内
            if len(start_point) >= 3:  # 单指数或双指数
                amplitude_params = [start_point[0]]  # 第一个参数通常是振幅
                if len(start_point) >= 5:  # 双指数
                    amplitude_params.append(start_point[2])  # 第二个振幅

                for amp in amplitude_params:
                    if amp is not None:
                        if 0.1 * y_range <= amp <= 5 * y_range:
                            score += 20
                        elif amp < 0.01 * y_range or amp > 10 * y_range:
                            score -= 30

            # 检查时间常数是否在合理范围内
            time_params = []
            if len(start_point) == 3:  # 单指数: [amp, tau, baseline]
                if start_point[1] is not None and start_point[1] > 0:
                    tau = start_point[1]  # tau 直接就是第二个参数
                    time_params.append(tau)
            elif len(start_point) == 5:  # 双指数: [amp1, tau1, amp2, tau2, baseline]
                time_params.extend([start_point[1], start_point[3]])  # tau1, tau2

            for tau in time_params:
                if tau is not None:
                    if 0.001 * x_range <= tau <= 10 * x_range:
                        score += 15
                    elif tau < 0.0001 * x_range or tau > 100 * x_range:
                        score -= 25

        except:
            score -= 10  # 评估出错，降低评分

        return max(0, score)  # 确保评分不为负

    def reset_start_point(self):
        """重置初始值为智能推荐值"""
        try:
            # 获取当前数据
            if not hasattr(self, 'current_data') or self.current_data is None:
                QMessageBox.warning(self, "警告", "请先加载数据文件")
                return
            
            # 获取筛选后的数据
            data = self.current_data
            Origional_x = data[:, 0]
            Origional_y_data = data[:, 1]
            x, y_data = self.data_range(Origional_x, Origional_y_data)
            
            if len(x) < 2:
                QMessageBox.warning(self, "警告", "筛选后的数据点不足")
                return
            
            # 获取当前模型
            model = self.comboBox.currentText()
            
            # 创建拟合器并计算智能初始值
            fitter = MATLABCurveFitter()
            fitter = self._setup_fitter_options(fitter)
            smart_p0 = fitter._get_default_start_point(model, x, y_data)
            
            # 填充到UI
            self._auto_fill_start_point(model, smart_p0)
            
            QMessageBox.information(self, "成功", f"已重置{model}模型的初始值为智能推荐值")
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"重置初始值失败: {str(e)}")

    def _read_param_block(self, edits):
        """读取一组 lineEdit，支持 inf / -inf / 空"""
        vals = []
        for le in edits:
            text = le.text().strip()
            if text == "":
                vals.append(None)
            elif text.lower() == "inf":
                vals.append(np.inf)
            elif text.lower() == "-inf":
                vals.append(-np.inf)
            else:
                try:
                    vals.append(float(text))
                except ValueError:
                    vals.append(None)
        return vals

    def _auto_fill_start_point(self, model, p0):
        """把自动初始值写回UI（只负责写初始值，不管上下限）"""
        if model == '双指数' and len(p0) >= 5:
            # 使用5位科学计数法显示
            self.lineEdit_6.setText(f"{p0[0]:.5e}")   # A1
            self.lineEdit_14.setText(f"{p0[1]:.5e}")  # τ1
            self.lineEdit_15.setText(f"{p0[2]:.5e}")  # A2
            self.lineEdit_25.setText(f"{p0[3]:.5e}")  # τ2
            self.lineEdit_36.setText(f"{p0[4]:.5e}")  # I0
        elif model == '单指数' and len(p0) >= 3:
            self.lineEdit_6.setText(f"{p0[0]:.5e}")   # A
            self.lineEdit_14.setText(f"{p0[1]:.5e}")  # τ 或 b
            self.lineEdit_36.setText(f"{p0[2]:.5e}")  # I0

    def calculate_lifetime(self):
        """使用线性插值和两侧外推方法计算温度，基于平均寿命"""
        try:
            # 检查是否有拟合结果
            if self.result is None or not self.result.success:
                self.textEdit("警告：请先进行成功的拟合计算")
                self._clear_oxidation_metrics()
                return
            
            # 检查标定数据（修改为检查是否为空列表）
            if not hasattr(self, 'temperature_calibration') or not self.temperature_calibration:
                self.textEdit("警告：未设置温度标定数据")
                self._clear_oxidation_metrics()
                return
            
            if not hasattr(self, 'lifetime_calibration') or not self.lifetime_calibration:
                self.textEdit("警告：未设置寿命标定数据")
                self._clear_oxidation_metrics()
                return
            
            # 计算平均寿命
            avg_lifetime = self.calculate_average_lifetime()
            if avg_lifetime is None:
                self.textEdit("警告：无法计算平均寿命")
                self._clear_oxidation_metrics()
                return
            
            # 使用线性插值和两侧外推计算温度（这会设置self.temperature）
            temperature = self.calculate_temperature_by_interpolation(avg_lifetime)
            if temperature is None:
                self.textEdit("警告：温度计算失败")
                self._clear_oxidation_metrics()
                return
            
            # 显示结果
            self.display_temperature_result(temperature, avg_lifetime)
            
        except Exception as e:
            self.textEdit(f"计算寿命温度时出错: {str(e)}")
            self._clear_oxidation_metrics()
            traceback.print_exc()

    def calculate_average_lifetime(self):
        """计算平均寿命 (a*t1^2 + c*t2^2) / (a*t1 + c*t2)"""
        try:
            model_type = self.comboBox.currentText()
            
            if model_type == '单指数':
                # 单指数模型：寿命就是衰减常数
                tau = self.result.params[1]  # b参数
                print(f"单指数寿命: {tau:.5e} s")
                return tau
                
            elif model_type == '双指数':
                # 双指数模型：计算加权平均寿命
                # 参数顺序: [a, t1, c, t2, e]
                a = self.result.params[0]  # 快速分量振幅
                t1 = self.result.params[1]  # 快速衰减时间常数
                c = self.result.params[2]  # 慢速分量振幅
                t2 = self.result.params[3]  # 慢速衰减时间常数
                
                # 计算平均寿命: (a*t1^2 + c*t2^2) / (a*t1 + c*t2)
                numerator = a * (t1 ** 2) + c * (t2 ** 2)
                denominator = a * t1 + c * t2
                
                if denominator == 0:
                    print("警告：分母为零，无法计算平均寿命")
                    return None
                    
                avg_lifetime = numerator / denominator
                print(f"双指数平均寿命计算:")
                print(f"  a = {a:.5e}, t1 = {t1:.5e} s")
                print(f"  c = {c:.5e}, t2 = {t2:.5e} s")
                print(f"  分子: {numerator:.5e}, 分母: {denominator:.5e}")
                print(f"  平均寿命: {avg_lifetime:.5e} s")
                return avg_lifetime
                
            else:
                print(f"不支持的模型类型: {model_type}")
                return None
                
        except Exception as e:
            print(f"计算平均寿命时出错: {str(e)}")
            traceback.print_exc()
            return None

    def calculate_temperature_by_interpolation(self, lifetime):
        """使用线性插值和两侧外推计算温度，并保存到self.temperature"""
        try:
            # 确保标定数据是numpy数组（单位：秒s）
            lifetimes = np.array(self.lifetime_calibration)
            temperatures = np.array(self.temperature_calibration)
            
            # 对数据进行排序
            sorted_indices = np.argsort(lifetimes)
            sorted_lifetimes = lifetimes[sorted_indices]
            sorted_temperatures = temperatures[sorted_indices]
            
            # 使用科学计数法显示（单位：秒）
            print(f"标定数据 - 寿命范围: [{min(sorted_lifetimes):.5e}, {max(sorted_lifetimes):.5e}] s")
            print(f"标定数据 - 温度范围: [{min(sorted_temperatures):.2f}, {max(sorted_temperatures):.2f}] °C")
            print(f"当前寿命: {lifetime:.5e} s")
            
            # 检查寿命值是否在标定数据范围内
            if lifetime <= sorted_lifetimes[0]:
                # 左侧外推：使用前两个点
                x0, x1 = sorted_lifetimes[0], sorted_lifetimes[1]
                y0, y1 = sorted_temperatures[0], sorted_temperatures[1]
                slope = (y1 - y0) / (x1 - x0)
                temperature = y0 + slope * (lifetime - x0)
                method = "左侧外推"
                
            elif lifetime >= sorted_lifetimes[-1]:
                # 右侧外推：使用最后两个点
                x0, x1 = sorted_lifetimes[-2], sorted_lifetimes[-1]
                y0, y1 = sorted_temperatures[-2], sorted_temperatures[-1]
                slope = (y1 - y0) / (x1 - x0)
                temperature = y1 + slope * (lifetime - x1)
                method = "右侧外推"
                
            else:
                # 线性插值：在范围内
                temperature = np.interp(lifetime, sorted_lifetimes, sorted_temperatures)
                method = "线性插值"
            
            # 保存温度到self.temperature
            self.temperature = temperature
            
            print(f"计算方法: {method}, 计算温度: {temperature:.2f} °C")
            return temperature
            
        except Exception as e:
            print(f"温度插值计算时出错: {str(e)}")
            traceback.print_exc()
            return None

    def display_temperature_result(self, temperature, avg_lifetime):
        """显示温度计算结果，保持现有textEdit格式"""
        try:
            model_type = self.comboBox.currentText()
            
            # 构建显示文本
            result_text = f"""
            <div align="center">
                <span style="font-size:36pt; font-weight:600; color:#aa0000;">{temperature:.2f} ℃</span>
            </div>
            """
            
            self.textEdit.setHtml(result_text)
            
            # 同时在控制台输出详细信息
            print(f"温度计算结果:")
            print(f"  模型类型: {model_type}")
            print(f"  平均寿命: {avg_lifetime:.6f}")
            print(f"  计算温度: {temperature:.2f} °C")
            print(f"  拟合优度 R²: {self.result.rsquare:.4f}")
            self.update_oxidation_metrics()
            
        except Exception as e:
            print(f"显示温度结果时出错: {str(e)}")
            # 如果出错，至少显示温度值
            self.textEdit.setHtml(
                f'<div align="center"><span style="font-size:36pt; font-weight:600; color:#aa0000;">{temperature:.2f} ℃</span></div>'
            )
            self.update_oxidation_metrics()

    def _on_oxidation_time_changed(self):
        """时间输入变化后，尝试刷新氧化参数。"""
        self.update_oxidation_metrics()

    def _clear_oxidation_metrics(self):
        """清空氧化参数显示。"""
        self.lineEdit_40.clear()
        self.lineEdit_41.clear()
        self.lineEdit_42.clear()

    def update_oxidation_metrics(self):
        """
        根据拟合温度和输入时间计算：
        厚度(um) = 4.18*10E-4*T(K)*(0.44+ln(t_h))
        速率(um/h) = 厚度 / t_h
        孔隙率(%) = -3.58*10E-4*T(K)*(31+ln(t_h)) + 30
        """
        try:
            temp_c = getattr(self, 'temperature', None)
            time_text = self.lineEdit_39.text().strip()

            if temp_c is None or time_text == "":
                self._clear_oxidation_metrics()
                return

            try:
                time_h = float(time_text)
            except ValueError:
                self._clear_oxidation_metrics()
                return

            if time_h <= 0:
                self._clear_oxidation_metrics()
                return

            temp_k = float(temp_c) + 273.15
            ln_time = np.log(time_h)

            thickness_um = 4.18 * 10**-4 * temp_k * (0.44 + ln_time)
            oxidation_rate = thickness_um / time_h
            porosity = -3.58 * 10**-4 * temp_k * (31 + ln_time) + 30

            self.lineEdit_40.setText(f"{thickness_um:.2f}")
            self.lineEdit_41.setText(f"{oxidation_rate:.2f}")
            self.lineEdit_42.setText(f"{porosity:.2f}")

        except Exception as e:
            print(f"更新氧化参数时出错: {str(e)}")
            self._clear_oxidation_metrics()
    
    def validate_folder_path(self, path):
        """验证文件夹路径是否有效"""
        return QDir(path).exists()

    def clear_figures(self):
        """清除所有图形"""
        for figure in [self.figure, self.figure2, self.figure3]:
            figure.clear()

    def cleanup(self):
        """清理资源"""
        if self._fit_thread and self._fit_thread.isRunning():
            self._fit_thread.quit()
            self._fit_thread.wait(1500)
        self.file_data_cache.clear()
        self.clear_figures()
        self.save_current_settings()

    def closeEvent(self, event):
        """重写关闭事件"""
        try:
            self.cleanup()
            event.accept()
        except Exception as e:
            print(f"关闭程序时出错: {str(e)}")
            event.accept()

if __name__ == '__main__':
    try:
        QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
        app = QApplication(sys.argv)
        myWin = mainWindow()
        myWin.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(f"程序启动时出错: {str(e)}")
        traceback.print_exc()
