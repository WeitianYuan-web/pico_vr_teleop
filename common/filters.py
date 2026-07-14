"""滤波工具：EMA、固定 alpha、加权滑动平均。"""

from __future__ import annotations

import numpy as np

from common.math_quat import slerp_quat_wxyz


def time_based_alpha(dt: float, tau: float) -> float:
    """将时间常数 tau（秒）换算为当前 dt 下的一阶平滑系数。"""
    if tau <= 0.0:
        return 1.0
    return 1.0 - float(np.exp(-dt / tau))


class EMAFilter:
    """基于时间常数 tau（秒）的一阶低通，每次 update 需传入真实 dt。"""

    def __init__(self, tau: float = 0.05):
        self.tau = tau
        self.value = None

    def reset(self) -> None:
        self.value = None

    def update(self, x: np.ndarray, dt: float) -> np.ndarray:
        if self.value is None:
            self.value = np.asarray(x, dtype=float).copy()
        else:
            alpha = time_based_alpha(dt, self.tau)
            self.value = alpha * x + (1.0 - alpha) * self.value
        return self.value


def lerp_position(prev: np.ndarray | None, raw: np.ndarray, alpha: float) -> np.ndarray:
    """固定 alpha 的位置一阶低通。"""
    alpha = float(np.clip(alpha, 0.0, 1.0))
    if prev is None or alpha >= 0.999:
        return np.asarray(raw, dtype=float).copy()
    return (1.0 - alpha) * prev + alpha * raw


def slerp_filter_quat(prev: np.ndarray | None, raw: np.ndarray, alpha: float) -> np.ndarray:
    """固定 alpha 的姿态 slerp 滤波（自动半球对齐）。"""
    alpha = float(np.clip(alpha, 0.0, 1.0))
    raw = np.asarray(raw, dtype=float).copy()
    if prev is None or alpha >= 0.999:
        return raw
    if np.dot(prev, raw) < 0.0:
        raw = -raw
    return slerp_quat_wxyz(prev, raw, alpha)


class WeightedMovingFilter:
    """
    /**
     * @brief 对固定维度向量做加权滑动平均
     * @param weights 窗口权重，和必须为 1
     * @param data_size 向量维度
     */
    """

    def __init__(self, weights: np.ndarray, data_size: int = 14):
        self._weights = np.asarray(weights, dtype=float)
        if not np.isclose(np.sum(self._weights), 1.0):
            raise ValueError("weights 之和必须为 1.0")
        self._window_size = len(self._weights)
        self._data_size = int(data_size)
        self._filtered_data = np.zeros(self._data_size, dtype=float)
        self._data_queue: list[np.ndarray] = []

    def add_data(self, new_data: np.ndarray) -> None:
        new_data = np.asarray(new_data, dtype=float)
        if new_data.shape != (self._data_size,):
            raise ValueError(f"期望 shape=({self._data_size},), 得到 {new_data.shape}")
        if self._data_queue and np.array_equal(new_data, self._data_queue[-1]):
            return
        if len(self._data_queue) >= self._window_size:
            self._data_queue.pop(0)
        self._data_queue.append(new_data.copy())
        if len(self._data_queue) < self._window_size:
            self._filtered_data = new_data.copy()
            return
        stacked = np.asarray(self._data_queue, dtype=float)
        self._filtered_data = stacked.T @ self._weights[::-1]

    @property
    def filtered_data(self) -> np.ndarray:
        return self._filtered_data

    def pin_indices(self, indices: slice | np.ndarray | list[int], values: np.ndarray) -> None:
        """
        /**
         * @brief 将指定维度钉死为给定值，并同步滑动窗口
         */
        """
        values = np.asarray(values, dtype=float).reshape(-1)
        self._filtered_data[indices] = values
        for item in self._data_queue:
            item[indices] = values
