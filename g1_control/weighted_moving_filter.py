"""加权滑动平均滤波（关节目标平滑）。"""

from __future__ import annotations

import numpy as np


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
