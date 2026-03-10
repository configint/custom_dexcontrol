"""Signal filtering utilities for robot command smoothing.

Adapted from omniteleop's MultiChannelFilter.  Provides per-component
configurable filtering (Butterworth low-pass, EMA, or none) that can be
layered on top of trajectory interpolation for additional smoothness.
"""

from __future__ import annotations

import numpy as np
from typing import Any, Dict, Optional

from scipy.signal import butter, sosfilt, sosfilt_zi


class ButterworthFilter:
    """Per-joint Butterworth low-pass filter."""

    def __init__(
        self,
        cutoff_freq: float = 10.0,
        order: int = 2,
        fs: float = 100.0,
    ) -> None:
        self.cutoff_freq = cutoff_freq
        self.order = order
        self.fs = fs

        nyquist = fs / 2.0
        normalized_cutoff = min(cutoff_freq / nyquist, 0.99)
        self.sos = butter(order, normalized_cutoff, btype="low", output="sos")
        self.filter_states: Dict[str, np.ndarray] = {}

    def filter_array(self, key: str, data: np.ndarray) -> np.ndarray:
        """Filter a 1-D array sample-by-sample, maintaining per-element state."""
        if not isinstance(data, np.ndarray):
            data = np.asarray(data, dtype=np.float64)
        if data.ndim == 0:
            data = data.reshape(1)

        filtered = np.empty_like(data)
        for i in range(len(data)):
            state_key = f"{key}_{i}"
            if state_key not in self.filter_states:
                self.filter_states[state_key] = sosfilt_zi(self.sos) * data[i]
            val, self.filter_states[state_key] = sosfilt(
                self.sos, [data[i]], zi=self.filter_states[state_key]
            )
            filtered[i] = val[0]
        return filtered

    def reset(self, key: Optional[str] = None) -> None:
        if key:
            to_remove = [k for k in self.filter_states if k.startswith(key)]
            for k in to_remove:
                del self.filter_states[k]
        else:
            self.filter_states.clear()


class MultiChannelFilter:
    """Per-component configurable filter bank.

    Initialisation from a plain dict (or ``None`` for pass-through)::

        config = {
            "default": {"type": "butterworth", "cutoff_freq": 8.0, "order": 2},
            "components": {
                "gripper": {"type": "ema", "alpha": 0.3},
            },
        }
        filt = MultiChannelFilter(config, control_rate=100.0)

    Then, every control-loop tick::

        smoothed = filt.apply(joint_pos)  # np.ndarray (n_joints,)
    """

    def __init__(
        self,
        filter_config: Optional[Dict[str, Any]] = None,
        control_rate: float = 100.0,
    ) -> None:
        self.control_rate = control_rate
        self._filter: Optional[Any] = None
        self._filter_type: str = "none"
        self._ema_alpha: float = 0.1
        self._prev_value: Optional[np.ndarray] = None

        if filter_config is None:
            return

        default_cfg = filter_config.get("default", {})
        self._filter_type = default_cfg.get("type", "none").lower()

        if self._filter_type == "butterworth":
            self._filter = ButterworthFilter(
                cutoff_freq=default_cfg.get("cutoff_freq", 10.0),
                order=default_cfg.get("order", 2),
                fs=control_rate,
            )
        elif self._filter_type == "ema":
            self._ema_alpha = default_cfg.get("alpha", 0.1)

    def apply(self, data: np.ndarray) -> np.ndarray:
        """Filter a 1-D joint-position (or velocity) array."""
        data = np.asarray(data, dtype=np.float64)

        if self._filter_type == "butterworth" and self._filter is not None:
            return self._filter.filter_array("joint", data)

        if self._filter_type == "ema":
            alpha = self._ema_alpha
            if self._prev_value is not None:
                smoothed = alpha * data + (1.0 - alpha) * self._prev_value
            else:
                smoothed = data.copy()
            self._prev_value = smoothed.copy()
            return smoothed

        # "none" — pass-through
        return data

    def reset(self) -> None:
        """Reset all internal state (call on discontinuities)."""
        if isinstance(self._filter, ButterworthFilter):
            self._filter.reset()
        self._prev_value = None
