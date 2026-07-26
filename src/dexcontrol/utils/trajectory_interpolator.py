"""Trajectory interpolation for smooth motion control.

Adapted from omniteleop's TrajectoryInterpolator for use in the gRPC
robot-env pipeline.  Maintains a rolling history of incoming command
points and produces smoothly interpolated positions (and optionally
velocities) at arbitrary query times — enabling input-rate to
control-rate upsampling (e.g. 20 Hz input → 100 Hz output).
"""

from __future__ import annotations

from collections import deque
from typing import Literal, Optional, Tuple

import numpy as np
from scipy.interpolate import PchipInterpolator, interp1d


class TrajectoryInterpolator:
    """Rolling-window trajectory interpolator with velocity computation.

    Keeps the most recent *history_size* command points and lazily
    rebuilds scipy interpolators when the history changes.  Supports
    both linear and cubic (PCHIP — monotone, C1-continuous, no
    overshoot) interpolation.

    Typical usage inside a control loop::

        interp = TrajectoryInterpolator(method="cubic", history_size=4)

        # Called at input rate (e.g. 20 Hz)
        interp.add_point(timestamp, joint_positions)

        # Called at control rate (e.g. 100 Hz)
        pos, vel = interp.interpolate(now, compute_velocity=True)
    """

    __slots__ = (
        "method",
        "history_size",
        "_times",
        "_positions",
        "_interpolators",
        "_interpolators_dirty",
        "_prev_positions",
        "_prev_time",
    )

    def __init__(
        self,
        method: Literal["linear", "cubic"] = "cubic",
        history_size: int = 4,
    ) -> None:
        self.method = method
        self.history_size = max(2, history_size)

        self._times: deque[float] = deque(maxlen=self.history_size)
        self._positions: deque[np.ndarray] = deque(maxlen=self.history_size)

        self._interpolators: Optional[object] = None
        self._interpolators_dirty = True

        self._prev_positions: Optional[np.ndarray] = None
        self._prev_time: float = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def add_point(self, timestamp: float, positions: np.ndarray) -> None:
        """Append a new (timestamp, positions) sample.

        Args:
            timestamp: ``time.perf_counter()`` value.
            positions: 1-D array of joint positions.
        """
        self._times.append(timestamp)
        self._positions.append(np.asarray(positions, dtype=np.float64))
        self._interpolators_dirty = True

    def set_linear_target(
        self,
        timestamp: float,
        target_positions: np.ndarray,
        *,
        initial_positions: np.ndarray,
        duration_s: float,
    ) -> None:
        """Plan a causal linear segment from the current command to a target.

        A target received at ``timestamp`` cannot be interpolated from a future
        sample that has not arrived yet.  Instead, sample the currently active
        command at that instant and schedule the new target one nominal input
        period into the future.  Replanning from the sampled command keeps
        position continuous even when input packets arrive early or late.

        This method intentionally supports only ``method="linear"``.  Future
        interpolation methods can provide their own segment planner while the
        control loop continues deriving velocity from the sampled positions.
        """
        if self.method != "linear":
            raise ValueError("set_linear_target requires method='linear'")

        start_time = float(timestamp)
        duration = float(duration_s)
        target = np.asarray(target_positions, dtype=np.float64)
        initial = np.asarray(initial_positions, dtype=np.float64)

        if not np.isfinite(start_time):
            raise ValueError("timestamp must be finite")
        if not np.isfinite(duration) or duration <= 0.0:
            raise ValueError("duration_s must be finite and positive")
        if (
            target.ndim != 1
            or initial.ndim != 1
            or target.shape != initial.shape
            or not np.all(np.isfinite(target))
            or not np.all(np.isfinite(initial))
        ):
            raise ValueError(
                "target_positions and initial_positions must be finite, "
                "same-shaped 1-D arrays"
            )

        current, _ = self.interpolate(start_time, compute_velocity=False)
        start = initial.copy() if current is None else current

        self.clear()
        self.add_point(start_time, start)
        self.add_point(start_time + duration, target)

    def interpolate(
        self,
        query_time: float,
        compute_velocity: bool = True,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Evaluate interpolated position (and velocity) at *query_time*.

        Returns ``(None, None)`` when there are fewer than 2 data points.
        """
        if len(self._times) < 2:
            # Not enough data — fall back to latest if available.
            if self._positions:
                return self._positions[-1].copy(), None
            return None, None

        if self._interpolators_dirty or self._interpolators is None:
            self._interpolators = self._build_interpolators()
            self._interpolators_dirty = False

        if self._interpolators is None:
            return None, None

        interp = self._interpolators["interp"]
        t0 = self._interpolators["t0"]
        t_span = self._interpolators["t_span"]

        rel_t = query_time - t0

        # If query_time is past the last data point, return the latest
        # position with zero velocity.  This prevents the "stutter"
        # artefact where the robot sits at a clamped position then
        # suddenly jumps when a new input arrives.
        if rel_t >= t_span:
            pos = self._positions[-1].copy()
            vel = np.zeros_like(pos) if compute_velocity else None
            self._prev_positions = pos.copy()
            self._prev_time = query_time
            return pos, vel

        # Clamp to the planned interval.  Evaluating the exact start point is
        # important when a segment is replanned from the current command.
        clamped = float(np.clip(rel_t, 0.0, t_span))

        pos = np.asarray(interp(clamped), dtype=np.float64)

        vel: Optional[np.ndarray] = None
        if compute_velocity:
            if isinstance(interp, PchipInterpolator):
                vel = np.asarray(interp(clamped, 1), dtype=np.float64)
            elif self._prev_positions is not None and self._prev_time > 0:
                dt = query_time - self._prev_time
                if dt > 0:
                    vel = (pos - self._prev_positions) / dt
                else:
                    vel = np.zeros_like(pos)
            else:
                dt_back = 0.01
                t_prev = max(clamped - dt_back, 0.0)
                actual_dt = clamped - t_prev
                if actual_dt > 0:
                    vel = (pos - np.asarray(interp(t_prev), dtype=np.float64)) / actual_dt
                else:
                    vel = np.zeros_like(pos)

            if not isinstance(interp, PchipInterpolator):
                self._prev_positions = pos.copy()
                self._prev_time = query_time

        return pos, vel

    def clear(self) -> None:
        """Drop all history."""
        self._times.clear()
        self._positions.clear()
        self._interpolators = None
        self._interpolators_dirty = True
        self._prev_positions = None
        self._prev_time = 0.0

    def has_sufficient_data(self) -> bool:
        return len(self._times) >= 2

    def get_latest(self) -> Optional[np.ndarray]:
        if self._positions:
            return self._positions[-1].copy()
        return None

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _build_interpolators(self) -> Optional[dict]:
        if len(self._times) < 2:
            return None

        times = np.array(self._times, dtype=np.float64)
        t0 = times[0]
        norm_t = times - t0
        positions = np.stack(list(self._positions), axis=0)  # (N, n_joints)

        use_cubic = self.method == "cubic" and len(norm_t) >= 4

        try:
            if use_cubic:
                interp = PchipInterpolator(norm_t, positions, axis=0, extrapolate=False)
            else:
                interp = interp1d(
                    norm_t,
                    positions,
                    kind="linear",
                    axis=0,
                    bounds_error=False,
                    fill_value=(positions[0], positions[-1]),
                )
        except Exception:
            return None

        return {"interp": interp, "t0": t0, "t_span": float(norm_t[-1])}
