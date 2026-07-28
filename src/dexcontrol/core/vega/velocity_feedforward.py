"""Commanded-trajectory velocity feedforward estimation for Vega.

Velocity feedforward must describe the motion of the commanded joint
trajectory.  It must not be derived from the tracking error between a target
and hardware feedback, because doing so turns the feedforward term into an
additional feedback controller.
"""

from __future__ import annotations

import numpy as np


class TargetVelocityFeedforward:
    """Estimate feedforward velocity from commanded joint-position samples.

    Each call to :meth:`update` consumes one position sample from either the
    input target stream or the interpolated output trajectory.  The returned
    velocity is the finite difference between consecutive samples over the
    configured nominal period, optionally smoothed and scaled, in
    radians/second.  Arrival timestamps are used only to detect a stale command
    stream; network and scheduler jitter must not change the denominator.
    """

    def __init__(
        self,
        *,
        nominal_dt_s: float = 0.05,
        velocity_ratio: float = 1.0,
        smoothing_alpha: float = 1.0,
        stale_timeout_s: float = 0.1,
    ) -> None:
        nominal_dt = float(nominal_dt_s)
        ratio = float(velocity_ratio)
        alpha = float(smoothing_alpha)
        timeout = float(stale_timeout_s)

        if not np.isfinite(nominal_dt) or nominal_dt <= 0.0:
            raise ValueError("nominal_dt_s must be finite and positive")
        if not np.isfinite(ratio) or ratio < 0.0:
            raise ValueError("velocity_ratio must be finite and non-negative")
        if not np.isfinite(alpha) or not 0.0 <= alpha <= 1.0:
            raise ValueError("smoothing_alpha must be in [0, 1]")
        if not np.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("stale_timeout_s must be finite and positive")

        self._nominal_dt_s = nominal_dt
        self._velocity_ratio = ratio
        self._smoothing_alpha = alpha
        self._stale_timeout_s = timeout
        self._previous_target: np.ndarray | None = None
        self._previous_timestamp: float | None = None
        self._velocity: np.ndarray | None = None

    def reset(self) -> None:
        """Clear target and velocity history."""
        self._previous_target = None
        self._previous_timestamp = None
        self._velocity = None

    def update(self, target: np.ndarray, timestamp: float) -> np.ndarray:
        """Update the estimate from a commanded joint-position sample."""
        target_array = np.asarray(target, dtype=np.float64)
        current_timestamp = float(timestamp)

        if target_array.ndim != 1 or not np.all(np.isfinite(target_array)):
            raise ValueError("target must be a finite 1-D array")
        if not np.isfinite(current_timestamp):
            raise ValueError("timestamp must be finite")
        zero_velocity = np.zeros_like(target_array)
        if self._previous_target is None or self._previous_timestamp is None:
            velocity = zero_velocity
        else:
            dt = current_timestamp - self._previous_timestamp
            history_is_valid = (
                0.0 < dt <= self._stale_timeout_s
                and self._previous_target.shape == target_array.shape
            )
            if not history_is_valid:
                velocity = zero_velocity
            else:
                raw_velocity = (
                    (target_array - self._previous_target)
                    / self._nominal_dt_s
                    * self._velocity_ratio
                )
                previous_velocity = (
                    self._velocity
                    if self._velocity is not None
                    and self._velocity.shape == target_array.shape
                    else zero_velocity
                )
                alpha = self._smoothing_alpha
                velocity = (
                    alpha * raw_velocity
                    + (1.0 - alpha) * previous_velocity
                )

        self._previous_target = target_array.copy()
        self._previous_timestamp = current_timestamp
        self._velocity = velocity.copy()
        return velocity.copy()

    def hold(
        self,
        target: np.ndarray,
        timestamp: float,
        *,
        zero_velocity: bool = False,
    ) -> np.ndarray:
        """Refresh the held position sample without re-differentiating it.

        A causal trajectory can reach its endpoint just before the next
        input-rate command arrives.  Re-differentiating the repeated endpoint
        would immediately replace the last motion feedforward with zero.  This
        method advances the position/timestamp history while preserving the
        last feedforward sample for a bounded grace period.  Callers can force
        the held velocity to zero when the input command timeout expires.
        """
        target_array = np.asarray(target, dtype=np.float64)
        current_timestamp = float(timestamp)

        if target_array.ndim != 1 or not np.all(np.isfinite(target_array)):
            raise ValueError("target must be a finite 1-D array")
        if not np.isfinite(current_timestamp):
            raise ValueError("timestamp must be finite")

        if (
            zero_velocity
            or self._velocity is None
            or self._velocity.shape != target_array.shape
        ):
            velocity = np.zeros_like(target_array)
        else:
            velocity = self._velocity.copy()

        self._previous_target = target_array.copy()
        self._previous_timestamp = current_timestamp
        self._velocity = velocity.copy()
        return velocity.copy()

    def update_for_command_age(
        self,
        target: np.ndarray,
        timestamp: float,
        *,
        command_age_s: float,
        trajectory_duration_s: float,
        hold_timeout_s: float,
    ) -> np.ndarray:
        """Update, hold, or stop feedforward based on input command age.

        The trajectory derivative is used while the planned segment is
        active.  After the segment ends, the last position and feedforward
        sample are held until ``hold_timeout_s``.  Once that input-age timeout
        expires, the held position is retained with exactly zero feedforward.
        """
        command_age = float(command_age_s)
        trajectory_duration = float(trajectory_duration_s)
        hold_timeout = float(hold_timeout_s)

        if not np.isfinite(command_age) or command_age < 0.0:
            raise ValueError("command_age_s must be finite and non-negative")
        if not np.isfinite(trajectory_duration) or trajectory_duration <= 0.0:
            raise ValueError(
                "trajectory_duration_s must be finite and positive"
            )
        if (
            not np.isfinite(hold_timeout)
            or hold_timeout < trajectory_duration
        ):
            raise ValueError(
                "hold_timeout_s must be finite and no shorter than "
                "trajectory_duration_s"
            )

        if command_age <= trajectory_duration:
            return self.update(target, timestamp)

        return self.hold(
            target,
            timestamp,
            zero_velocity=command_age > hold_timeout,
        )

    def sample(self, timestamp: float) -> np.ndarray | None:
        """Return the latest estimate, or zero if the input stream is stale."""
        if (
            self._velocity is None
            or self._previous_timestamp is None
        ):
            return None

        current_timestamp = float(timestamp)
        if not np.isfinite(current_timestamp):
            raise ValueError("timestamp must be finite")

        age = current_timestamp - self._previous_timestamp
        if age < 0.0 or age > self._stale_timeout_s:
            return np.zeros_like(self._velocity)
        return self._velocity.copy()
