"""Cartesian command conversion helpers for the Vega controller.

``target_cartesian_delta`` is a physical pose error expressed in metres and
radians.  It must pass through unchanged while it is inside the configured
per-step safety limits, and be norm-clipped only when it exceeds them.

``cartesian_velocity`` keeps its legacy normalized ``[-1, 1]`` semantics and
is therefore scaled to the same per-step limits on every command.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R


def _validate_limit(name: str, value: float) -> float:
    limit = float(value)
    if not np.isfinite(limit) or limit < 0.0:
        raise ValueError(f"{name} must be a finite non-negative value, got {value}")
    return limit


def _clip_vector_norm(vector: np.ndarray, max_norm: float) -> np.ndarray:
    """Return ``vector`` unchanged below ``max_norm``, otherwise norm-clip it."""
    norm = float(np.linalg.norm(vector))
    if norm == 0.0 or norm <= max_norm:
        return vector
    return vector * (max_norm / norm)


def clip_physical_cartesian_delta(
    command: np.ndarray,
    max_linear_delta: float,
    max_rotation_delta: float,
) -> np.ndarray:
    """Clip a physical Cartesian pose delta in metres/radians.

    Linear and rotational 3-vectors are clipped independently so their
    directions are preserved.  Any trailing values, such as a gripper command,
    are copied without modification.
    """
    linear_limit = _validate_limit("max_linear_delta", max_linear_delta)
    rotation_limit = _validate_limit("max_rotation_delta", max_rotation_delta)

    converted = np.asarray(command, dtype=np.float64).copy()
    if converted.ndim != 1 or converted.shape[0] < 6:
        raise ValueError(
            "Cartesian command must be a one-dimensional array with at least 6 values"
        )

    converted[:3] = _clip_vector_norm(converted[:3], linear_limit)
    converted[3:6] = _clip_vector_norm(converted[3:6], rotation_limit)
    return converted


def normalized_cartesian_velocity_to_delta(
    command: np.ndarray,
    max_linear_delta: float,
    max_rotation_delta: float,
) -> np.ndarray:
    """Convert a legacy normalized Cartesian velocity to a per-step delta."""
    linear_limit = _validate_limit("max_linear_delta", max_linear_delta)
    rotation_limit = _validate_limit("max_rotation_delta", max_rotation_delta)

    converted = np.asarray(command, dtype=np.float64).copy()
    if converted.ndim != 1 or converted.shape[0] < 6:
        raise ValueError(
            "Cartesian command must be a one-dimensional array with at least 6 values"
        )

    linear = _clip_vector_norm(converted[:3], 1.0)
    rotation = _clip_vector_norm(converted[3:6], 1.0)
    converted[:3] = linear * linear_limit
    converted[3:6] = rotation * rotation_limit
    return converted


def rebase_physical_cartesian_delta(
    command: np.ndarray,
    reference_pose: np.ndarray,
    current_pose: np.ndarray,
    max_linear_delta: float,
    max_rotation_delta: float,
) -> np.ndarray:
    """Re-express a target pose error from an observed pose at a newer pose.

    ``command`` is the physical Cartesian delta calculated against
    ``reference_pose``.  The returned delta reaches the same Cartesian goal
    from ``current_pose`` and is clipped to the configured per-step limits.
    Poses and deltas use ``[x, y, z, roll, pitch, yaw]`` in metres/radians.
    Any trailing command values, such as the gripper target, are preserved.
    """
    linear_limit = _validate_limit("max_linear_delta", max_linear_delta)
    rotation_limit = _validate_limit("max_rotation_delta", max_rotation_delta)

    rebased = np.asarray(command, dtype=np.float64).copy()
    reference = np.asarray(reference_pose, dtype=np.float64)
    current = np.asarray(current_pose, dtype=np.float64)
    if rebased.ndim != 1 or rebased.shape[0] < 6:
        raise ValueError(
            "Cartesian command must be a one-dimensional array with at least 6 values"
        )
    if reference.shape != (6,) or current.shape != (6,):
        raise ValueError("reference_pose and current_pose must each have shape (6,)")
    if not (
        np.all(np.isfinite(rebased[:6]))
        and np.all(np.isfinite(reference))
        and np.all(np.isfinite(current))
    ):
        raise ValueError("Cartesian command and poses must contain only finite values")

    target_xyz = reference[:3] + rebased[:3]
    rebased[:3] = _clip_vector_norm(target_xyz - current[:3], linear_limit)

    delta_rotation = R.from_euler("xyz", rebased[3:6])
    reference_rotation = R.from_euler("xyz", reference[3:6])
    current_rotation = R.from_euler("xyz", current[3:6])
    target_rotation = delta_rotation * reference_rotation
    current_to_target = target_rotation * current_rotation.inv()
    rotation_vector = _clip_vector_norm(
        current_to_target.as_rotvec(),
        rotation_limit,
    )
    rebased[3:6] = R.from_rotvec(rotation_vector).as_euler("xyz")
    return rebased
