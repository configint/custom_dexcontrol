"""Unit tests for Vega Cartesian command semantics.

The production package imports robot-only dependencies from ``dexcontrol``'s
top-level module, so load this pure numerical helper directly for host-side
tests.
"""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

import numpy as np


_MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "dexcontrol"
    / "core"
    / "vega"
    / "cartesian_commands.py"
)
_SPEC = importlib.util.spec_from_file_location("vega_cartesian_commands", _MODULE_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)

clip_physical_cartesian_delta = _MODULE.clip_physical_cartesian_delta
absolute_cartesian_target_to_delta = (
    _MODULE.absolute_cartesian_target_to_delta
)
normalized_cartesian_velocity_to_delta = (
    _MODULE.normalized_cartesian_velocity_to_delta
)


class PhysicalCartesianDeltaTests(unittest.TestCase):
    def test_small_physical_error_passes_through_unchanged(self) -> None:
        command = np.array([0.01, -0.02, 0.03, 0.10, -0.05, 0.02, 0.7])

        result = clip_physical_cartesian_delta(
            command,
            max_linear_delta=0.065625,
            max_rotation_delta=0.2625,
        )

        np.testing.assert_allclose(result, command)
        self.assertIsNot(result, command)

    def test_linear_and_rotation_norms_are_clipped_independently(self) -> None:
        command = np.array([3.0, 4.0, 0.0, 0.0, 0.0, -0.5, 0.25])

        result = clip_physical_cartesian_delta(
            command,
            max_linear_delta=0.1,
            max_rotation_delta=0.2,
        )

        np.testing.assert_allclose(result[:3], [0.06, 0.08, 0.0])
        np.testing.assert_allclose(result[3:6], [0.0, 0.0, -0.2])
        self.assertEqual(result[6], command[6])

    def test_zero_limit_zeroes_only_nonzero_cartesian_vectors(self) -> None:
        command = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4])

        result = clip_physical_cartesian_delta(
            command,
            max_linear_delta=0.0,
            max_rotation_delta=0.0,
        )

        np.testing.assert_allclose(result[:6], np.zeros(6))
        self.assertEqual(result[6], command[6])

    def test_invalid_limit_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            clip_physical_cartesian_delta(
                np.zeros(7),
                max_linear_delta=-0.1,
                max_rotation_delta=0.2,
            )


class LegacyCartesianVelocityTests(unittest.TestCase):
    def test_small_normalized_velocity_is_still_scaled(self) -> None:
        command = np.array([0.1, 0.0, 0.0, 0.0, -0.2, 0.0, 0.7])

        result = normalized_cartesian_velocity_to_delta(
            command,
            max_linear_delta=0.065625,
            max_rotation_delta=0.2625,
        )

        np.testing.assert_allclose(result[:3], [0.0065625, 0.0, 0.0])
        np.testing.assert_allclose(result[3:6], [0.0, -0.0525, 0.0])
        self.assertEqual(result[6], command[6])

    def test_normalized_velocity_is_norm_limited_before_scaling(self) -> None:
        command = np.array([3.0, 4.0, 0.0, 0.0, 0.0, 2.0])

        result = normalized_cartesian_velocity_to_delta(
            command,
            max_linear_delta=0.1,
            max_rotation_delta=0.2,
        )

        np.testing.assert_allclose(result[:3], [0.06, 0.08, 0.0])
        np.testing.assert_allclose(result[3:6], [0.0, 0.0, 0.2])


class AbsoluteCartesianTargetTests(unittest.TestCase):
    def test_target_is_recomputed_from_current_pose(self) -> None:
        target = np.array([0.50, 0.10, 0.30, 0.0, 0.0, 0.0])
        first_current = np.array([0.45, 0.10, 0.30, 0.0, 0.0, 0.0])
        later_current = np.array([0.48, 0.10, 0.30, 0.0, 0.0, 0.0])

        first_delta = absolute_cartesian_target_to_delta(
            target, first_current, 0.1, 0.3
        )
        later_delta = absolute_cartesian_target_to_delta(
            target, later_current, 0.1, 0.3
        )

        np.testing.assert_allclose(first_delta[:3], [0.05, 0.0, 0.0])
        np.testing.assert_allclose(later_delta[:3], [0.02, 0.0, 0.0])

    def test_unclipped_rotation_delta_reconstructs_target(self) -> None:
        current = np.array([0.0, 0.0, 0.0, 0.2, -0.1, 0.3])
        target = np.array([0.0, 0.0, 0.0, -0.4, 0.25, -0.2])

        delta = absolute_cartesian_target_to_delta(
            target, current, 1.0, np.pi
        )

        reconstructed = (
            _MODULE.R.from_euler("xyz", delta[3:6])
            * _MODULE.R.from_euler("xyz", current[3:6])
        )
        expected = _MODULE.R.from_euler("xyz", target[3:6])
        np.testing.assert_allclose(
            (reconstructed * expected.inv()).as_rotvec(),
            np.zeros(3),
            atol=1e-12,
        )

    def test_translation_and_physical_rotation_are_bounded(self) -> None:
        current = np.zeros(6)
        target = np.array([3.0, 4.0, 0.0, 0.0, 0.0, 1.0])

        delta = absolute_cartesian_target_to_delta(
            target, current, 0.1, 0.2
        )

        np.testing.assert_allclose(delta[:3], [0.06, 0.08, 0.0])
        self.assertAlmostEqual(
            np.linalg.norm(
                _MODULE.R.from_euler("xyz", delta[3:6]).as_rotvec()
            ),
            0.2,
        )


if __name__ == "__main__":
    unittest.main()
