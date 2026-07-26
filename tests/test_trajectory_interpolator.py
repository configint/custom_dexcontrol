"""Unit tests for causal interpolation and output-rate velocity sampling."""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

import numpy as np


_ROOT = Path(__file__).resolve().parents[1]


def _load_module(name: str, relative_path: str):
    module_path = _ROOT / relative_path
    spec = importlib.util.spec_from_file_location(name, module_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_INTERPOLATOR_MODULE = _load_module(
    "trajectory_interpolator",
    "src/dexcontrol/utils/trajectory_interpolator.py",
)
_VELOCITY_MODULE = _load_module(
    "interpolated_velocity_feedforward",
    "src/dexcontrol/core/vega/velocity_feedforward.py",
)

TrajectoryInterpolator = _INTERPOLATOR_MODULE.TrajectoryInterpolator
TargetVelocityFeedforward = _VELOCITY_MODULE.TargetVelocityFeedforward


class OnlineLinearInterpolationTests(unittest.TestCase):
    def test_target_is_spread_over_one_input_period(self) -> None:
        interpolator = TrajectoryInterpolator(method="linear", history_size=3)
        interpolator.set_linear_target(
            10.0,
            np.array([1.0, -2.0]),
            initial_positions=np.array([0.0, 0.0]),
            duration_s=0.05,
        )

        start, _ = interpolator.interpolate(10.0, compute_velocity=False)
        first_tick, _ = interpolator.interpolate(10.005, compute_velocity=False)
        halfway, _ = interpolator.interpolate(10.025, compute_velocity=False)
        end, _ = interpolator.interpolate(10.05, compute_velocity=False)

        assert start is not None
        assert first_tick is not None
        assert halfway is not None
        assert end is not None
        np.testing.assert_allclose(start, [0.0, 0.0])
        np.testing.assert_allclose(first_tick, [0.1, -0.2])
        np.testing.assert_allclose(halfway, [0.5, -1.0])
        np.testing.assert_allclose(end, [1.0, -2.0])

    def test_replanning_starts_from_current_interpolated_command(self) -> None:
        interpolator = TrajectoryInterpolator(method="linear")
        interpolator.set_linear_target(
            1.0,
            np.array([1.0]),
            initial_positions=np.array([0.0]),
            duration_s=0.05,
        )

        before_replan, _ = interpolator.interpolate(1.02, compute_velocity=False)
        interpolator.set_linear_target(
            1.02,
            np.array([2.0]),
            initial_positions=np.array([-10.0]),
            duration_s=0.05,
        )
        after_replan, _ = interpolator.interpolate(1.02, compute_velocity=False)
        new_end, _ = interpolator.interpolate(1.07, compute_velocity=False)

        assert before_replan is not None
        assert after_replan is not None
        assert new_end is not None
        np.testing.assert_allclose(before_replan, [0.4])
        np.testing.assert_allclose(after_replan, before_replan)
        np.testing.assert_allclose(new_end, [2.0])

    def test_velocity_is_derived_from_each_200hz_position_sample(self) -> None:
        interpolator = TrajectoryInterpolator(method="linear")
        interpolator.set_linear_target(
            2.0,
            np.array([1.0]),
            initial_positions=np.array([0.0]),
            duration_s=0.05,
        )
        velocity = TargetVelocityFeedforward(
            nominal_dt_s=0.005,
            smoothing_alpha=1.0,
            stale_timeout_s=0.1,
        )

        sampled_velocities = []
        for tick in range(11):
            timestamp = 2.0 + tick * 0.005
            position, _ = interpolator.interpolate(
                timestamp,
                compute_velocity=False,
            )
            assert position is not None
            sampled_velocities.append(velocity.update(position, timestamp)[0])

        held_position, _ = interpolator.interpolate(
            2.055,
            compute_velocity=False,
        )
        assert held_position is not None
        held_velocity = velocity.update(held_position, 2.055)

        self.assertEqual(sampled_velocities[0], 0.0)
        np.testing.assert_allclose(sampled_velocities[1:], np.full(10, 20.0))
        np.testing.assert_allclose(held_velocity, [0.0])

    def test_linear_target_api_rejects_non_linear_method(self) -> None:
        interpolator = TrajectoryInterpolator(method="cubic")

        with self.assertRaises(ValueError):
            interpolator.set_linear_target(
                0.0,
                np.ones(1),
                initial_positions=np.zeros(1),
                duration_s=0.05,
            )


if __name__ == "__main__":
    unittest.main()
