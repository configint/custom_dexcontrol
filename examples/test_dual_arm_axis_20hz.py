#!/usr/bin/env python3
"""Send per-axis Cartesian commands at 20Hz to left/right RobotEnv servers.

Behavior:
- One action uses one Cartesian axis only (x, y, z, roll, pitch, yaw).
- Amplitude defaults to 0.5 for the active axis.
- For each axis, runs both + and - directions.
- In each control loop tick, commands are sent sequentially: left first, then right.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[1]
for p in (_REPO, _REPO / "src"):
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

from dexcontrol.core.robot_env_client import RobotEnvClient


LOG_PATH = "/home/dexmate/.cursor/debug-daf7f0.log"
SESSION_ID = "daf7f0"


def _json_safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


def debug_log(run_id: str, hypothesis_id: str, location: str, message: str, data: dict) -> None:
    payload = {
        "sessionId": SESSION_ID,
        "runId": run_id,
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": _json_safe(data),
        "timestamp": int(time.time() * 1000),
    }
    try:
        with open(LOG_PATH, "a", encoding="utf-8") as f:
            f.write(json.dumps(payload, ensure_ascii=True) + "\n")
    except Exception as exc:
        # #region agent log
        fallback = {
            "sessionId": SESSION_ID,
            "runId": run_id,
            "hypothesisId": "H6",
            "location": "examples/test_dual_arm_axis_20hz.py:debug_log",
            "message": "debug_log_write_failed",
            "data": {
                "original_hypothesis": hypothesis_id,
                "original_message": message,
                "error": str(exc),
            },
            "timestamp": int(time.time() * 1000),
        }
        # #endregion
        try:
            with open(LOG_PATH, "a", encoding="utf-8") as f:
                f.write(json.dumps(fallback, ensure_ascii=True, default=str) + "\n")
        except Exception:
            pass


def make_action(axis_idx: int, value: float, gripper: float) -> list[float]:
    action = [0.0] * 7
    action[axis_idx] = float(value)
    action[6] = float(gripper)
    return action


def _wrap_angle_delta(rad_delta: float) -> float:
    return float(np.arctan2(np.sin(rad_delta), np.cos(rad_delta)))


def compute_cartesian_delta(prev_pose: np.ndarray, curr_pose: np.ndarray):
    """Compute cartesian delta with wrapped angle deltas for rpy."""
    raw = np.asarray(curr_pose, dtype=np.float64) - np.asarray(prev_pose, dtype=np.float64)
    wrapped = raw.copy()
    wrap_flags = [False, False, False]
    for i in range(3, 6):
        w = _wrap_angle_delta(float(raw[i]))
        wrapped[i] = w
        wrap_flags[i - 3] = bool(abs(raw[i] - w) > 1e-6)
    return wrapped, raw, wrap_flags


def compute_axis_pollution(
    delta: list[float] | None,
    axis_idx: int,
    eps: float = 1e-9,
    min_primary_abs: float = 1e-4,
):
    """Return pollution metrics for one loop delta.

    pollution_pct = (sum(abs(non-command axes)) / abs(command axis)) * 100
    computed within xyz group (0..2) or rpy group (3..5) to avoid unit mixing.
    """
    if delta is None:
        return None
    d = np.asarray(delta, dtype=np.float64)
    if d.shape[0] < 6:
        return None
    if axis_idx < 3:
        group_start, group_end = 0, 3
    else:
        group_start, group_end = 3, 6
    group = np.abs(d[group_start:group_end])
    local_idx = axis_idx - group_start
    primary = float(group[local_idx])
    cross = float(np.sum(group) - primary)
    valid = primary >= float(min_primary_abs)
    pollution_pct = (cross / max(primary, eps)) * 100.0 if valid else None
    return {
        "primary_abs": primary,
        "cross_abs_sum": cross,
        "pollution_pct": pollution_pct,
        "valid_for_pct": valid,
        "min_primary_abs": float(min_primary_abs),
        "group": "xyz" if axis_idx < 3 else "rpy",
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Dual-arm axis test at fixed Hz")
    parser.add_argument("--left-host", default="localhost")
    parser.add_argument("--left-port", type=int, default=50061)
    parser.add_argument("--right-host", default="localhost")
    parser.add_argument("--right-port", type=int, default=50063)
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--action-space", default="cartesian_velocity", choices=["cartesian_velocity", "cartesian_delta"])
    parser.add_argument("--amplitude", type=float, default=0.25, help="Value for active axis")
    parser.add_argument("--steps-per-direction", type=int, default=20)
    parser.add_argument("--gripper", type=float, default=0.5)
    parser.add_argument(
        "--min-primary-abs",
        type=float,
        default=1e-4,
        help="Minimum commanded-axis delta magnitude required to include pollution %% in summary",
    )
    parser.add_argument("--no-reset", action="store_true")
    parser.add_argument("--run-id", default="baseline")
    args = parser.parse_args()

    dt = 1.0 / max(1.0, args.hz)
    axis_order = [("x", 0), ("y", 1), ("z", 2), ("roll", 3), ("pitch", 4), ("yaw", 5)]
    directions = [("+", 1.0), ("-", -1.0)]

    # #region agent log
    debug_log(
        run_id=args.run_id,
        hypothesis_id="H5",
        location="examples/test_dual_arm_axis_20hz.py:startup",
        message="runtime_config",
        data={
            "left": f"{args.left_host}:{args.left_port}",
            "right": f"{args.right_host}:{args.right_port}",
            "hz": args.hz,
            "dt": dt,
            "action_space": args.action_space,
            "amplitude": args.amplitude,
            "steps_per_direction": args.steps_per_direction,
        },
    )
    # #endregion

    left = RobotEnvClient(
        robot_ip=args.left_host,
        robot_port=args.left_port,
        action_space=args.action_space,
        do_reset=not args.no_reset,
    )
    right = RobotEnvClient(
        robot_ip=args.right_host,
        robot_port=args.right_port,
        action_space=args.action_space,
        do_reset=not args.no_reset,
    )

    loop_count = 0
    prev_left_cartesian = None
    prev_right_cartesian = None
    pollution_stats = {"left": {}, "right": {}}
    invalid_pollution_counts = {"left": {}, "right": {}}
    try:
        for axis_name, axis_idx in axis_order:
            for sign_name, sign in directions:
                value = sign * args.amplitude
                action = make_action(axis_idx=axis_idx, value=value, gripper=args.gripper)
                # #region agent log
                debug_log(
                    run_id=args.run_id,
                    hypothesis_id="H4",
                    location="examples/test_dual_arm_axis_20hz.py:axis_start",
                    message="axis_segment_start",
                    data={
                        "axis": axis_name,
                        "axis_idx": axis_idx,
                        "direction": sign_name,
                        "value": value,
                        "action": action,
                    },
                )
                # #endregion

                print(f"[SEGMENT] axis={axis_name:>5s} dir={sign_name} value={value:+.3f}")
                for step_i in range(args.steps_per_direction):
                    t0 = time.monotonic()
                    left_ok = True
                    right_ok = True
                    left_cartesian = None
                    right_cartesian = None
                    left_raw = None
                    right_raw = None
                    left_wrap_flags = None
                    right_wrap_flags = None

                    try:
                        left_info = left.step(action)
                        left_cartesian = left_info.get("cartesian_position")
                    except Exception as exc:
                        left_ok = False
                        # #region agent log
                        debug_log(
                            run_id=args.run_id,
                            hypothesis_id="H3",
                            location="examples/test_dual_arm_axis_20hz.py:left_step",
                            message="left_step_failed",
                            data={"error": str(exc), "axis": axis_name, "direction": sign_name, "step_i": step_i},
                        )
                        # #endregion

                    try:
                        right_info = right.step(action)
                        right_cartesian = right_info.get("cartesian_position")
                    except Exception as exc:
                        right_ok = False
                        # #region agent log
                        debug_log(
                            run_id=args.run_id,
                            hypothesis_id="H3",
                            location="examples/test_dual_arm_axis_20hz.py:right_step",
                            message="right_step_failed",
                            data={"error": str(exc), "axis": axis_name, "direction": sign_name, "step_i": step_i},
                        )
                        # #endregion

                    elapsed = time.monotonic() - t0
                    sleep_s = dt - elapsed
                    if sleep_s > 0:
                        time.sleep(sleep_s)
                    overrun_s = max(0.0, elapsed - dt)

                    left_delta = None
                    right_delta = None
                    bias_xyz = None
                    bias_rpy = None
                    if left_cartesian is not None:
                        left_arr = np.asarray(left_cartesian, dtype=np.float64)
                        if prev_left_cartesian is not None:
                            left_wrapped, left_raw, left_wrap_flags = compute_cartesian_delta(prev_left_cartesian, left_arr)
                            left_delta = left_wrapped.tolist()
                        prev_left_cartesian = left_arr
                    else:
                        left_raw = None
                        left_wrap_flags = None
                    if right_cartesian is not None:
                        right_arr = np.asarray(right_cartesian, dtype=np.float64)
                        if prev_right_cartesian is not None:
                            right_wrapped, right_raw, right_wrap_flags = compute_cartesian_delta(prev_right_cartesian, right_arr)
                            right_delta = right_wrapped.tolist()
                        prev_right_cartesian = right_arr
                    else:
                        right_raw = None
                        right_wrap_flags = None
                    if left_cartesian is not None and right_cartesian is not None:
                        l_now = np.asarray(left_cartesian, dtype=np.float64)
                        r_now = np.asarray(right_cartesian, dtype=np.float64)
                        bias_xyz = (l_now[:3] - r_now[:3]).tolist()
                        bias_rpy = (l_now[3:6] - r_now[3:6]).tolist()

                    left_pollution = compute_axis_pollution(
                        left_delta,
                        axis_idx=axis_idx,
                        min_primary_abs=args.min_primary_abs,
                    )
                    right_pollution = compute_axis_pollution(
                        right_delta,
                        axis_idx=axis_idx,
                        min_primary_abs=args.min_primary_abs,
                    )
                    stat_key = f"{axis_name}{sign_name}"
                    if left_pollution is not None:
                        if left_pollution["valid_for_pct"]:
                            pollution_stats["left"].setdefault(stat_key, []).append(left_pollution["pollution_pct"])
                        else:
                            invalid_pollution_counts["left"][stat_key] = invalid_pollution_counts["left"].get(stat_key, 0) + 1
                    if right_pollution is not None:
                        if right_pollution["valid_for_pct"]:
                            pollution_stats["right"].setdefault(stat_key, []).append(right_pollution["pollution_pct"])
                        else:
                            invalid_pollution_counts["right"][stat_key] = invalid_pollution_counts["right"].get(stat_key, 0) + 1

                    # #region agent log
                    debug_log(
                        run_id=args.run_id,
                        hypothesis_id="H2",
                        location="examples/test_dual_arm_axis_20hz.py:loop_tick",
                        message="loop_timing",
                        data={
                            "loop_count": loop_count,
                            "axis": axis_name,
                            "direction": sign_name,
                            "step_i": step_i,
                            "left_ok": left_ok,
                            "right_ok": right_ok,
                            "elapsed_s": elapsed,
                            "target_dt_s": dt,
                            "sleep_s": max(0.0, sleep_s),
                            "overrun_s": overrun_s,
                            "left_cartesian_position": left_cartesian,
                            "right_cartesian_position": right_cartesian,
                            "left_cartesian_delta": left_delta,
                            "right_cartesian_delta": right_delta,
                            "left_cartesian_delta_raw": left_raw.tolist() if left_raw is not None else None,
                            "right_cartesian_delta_raw": right_raw.tolist() if right_raw is not None else None,
                            "left_rpy_wrap_applied": left_wrap_flags,
                            "right_rpy_wrap_applied": right_wrap_flags,
                            "left_minus_right_xyz_bias": bias_xyz,
                            "left_minus_right_rpy_bias": bias_rpy,
                            "left_axis_pollution": left_pollution,
                            "right_axis_pollution": right_pollution,
                        },
                    )
                    # #endregion

                    if not (left_ok and right_ok):
                        # #region agent log
                        debug_log(
                            run_id=args.run_id,
                            hypothesis_id="H1",
                            location="examples/test_dual_arm_axis_20hz.py:loop_tick",
                            message="command_failed_in_loop",
                            data={
                                "axis": axis_name,
                                "direction": sign_name,
                                "action_len": len(action),
                                "action_space": args.action_space,
                            },
                        )
                        # #endregion
                    loop_count += 1

        for arm_name, arm_stats in pollution_stats.items():
            all_keys = sorted(set(arm_stats.keys()) | set(invalid_pollution_counts[arm_name].keys()))
            for key in all_keys:
                values = arm_stats.get(key, [])
                if not values:
                    summary = {
                        "arm": arm_name,
                        "segment": key,
                        "samples": 0,
                        "excluded_low_primary_samples": int(invalid_pollution_counts[arm_name].get(key, 0)),
                        "pollution_mean_pct": None,
                        "pollution_p95_pct": None,
                        "pollution_max_pct": None,
                    }
                    print(
                        f"[SUMMARY] {arm_name:>5s} {key:>5s} "
                        f"mean=n/a p95=n/a max=n/a (n=0, excluded={summary['excluded_low_primary_samples']})"
                    )
                    # #region agent log
                    debug_log(
                        run_id=args.run_id,
                        hypothesis_id="H7",
                        location="examples/test_dual_arm_axis_20hz.py:summary",
                        message="axis_pollution_summary",
                        data=summary,
                    )
                    # #endregion
                    continue
                arr = np.asarray(values, dtype=np.float64)
                summary = {
                    "arm": arm_name,
                    "segment": key,
                    "samples": int(arr.size),
                    "excluded_low_primary_samples": int(invalid_pollution_counts[arm_name].get(key, 0)),
                    "pollution_mean_pct": float(arr.mean()),
                    "pollution_p95_pct": float(np.percentile(arr, 95)),
                    "pollution_max_pct": float(arr.max()),
                }
                print(
                    f"[SUMMARY] {arm_name:>5s} {key:>5s} "
                    f"mean={summary['pollution_mean_pct']:.2f}% "
                    f"p95={summary['pollution_p95_pct']:.2f}% "
                    f"max={summary['pollution_max_pct']:.2f}% "
                    f"(n={summary['samples']})"
                )
                # #region agent log
                debug_log(
                    run_id=args.run_id,
                    hypothesis_id="H7",
                    location="examples/test_dual_arm_axis_20hz.py:summary",
                    message="axis_pollution_summary",
                    data=summary,
                )
                # #endregion
    finally:
        left.close()
        right.close()
        # #region agent log
        debug_log(
            run_id=args.run_id,
            hypothesis_id="H5",
            location="examples/test_dual_arm_axis_20hz.py:shutdown",
            message="clients_closed",
            data={"total_loops": loop_count},
        )
        # #endregion


if __name__ == "__main__":
    main()
