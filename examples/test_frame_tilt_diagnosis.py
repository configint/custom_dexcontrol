#!/usr/bin/env python3
"""Diagnose frame-tilt vs kinematic-coupling using axis segment tests.

This script is designed to separate two effects:
1) Frame tilt / frame mapping issue: dz/dx (or dz/dy) slope is direction-symmetric.
2) IK/control asymmetry: + and - direction contamination differ significantly.

It runs short Cartesian commands for +x/-x/+y/-y and reports:
- primary displacement
- vertical contamination ratio
- horizontal cross-axis ratio
- direction asymmetry scores
"""

from __future__ import annotations

import argparse
import json
import math
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

# Match RobotEnv Vega server presets so users don't need raw joint vectors.
PRESET_HOME_JOINTS = {
    "left": np.array([-1.4234, 1.3524, 2.8707, -1.9810, 0.6751, -0.1662, 0.0680], dtype=np.float64),
    "right": np.array([1.4234, -1.3524, -2.8707, -1.9810, -0.1515, 0.1662, -0.0680], dtype=np.float64),
}
PRESET_MIDDLE_JOINTS = {
    "left": np.array([-2.2180, 0.7430, 2.8684, -1.3442, -1.2865, -0.6128, -1.1779], dtype=np.float64),
    "right": np.array([2.2180, -0.7430, -2.8684, -1.3442, 1.8101, 0.6128, 1.1779], dtype=np.float64),
}


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
    except Exception:
        pass


def get_cartesian(client: RobotEnvClient) -> np.ndarray:
    obs, _ = client.get_observation()
    cart = obs["robot_state"]["cartesian_position"]
    return np.asarray(cart, dtype=np.float64)


def get_joints(client: RobotEnvClient) -> np.ndarray:
    obs, _ = client.get_observation()
    joints = obs["robot_state"]["joint_positions"]
    return np.asarray(joints, dtype=np.float64)


def run_segment(
    client: RobotEnvClient,
    action: list[float],
    steps: int,
    dt: float,
    axis_name: str,
    direction_name: str,
    run_id: str,
):
    cart_before = get_cartesian(client)
    joints_before = get_joints(client)
    failures = 0
    elapsed_samples = []

    # #region agent log
    debug_log(
        run_id=run_id,
        hypothesis_id="H2",
        location="examples/test_frame_tilt_diagnosis.py:segment_start",
        message="segment_start",
        data={"axis": axis_name, "direction": direction_name, "steps": steps, "action": action},
    )
    # #endregion

    for step_i in range(steps):
        t0 = time.monotonic()
        try:
            client.step(action)
        except Exception as exc:
            failures += 1
            # #region agent log
            debug_log(
                run_id=run_id,
                hypothesis_id="H4",
                location="examples/test_frame_tilt_diagnosis.py:segment_loop",
                message="step_failed",
                data={
                    "axis": axis_name,
                    "direction": direction_name,
                    "step_i": step_i,
                    "error": str(exc),
                },
            )
            # #endregion
        elapsed = time.monotonic() - t0
        elapsed_samples.append(elapsed)
        rem = dt - elapsed
        if rem > 0:
            time.sleep(rem)

    cart_after = get_cartesian(client)
    joints_after = get_joints(client)
    return {
        "cart_before": cart_before,
        "cart_after": cart_after,
        "cart_delta": cart_after - cart_before,
        "joint_before": joints_before,
        "joint_after": joints_after,
        "joint_delta": joints_after - joints_before,
        "step_failures": failures,
        "mean_elapsed_s": float(np.mean(elapsed_samples)) if elapsed_samples else 0.0,
        "max_elapsed_s": float(np.max(elapsed_samples)) if elapsed_samples else 0.0,
    }


def contamination_metrics(cart_delta: np.ndarray, primary_axis: int):
    primary = float(cart_delta[primary_axis])
    primary_abs = abs(primary)

    if primary_axis == 0:
        horiz_cross = float(cart_delta[1])  # y
    else:
        horiz_cross = float(cart_delta[0])  # x
    vert_cross = float(cart_delta[2])  # z

    eps = 1e-9
    slope = None
    if primary_abs > eps:
        slope = vert_cross / primary
    return {
        "primary": primary,
        "primary_abs": primary_abs,
        "horiz_cross": horiz_cross,
        "vert_cross": vert_cross,
        "vert_ratio_abs": abs(vert_cross) / max(primary_abs, eps),
        "horiz_ratio_abs": abs(horiz_cross) / max(primary_abs, eps),
        "slope_vert_over_primary": slope,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Frame tilt diagnosis with +x/-x/+y/-y segments")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=50061)
    parser.add_argument("--arm-side", default="left", choices=["left", "right"])
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--action-space", default="cartesian_velocity", choices=["cartesian_velocity", "cartesian_delta"])
    parser.add_argument("--amplitude", type=float, default=0.2)
    parser.add_argument("--steps", type=int, default=20)
    parser.add_argument("--trials", type=int, default=2)
    parser.add_argument("--gripper", type=float, default=0.5)
    parser.add_argument(
        "--pre-reset-joints",
        type=float,
        nargs=7,
        default=None,
        help="Optional 7 joint values (rad) to move before diagnosis",
    )
    parser.add_argument(
        "--pre-pose-name",
        default="none",
        choices=["none", "home", "middle"],
        help="Optional named preset before diagnosis (from server defaults)",
    )
    parser.add_argument(
        "--pre-action",
        type=float,
        nargs=7,
        default=None,
        help="Optional 7D action to run before diagnosis",
    )
    parser.add_argument(
        "--pre-action-steps",
        type=int,
        default=0,
        help="How many steps to run for --pre-action before diagnosis",
    )
    parser.add_argument(
        "--pre-settle-s",
        type=float,
        default=1.0,
        help="Settle time in seconds after pre-positioning",
    )
    parser.add_argument("--no-reset", action="store_true")
    parser.add_argument("--run-id", default="frame-diagnosis")
    args = parser.parse_args()

    dt = 1.0 / max(1.0, args.hz)

    # H1: Frame tilt causes direction-symmetric vertical slope (dz/dx, dz/dy).
    # H2: Kinematic coupling causes direction-asymmetric contamination (+ vs - differs).
    # H3: Frequency/scaling effects appear as timing pressure or inconsistent per-step response.
    # H4: Hard failures (IK/joint errors) dominate contamination spikes.
    # H5: Return-to-origin drift indicates non-canceling path behavior.

    # #region agent log
    debug_log(
        run_id=args.run_id,
        hypothesis_id="H3",
        location="examples/test_frame_tilt_diagnosis.py:startup",
        message="runtime_config",
        data={
            "host": args.host,
            "port": args.port,
            "hz": args.hz,
            "dt": dt,
            "action_space": args.action_space,
            "amplitude": args.amplitude,
            "steps": args.steps,
            "trials": args.trials,
            "arm_side": args.arm_side,
            "pre_pose_name": args.pre_pose_name,
            "pre_reset_joints": args.pre_reset_joints,
            "pre_action": args.pre_action,
            "pre_action_steps": args.pre_action_steps,
            "pre_settle_s": args.pre_settle_s,
        },
    )
    # #endregion

    client = RobotEnvClient(
        robot_ip=args.host,
        robot_port=args.port,
        action_space=args.action_space,
        do_reset=not args.no_reset,
    )

    # Optional deterministic pre-positioning phase (excluded from diagnosis segments).
    selected_preset = None
    if args.pre_reset_joints is not None:
        selected_preset = np.asarray(args.pre_reset_joints, dtype=np.float64)
    elif args.pre_pose_name == "home":
        selected_preset = PRESET_HOME_JOINTS[args.arm_side].copy()
    elif args.pre_pose_name == "middle":
        selected_preset = PRESET_MIDDLE_JOINTS[args.arm_side].copy()

    pre_action = np.asarray(args.pre_action, dtype=np.float64).tolist() \
        if args.pre_action is not None and args.pre_action_steps > 0 else None

    def apply_pre_phase(trial_idx: int) -> None:
        if selected_preset is not None:
            # #region agent log
            debug_log(
                run_id=args.run_id,
                hypothesis_id="H11",
                location="examples/test_frame_tilt_diagnosis.py:pre_position",
                message="pre_reset_start",
                data={
                    "trial": trial_idx,
                    "pre_reset_joints": selected_preset.tolist(),
                    "pre_pose_name": args.pre_pose_name,
                },
            )
            # #endregion
            client.reset(reset_pose=selected_preset)
            if args.pre_settle_s > 0:
                time.sleep(args.pre_settle_s)
            # #region agent log
            debug_log(
                run_id=args.run_id,
                hypothesis_id="H11",
                location="examples/test_frame_tilt_diagnosis.py:pre_position",
                message="pre_reset_done",
                data={"trial": trial_idx, "pose_after_pre_reset": get_cartesian(client).tolist()},
            )
            # #endregion

        if pre_action is not None:
            # #region agent log
            debug_log(
                run_id=args.run_id,
                hypothesis_id="H11",
                location="examples/test_frame_tilt_diagnosis.py:pre_position",
                message="pre_action_start",
                data={"trial": trial_idx, "pre_action": pre_action, "steps": args.pre_action_steps},
            )
            # #endregion
            for _ in range(args.pre_action_steps):
                client.step(pre_action)
                time.sleep(dt)
            if args.pre_settle_s > 0:
                time.sleep(args.pre_settle_s)
            # #region agent log
            debug_log(
                run_id=args.run_id,
                hypothesis_id="H11",
                location="examples/test_frame_tilt_diagnosis.py:pre_position",
                message="pre_action_done",
                data={"trial": trial_idx, "pose_after_pre_action": get_cartesian(client).tolist()},
            )
            # #endregion

    segments = [
        ("x", "+", [args.amplitude, 0.0, 0.0, 0.0, 0.0, 0.0, args.gripper], 0),
        ("x", "-", [-args.amplitude, 0.0, 0.0, 0.0, 0.0, 0.0, args.gripper], 0),
        ("y", "+", [0.0, args.amplitude, 0.0, 0.0, 0.0, 0.0, args.gripper], 1),
        ("y", "-", [0.0, -args.amplitude, 0.0, 0.0, 0.0, 0.0, args.gripper], 1),
    ]

    all_metrics = {"x+": [], "x-": [], "y+": [], "y-": []}
    origin = get_cartesian(client)
    try:
        for trial in range(args.trials):
            if trial > 0 and not args.no_reset:
                client.reset()
                time.sleep(0.5)
            apply_pre_phase(trial)
            if trial == 0:
                # define origin after trial-0 pre-phase to avoid mixing setup motion
                origin = get_cartesian(client)

            for axis_name, direction_name, action, primary_axis in segments:
                result = run_segment(
                    client=client,
                    action=action,
                    steps=args.steps,
                    dt=dt,
                    axis_name=axis_name,
                    direction_name=direction_name,
                    run_id=args.run_id,
                )
                cm = contamination_metrics(result["cart_delta"], primary_axis=primary_axis)
                key = f"{axis_name}{direction_name}"
                rec = {
                    "trial": trial,
                    "key": key,
                    "cart_delta": result["cart_delta"].tolist(),
                    "joint_delta_norm": float(np.linalg.norm(result["joint_delta"])),
                    "step_failures": result["step_failures"],
                    "mean_elapsed_s": result["mean_elapsed_s"],
                    "max_elapsed_s": result["max_elapsed_s"],
                    **cm,
                }
                all_metrics[key].append(rec)
                print(
                    f"[{key}] trial={trial} dxyz={np.round(result['cart_delta'][:3], 5).tolist()} "
                    f"vert_ratio={cm['vert_ratio_abs']:.3f} horiz_ratio={cm['horiz_ratio_abs']:.3f} "
                    f"slope={cm['slope_vert_over_primary']:.3f} failures={result['step_failures']}"
                )
                # #region agent log
                debug_log(
                    run_id=args.run_id,
                    hypothesis_id="H1",
                    location="examples/test_frame_tilt_diagnosis.py:segment_end",
                    message="segment_result",
                    data=rec,
                )
                # #endregion

        summary = {}
        for key, rows in all_metrics.items():
            if not rows:
                continue
            vert = np.asarray([r["vert_ratio_abs"] for r in rows], dtype=np.float64)
            horiz = np.asarray([r["horiz_ratio_abs"] for r in rows], dtype=np.float64)
            slope_values = [r["slope_vert_over_primary"] for r in rows if r["slope_vert_over_primary"] is not None]
            slope = np.asarray(slope_values, dtype=np.float64) if slope_values else np.asarray([], dtype=np.float64)
            summary[key] = {
                "n": len(rows),
                "vert_ratio_mean": float(vert.mean()),
                "vert_ratio_p95": float(np.percentile(vert, 95)),
                "horiz_ratio_mean": float(horiz.mean()),
                "slope_mean": float(slope.mean()) if slope.size > 0 else None,
                "failures_total": int(sum(r["step_failures"] for r in rows)),
            }

        x_asym = None
        y_asym = None
        if "x+" in summary and "x-" in summary:
            x_slope_diff = None
            if summary["x+"]["slope_mean"] is not None and summary["x-"]["slope_mean"] is not None:
                x_slope_diff = summary["x-"]["slope_mean"] - summary["x+"]["slope_mean"]
            x_asym = {
                "vert_ratio_mean_diff": summary["x-"]["vert_ratio_mean"] - summary["x+"]["vert_ratio_mean"],
                "slope_mean_diff": x_slope_diff,
            }
        if "y+" in summary and "y-" in summary:
            y_slope_diff = None
            if summary["y+"]["slope_mean"] is not None and summary["y-"]["slope_mean"] is not None:
                y_slope_diff = summary["y-"]["slope_mean"] - summary["y+"]["slope_mean"]
            y_asym = {
                "vert_ratio_mean_diff": summary["y-"]["vert_ratio_mean"] - summary["y+"]["vert_ratio_mean"],
                "slope_mean_diff": y_slope_diff,
            }

        end_pose = get_cartesian(client)
        drift = end_pose - origin
        global_result = {
            "summary": summary,
            "x_asymmetry": x_asym,
            "y_asymmetry": y_asym,
            "origin_pose": origin.tolist(),
            "end_pose": end_pose.tolist(),
            "origin_drift_xyz": drift[:3].tolist(),
            "origin_drift_norm": float(np.linalg.norm(drift[:3])),
            "interpretation_hint": {
                "H1_frame_tilt_likely_if": "slope_mean for + and - similar sign/magnitude with low asymmetry",
                "H2_kinematic_asym_likely_if": "vert_ratio_mean_diff large between + and -",
                "H4_failure_likely_if": "failures_total > 0",
                "H5_drift_likely_if": "origin_drift_norm is non-trivial after full cycle",
            },
        }

        print("\n=== Summary ===")
        for key in ("x+", "x-", "y+", "y-"):
            if key in summary:
                s = summary[key]
                print(
                    f"{key}: vert_mean={s['vert_ratio_mean']:.3f} vert_p95={s['vert_ratio_p95']:.3f} "
                    f"horiz_mean={s['horiz_ratio_mean']:.3f} slope_mean={s['slope_mean']:.3f} "
                    f"fails={s['failures_total']}"
                )
        print(f"x_asym={x_asym}")
        print(f"y_asym={y_asym}")
        print(f"origin_drift_xyz={np.round(drift[:3], 6).tolist()} norm={global_result['origin_drift_norm']:.6f}")

        # #region agent log
        debug_log(
            run_id=args.run_id,
            hypothesis_id="H5",
            location="examples/test_frame_tilt_diagnosis.py:final",
            message="diagnosis_summary",
            data=global_result,
        )
        # #endregion
    finally:
        client.close()
        # #region agent log
        debug_log(
            run_id=args.run_id,
            hypothesis_id="H5",
            location="examples/test_frame_tilt_diagnosis.py:shutdown",
            message="client_closed",
            data={},
        )
        # #endregion


if __name__ == "__main__":
    main()
