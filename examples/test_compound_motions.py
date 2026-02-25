#!/usr/bin/env python3
"""Test compound Cartesian motions — combinations of xyz + rpy.

Two phases:
  Phase 1 (interactive): Show the planned trajectory, press Enter to execute each segment.
  Phase 2 (continuous):  Run a continuous sequence without pausing.

Usage:
  python examples/test_compound_motions.py --port 50061
  python examples/test_compound_motions.py --port 50063 --hz 20
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
for p in (_REPO, _REPO / "src"):
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

import numpy as np

from dexcontrol.core.robot_env_client import RobotEnvClient


def fmt(v, prec=3):
    return np.array2string(np.asarray(v), precision=prec, suppress_small=True, separator=", ")


def get_cart(client) -> np.ndarray:
    obs, _ = client.get_observation()
    return np.asarray(obs["robot_state"]["cartesian_position"], dtype=np.float64)


def get_joints(client) -> np.ndarray:
    obs, _ = client.get_observation()
    return np.asarray(obs["robot_state"]["joint_positions"], dtype=np.float64)


def run_segment(client, name: str, actions: list[list[float]], hz: float, record: bool = True):
    """Execute a list of actions at given hz. Returns (cart_before, cart_after, elapsed, records)."""
    dt = 1.0 / hz
    cart_before = get_cart(client)
    records = []
    t0 = time.time()
    for action in actions:
        t_cmd = time.time()
        client.step(action)
        if record:
            cart_now = get_cart(client)
            records.append({"t": time.time() - t0, "cart": cart_now.copy()})
        elapsed_cmd = time.time() - t_cmd
        remaining = dt - elapsed_cmd
        if remaining > 0:
            time.sleep(remaining)
    elapsed = time.time() - t0
    cart_after = get_cart(client)
    return cart_before, cart_after, elapsed, records


def make_actions(dx=0., dy=0., dz=0., dr=0., dp=0., dyaw=0., gripper=0.5, n_steps=1):
    """Create n_steps identical actions."""
    return [[dx, dy, dz, dr, dp, dyaw, gripper]] * n_steps


def print_segment_result(name, cart_before, cart_after, elapsed):
    d = cart_after - cart_before
    xyz = d[:3]
    rpy_deg = np.rad2deg(d[3:6])
    print(f"  {name:30s}  dt={elapsed:.2f}s  "
          f"Δxyz={fmt(xyz, 4)}m  Δrpy={fmt(rpy_deg, 2)}°")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=50061)
    parser.add_argument("--hz", type=float, default=20, help="Control frequency")
    parser.add_argument("--no-reset", action="store_true")
    parser.add_argument("--delta-m", type=float, default=0.04, help="Position step (m)")
    parser.add_argument("--delta-deg", type=float, default=10.0, help="Rotation step (deg)")
    parser.add_argument("--steps", type=int, default=15, help="Steps per segment")
    args = parser.parse_args()

    dm = args.delta_m
    dr = np.deg2rad(args.delta_deg)
    N = args.steps

    L = N * 2  # long segment

    segments_phase1 = [
        # Big diagonal moves — total ~0.6m each direction
        ("Diagonal XY forward",      make_actions(dx=dm, dy=dm, n_steps=L)),
        ("Diagonal XY back",         make_actions(dx=-dm, dy=-dm, n_steps=L)),
        ("Diagonal XZ up-forward",   make_actions(dx=dm, dz=dm, n_steps=L)),
        ("Diagonal XZ back-down",    make_actions(dx=-dm, dz=-dm, n_steps=L)),
        # Reach far out, rotate hard, come back — IK boundary stress
        ("Reach +X far",             make_actions(dx=dm, n_steps=L*2)),
        ("Rotate at full reach",     make_actions(dr=dr*2, dp=-dr*2, dyaw=dr, n_steps=L)),
        ("Un-rotate",                make_actions(dr=-dr*2, dp=dr*2, dyaw=-dr, n_steps=L)),
        ("Return -X",                make_actions(dx=-dm, n_steps=L*2)),
        # Deep Z drop + wide lateral sweep
        ("Drop -Z deep",             make_actions(dz=-dm, n_steps=L)),
        ("Wide sweep +Y low",        make_actions(dy=dm, n_steps=L*2)),
        ("Wide sweep -Y low",        make_actions(dy=-dm, n_steps=L*2)),
        ("Rise +Z back",             make_actions(dz=dm, n_steps=L)),
        # 6-DoF zigzag — alternating compound moves
        ("6-DoF push A",             make_actions(dx=dm, dy=-dm*0.7, dz=dm*0.5, dr=dr, dp=-dr, dyaw=dr*1.5, n_steps=L)),
        ("6-DoF push B",             make_actions(dx=-dm*0.5, dy=dm, dz=-dm*0.3, dr=-dr*1.5, dp=dr*0.5, dyaw=-dr, n_steps=L)),
        ("6-DoF return home",        make_actions(dx=-dm*0.5, dy=-dm*0.3, dz=-dm*0.2, dr=dr*0.5, dp=dr*0.5, dyaw=-dr*0.5, n_steps=L)),
    ]

    # Big circle in XY — 2 full revolutions
    circle_steps = 120
    circle_r = dm
    circle_actions = []
    for i in range(circle_steps):
        angle = 2 * np.pi * i / 60  # 2 laps
        circle_actions.append([circle_r * np.cos(angle), circle_r * np.sin(angle), 0, 0, 0, 0, 0.5])

    # Large square — long sides
    sq_side = L * 2
    square_actions = (
        make_actions(dx=dm, n_steps=sq_side)
        + make_actions(dy=dm, n_steps=sq_side)
        + make_actions(dx=-dm, n_steps=sq_side)
        + make_actions(dy=-dm, n_steps=sq_side)
    )

    # Figure-8 in XZ — bigger amplitude
    fig8_steps = 120
    fig8_actions = []
    for i in range(fig8_steps):
        t = 2 * np.pi * i / fig8_steps
        fig8_actions.append([dm * 1.5 * np.cos(t), 0, dm * np.sin(2 * t), 0, 0, 0, 0.5])

    # Spiral up with yaw — 3 full turns, then spiral back down
    spiral_up_steps = 90
    spiral_actions = []
    for i in range(spiral_up_steps):
        t = 2 * np.pi * i / 30  # 3 turns
        spiral_actions.append([
            dm * np.cos(t), dm * np.sin(t), dm * 0.3,
            0, 0, np.deg2rad(3.0), 0.5,
        ])
    for i in range(spiral_up_steps):
        t = 2 * np.pi * i / 30
        spiral_actions.append([
            dm * np.cos(t + np.pi), dm * np.sin(t + np.pi), -dm * 0.3,
            0, 0, -np.deg2rad(3.0), 0.5,
        ])

    # Lissajous 3D — bigger, more cycles
    liss_steps = 120
    liss_actions = []
    for i in range(liss_steps):
        t = 2 * np.pi * i / liss_steps
        liss_actions.append([
            dm * 1.2 * np.sin(2 * t),
            dm * 1.2 * np.sin(3 * t),
            dm * 0.6 * np.sin(t),
            0, 0, 0, 0.5,
        ])

    # Helix with roll — translation + rotation stress
    helix_steps = 100
    helix_actions = []
    for i in range(helix_steps):
        t = 2 * np.pi * i / 50  # 2 turns
        helix_actions.append([
            dm * np.cos(t), dm * np.sin(t), dm * 0.2,
            np.deg2rad(1.5) * np.sin(t), np.deg2rad(1.5) * np.cos(t), 0, 0.5,
        ])

    segments_phase2 = [
        ("Big Circle XY (2 laps)",   circle_actions),
        ("Big Square XY",            square_actions),
        ("Figure-8 XZ",              fig8_actions),
        ("Spiral up+down + Yaw",     spiral_actions),
        ("Lissajous XYZ",            liss_actions),
        ("Helix + Roll/Pitch",       helix_actions),
    ]

    print(f"\n=== Compound Motion Test ===")
    print(f"Host: {args.host}:{args.port}  Hz: {args.hz}  delta: {dm}m / {args.delta_deg}°  steps/seg: {N}")

    client = RobotEnvClient(
        robot_ip=args.host,
        robot_port=args.port,
        action_space="cartesian_delta",
        do_reset=not args.no_reset,
    )

    if not args.no_reset:
        print("Reset done.")
        time.sleep(1.0)

    home_cart = get_cart(client)
    home_joints = get_joints(client)
    print(f"\nHome pose: xyz={fmt(home_cart[:3])}  rpy={fmt(np.rad2deg(home_cart[3:6]))}°")

    # ── Phase 1: Interactive ──
    print(f"\n{'='*60}")
    print("PHASE 1: Interactive — press Enter before each segment")
    print(f"{'='*60}")

    print("\nPlanned segments:")
    for i, (name, actions) in enumerate(segments_phase1):
        total_d = np.sum(np.array(actions)[:, :6], axis=0)
        total_d[3:6] = np.rad2deg(total_d[3:6])
        print(f"  {i+1}. {name:30s}  total Δxyz={fmt(total_d[:3],3)}m  Δrpy={fmt(total_d[3:6],1)}°  ({len(actions)} steps)")

    for name, actions in segments_phase1:
        input(f"\n  [{name}] Press Enter to run...")
        cb, ca, elapsed, _ = run_segment(client, name, actions, args.hz, record=False)
        print_segment_result(name, cb, ca, elapsed)

    print(f"\n  Phase 1 complete. Resetting...")
    client.reset()
    time.sleep(1.0)

    # ── Phase 2: Continuous ──
    print(f"\n{'='*60}")
    print("PHASE 2: Continuous — runs without pausing")
    print(f"{'='*60}")

    print("\nPlanned sequences:")
    for i, (name, actions) in enumerate(segments_phase2):
        print(f"  {i+1}. {name:30s}  ({len(actions)} steps, ~{len(actions)/args.hz:.1f}s)")

    input("\nPress Enter to start all continuous sequences...")

    home_xyz = home_cart[:3]
    for name, actions in segments_phase2:
        print(f"\n  Running: {name} ({len(actions)} steps)...")
        cb, ca, elapsed, records = run_segment(client, name, actions, args.hz, record=True)
        print_segment_result(name, cb, ca, elapsed)

        if records:
            positions = np.array([r["cart"][:3] for r in records])
            total_path = np.sum(np.sqrt(np.sum(np.diff(positions, axis=0)**2, axis=1)))
            max_dist = np.max(np.sqrt(np.sum((positions - home_xyz)**2, axis=1)))
            end_dist = np.sqrt(np.sum((positions[-1] - home_xyz)**2))
            print(f"    Path: {total_path:.4f}m  Speed: {total_path/elapsed:.4f}m/s  "
                  f"Max from home: {max_dist:.4f}m  End from home: {end_dist:.4f}m")

        client.reset()
        time.sleep(0.5)

    # ── Latency measurement ──
    print(f"\n{'='*60}")
    print("LATENCY TEST: 50 steps, measure round-trip time")
    print(f"{'='*60}")

    latencies = []
    for i in range(50):
        t0 = time.time()
        client.step([0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5])
        latencies.append((time.time() - t0) * 1000)

    latencies = np.array(latencies)
    print(f"  Step RPC latency (ms): mean={latencies.mean():.1f}  std={latencies.std():.1f}  "
          f"min={latencies.min():.1f}  max={latencies.max():.1f}  p95={np.percentile(latencies, 95):.1f}")

    cart_cmd = get_cart(client)
    time.sleep(0.2)
    cart_delayed = get_cart(client)
    settling_delta = np.abs(cart_delayed - cart_cmd)
    print(f"  Settling (200ms after last cmd): max Δ = {np.max(settling_delta[:3])*1000:.2f}mm")

    print(f"\nDone.")
    client.close()


if __name__ == "__main__":
    main()
