# Copyright (C) 2025 Dexmate Inc.
#
# This software is dual-licensed:
#
# 1. GNU Affero General Public License v3.0 (AGPL-3.0)
#    See LICENSE-AGPL for details
#
# 2. Commercial License
#    For commercial licensing terms, contact: contact@dexmate.ai

"""Wuji Hand adapter for dexcontrol.

Wraps the wuji-sdk to expose the same interface as DexGripper/Hand/
RobotiqGripper so a Wuji Hand (first gen, USB) or Wuji Hand 2 (Ethernet)
can be used as a drop-in end-effector in VegaRobot. Joint space is the
20-element firmware vector (5 fingers x 4 joints, finger-major
thumb/index/middle/ring/pinky), radians.

The two hand generations expose different command channels in wuji-sdk —
first gen drives a low-pass realtime controller, Hand 2 streams MIT
position commands through a joint_command publisher — so the adapter
auto-detects the model at connect time (isinstance(hand, WujiHand2)) and
hides the difference behind set_joint_pos().

Same worker-thread contract as RobotiqGripper: the cached position is
updated only from real joint_states frames — `set_joint_pos` never writes
the commanded target into the cache. A watchdog stops republishing
commands when the state stream goes silent (USB unplug / link loss) and
resumes when frames come back.

Tactile: the receive path is always wired up (unless WUJI_TACTILE=off);
the recorded `hand_tactile` schema is pinned per hand model:
  - Wuji Hand 2: 15-vector — per-finger aggregate [fx, fy, fz] (newtons),
    finger-major, decoded from the self-describing fingertip info contract.
  - Wuji Hand (first gen, plug-in tactile glove): 40-vector — the pressure
    frame mean-pooled to a fixed 5x8 grid (NaN-invalid cells excluded).
    Raw frames additionally cached when WUJI_TACTILE_RAW=1.
A hand without the tactile hardware simply reports None (key never set
downstream).

Requirements:
    pip install wuji-sdk
"""

from __future__ import annotations

import json
import os
import struct
import threading
import time
from queue import Empty, Full, Queue

import numpy as np
from loguru import logger

try:
    from wuji_sdk import (
        DeviceType,
        JointCommand,
        LowPass,
        SdkManager,
        WujiHand2,
    )
except ImportError as e:
    raise ImportError(
        "wuji-sdk is not installed. Run: pip install wuji-sdk"
    ) from e

_N_FINGERS = 5
_JOINTS_PER_FINGER = 4
_N_JOINTS = _N_FINGERS * _JOINTS_PER_FINGER

_FINGERS = ["thumb", "index", "middle", "ring", "pinky"]

# Firmware joint order is finger-major. Names follow the wuji URDF
# convention (wuji-description): thumb j1..j4 = cmc_flex/cmc_abd/mcp/ip,
# other fingers j1..j4 = mcp_flex/mcp_abd/pip/dip.
_JOINT_NAMES = [f"{f}_j{j + 1}" for f in _FINGERS for j in range(_JOINTS_PER_FINGER)]

# Fallback soft limits (radians, [lower, upper] per joint) assembled from the
# published Hand 2 joint ranges. Bring-up placeholders: replaced by
# firmware-reported limits when the SDK exposes an accessor (checked at
# connect), and always overridable via the constructor.
_FALLBACK_LIMITS = np.array(
    # thumb: cmc_flex, cmc_abd, mcp, ip
    [[-1.19, 1.29], [-1.48, 0.70], [-1.05, 1.57], [-1.05, 1.57]]
    # index/middle/ring/pinky: mcp_flex, mcp_abd, pip, dip
    + [[-1.05, 1.57], [-0.35, 0.35], [-1.05, 2.09], [-1.05, 2.09]] * 4,
    dtype=np.float64,
)

# Predefined poses (radians, firmware order). "open" is the neutral extended
# hand; "close" is a conservative power-grasp curl. Tuned on hardware at
# bring-up (M1) — keep conservative until then.
_CLOSE_POSE = np.array(
    [0.8, 0.5, 0.6, 0.6] + [1.2, 0.0, 1.2, 0.9] * 4, dtype=np.float64
)
_POSE_POOL = {
    "open": np.zeros(_N_JOINTS, dtype=np.float64),
    "close": _CLOSE_POSE,
}

# Fingertip contract field type -> struct format char (Hand 2 tactile).
# Unknown types raise, never guess — mirrors the official example.
_FIELD_FMT = {"i8": "<b", "u8": "<B", "i16": "<h", "u16": "<H",
              "i32": "<i", "u32": "<I", "f32": "<f"}

# Pooled tactile grid for the first-gen tactile glove pressure frame.
_GLOVE_POOL_ROWS = 5
_GLOVE_POOL_COLS = 8

# Serial-number handedness convention (4th char): J = left, K = right.
_SN_HANDEDNESS = {"J": "left", "K": "right"}


def _sn_handedness(sn: str) -> str | None:
    if len(sn) >= 4:
        return _SN_HANDEDNESS.get(sn[3].upper())
    return None


class WujiHandAdapter:
    """Wuji Hand (1 or 2) controller compatible with the VegaRobot hand interface."""

    N_JOINTS = _N_JOINTS
    joint_name = list(_JOINT_NAMES)

    def __init__(
        self,
        handedness: str,
        sn: str | None = None,
        effort_limit: float = 1.5,
        lowpass_cutoff_hz: float = 5.0,
        mit_kp: float = 3.0,
        mit_kd: float = 0.05,
        command_hz: float = 100.0,
        init_timeout: float = 15.0,
        joint_limits: np.ndarray | None = None,
    ) -> None:
        """Connect to a Wuji hand and prepare it for streaming commands.

        Args:
            handedness: "left" | "right" — which hand this arm's server owns.
                Used to pick the device when no serial number is given.
            sn: Explicit device serial number; wins over handedness.
            effort_limit: Per-joint current cap in Amps. Keep low for bring-up.
            lowpass_cutoff_hz: First-gen realtime controller filter cutoff.
            mit_kp/mit_kd: Hand 2 MIT impedance gains.
            command_hz: Worker republish rate (Hand 2 needs a continuous MIT
                command stream; this also acts as a keepalive).
            init_timeout: Seconds to wait for scan/enable to complete.
            joint_limits: Optional (20, 2) [lower, upper] radians override.
        """
        if handedness not in ("left", "right"):
            raise ValueError(f"handedness must be 'left' or 'right', got {handedness!r}")
        self._handedness = handedness
        self._tactile_enabled = os.environ.get("WUJI_TACTILE", "on").lower() not in (
            "off", "0", "false",
        )
        self._tactile_raw = os.environ.get("WUJI_TACTILE_RAW", "0").lower() in (
            "1", "on", "true",
        )

        self._command_hz = float(command_hz)
        self._manager = SdkManager.instance()
        self._hand = self._connect(sn, init_timeout)
        self._is_hand2 = isinstance(self._hand, WujiHand2)

        self._limits = self._resolve_limits(joint_limits)

        # Locks & caches. _cached_pos is measured state only (worker-updated).
        self._state_lock = threading.Lock()
        self._cached_pos = np.zeros(_N_JOINTS, dtype=np.float64)
        self._cached_tactile: np.ndarray | None = None
        self._cached_tactile_raw: np.ndarray | None = None
        self._last_state_time = 0.0

        # Command channel (model-specific) + state/tactile subscriptions.
        self._publisher = None
        self._rt_ctx = None
        self._rt = None
        self._configure_control(effort_limit, mit_kp, mit_kd,
                                lowpass_cutoff_hz, init_timeout)
        self._state_sub = self._hand.joint_states().subscribe()
        self._tactile_subs: dict[str, object] = {}
        self._tactile_decoders: dict[str, object] = {}
        self._glove_frame_sub = None
        if self._tactile_enabled:
            self._setup_tactile()

        # Worker thread (same drain-queue pattern as RobotiqGripper).
        self._command_queue: Queue[np.ndarray] = Queue(maxsize=1)
        self._latest_target: np.ndarray | None = None
        self._stop_event = threading.Event()
        self._connected = True
        self._worker = threading.Thread(
            target=self._worker_loop,
            name=f"wuji-hand-{handedness}",
            daemon=True,
        )
        self._worker.start()

        logger.info(
            "Wuji {} ({}) ready: sn={}, tactile={}, async worker started.",
            "Hand 2" if self._is_hand2 else "Hand",
            handedness,
            self.serial_number,
            "on" if (self._tactile_subs or self._glove_frame_sub) else "off",
        )

    # ------------------------------------------------------------------
    # Connection / configuration
    # ------------------------------------------------------------------

    def _connect(self, sn: str | None, init_timeout: float):
        """Scan and connect the requested hand device."""
        deadline = time.monotonic() + init_timeout
        hand_types = (DeviceType.WujiHand, DeviceType.WujiHand2)
        while True:
            devices = [d for d in self._manager.scan() if d.device_type in hand_types]
            if sn is not None:
                match = [d for d in devices if d.sn == sn]
                if match:
                    return self._manager.connect(
                        sn=sn, device_name=f"wuji_hand_{self._handedness}"
                    )
            else:
                # Prefer the SN handedness convention (4th char J=left/K=right);
                # verify after connect when the SDK reports handedness.
                candidates = [d for d in devices if _sn_handedness(d.sn) == self._handedness]
                if not candidates and devices:
                    candidates = devices
                for dev in candidates:
                    hand = self._manager.connect(
                        sn=dev.sn, device_name=f"wuji_hand_{self._handedness}"
                    )
                    reported = getattr(hand, "handedness_name", lambda: None)()
                    if reported is None or str(reported).lower() == self._handedness:
                        return hand
                    logger.warning(
                        "Wuji hand sn={} reports handedness {!r}, want {!r}; skipping.",
                        dev.sn, reported, self._handedness,
                    )
                    self._manager.disconnect_all()
            if time.monotonic() >= deadline:
                raise ConnectionError(
                    f"No Wuji hand found for handedness={self._handedness!r} "
                    f"(sn={sn!r}) within {init_timeout}s"
                )
            time.sleep(0.5)

    def _configure_control(self, effort_limit, mit_kp, mit_kd,
                           lowpass_cutoff_hz, init_timeout) -> None:
        """Set limits/gains, enable motors, open the model's command channel."""
        if self._is_hand2:
            self._hand.effort_limit().set(effort_limit)
            self._hand.mit_params().set((mit_kp, mit_kd))
            self._hand.enable()
            self._wait_enabled(min(init_timeout, 5.0))
            self._publisher = self._hand.joint_command().publish()
        else:
            self._hand.set_all_effort_limit(effort_limit)
            self._hand.enable()
            # The realtime controller is a context manager in the SDK examples;
            # we hold it open for the adapter's lifetime and close it in
            # shutdown(). It interpolates our command_hz targets up to the
            # 1 kHz PDO tick (LowPass).
            self._rt_ctx = self._hand.realtime_controller(
                LowPass(cutoff_hz=lowpass_cutoff_hz)
            )
            self._rt = self._rt_ctx.__enter__()

    def _wait_enabled(self, timeout: float) -> None:
        """Hand 2: wait until every online motor reports ext_state==2 (Enabled).

        enable() is an action; streaming commands too early can race ahead of
        the enable completing (per the official publish example).
        """
        deadline = time.monotonic() + timeout
        diag_sub = self._hand.joint_diagnostics().subscribe()
        try:
            while time.monotonic() < deadline:
                time.sleep(0.2)
                frame = diag_sub.recv()
                if frame is None or not frame.joints:
                    continue
                if all(e.status_word.ext_state == 2 for e in frame.joints):
                    return
            logger.warning(
                "Wuji Hand 2 ({}): enable timeout — not all motors reached "
                "Enabled; commands may be dropped until they do.",
                self._handedness,
            )
        finally:
            diag_sub.close()

    def _resolve_limits(self, override: np.ndarray | None) -> np.ndarray:
        """Joint limits: explicit override > SDK/firmware-reported > fallback table."""
        if override is not None:
            limits = np.asarray(override, dtype=np.float64)
            if limits.shape != (_N_JOINTS, 2):
                raise ValueError(f"joint_limits must be (20, 2), got {limits.shape}")
            return limits
        # No joint-limit accessor is documented in wuji_sdk; probe common names
        # so firmware limits win automatically once the SDK exposes them.
        for attr in ("joint_limits", "joint_soft_limits"):
            getter = getattr(self._hand, attr, None)
            if getter is None:
                continue
            try:
                raw = getter() if callable(getter) else getter
                raw = getattr(raw, "get", lambda: raw)()
                limits = np.asarray(raw, dtype=np.float64).reshape(_N_JOINTS, 2)
                logger.info("Wuji hand joint limits read from SDK ({}).", attr)
                return limits
            except Exception:
                continue
        return _FALLBACK_LIMITS.copy()

    def _setup_tactile(self) -> None:
        """Wire up the model's tactile stream; degrade to no-tactile on failure."""
        try:
            if self._is_hand2:
                accessors = {
                    "thumb": self._hand.fingertip_thumb_data,
                    "index": self._hand.fingertip_index_data,
                    "middle": self._hand.fingertip_middle_data,
                    "ring": self._hand.fingertip_ring_data,
                    "pinky": self._hand.fingertip_pinky_data,
                }
                for i, name in enumerate(_FINGERS):
                    info = self._hand.get_fingertip_info(i)
                    fmt = json.loads(info.format)
                    if fmt["v"] != 1 or fmt["encoding"] != "point_array":
                        raise ValueError(f"unsupported fingertip format: {fmt.get('encoding')}")
                    self._tactile_decoders[name] = self._make_aggregate_decoder(fmt)
                    self._tactile_subs[name] = accessors[name]().subscribe()
            else:
                # First-gen tactile glove pairs at connect time only; a short
                # status poll tells whether one is attached.
                status_sub = self._hand.tactile_status().subscribe()
                attached = False
                deadline = time.monotonic() + 1.0
                try:
                    while time.monotonic() < deadline:
                        status = status_sub.recv()
                        if status is not None:
                            attached = status.state == 1
                            break
                        time.sleep(0.02)
                finally:
                    status_sub.close()
                if attached:
                    self._glove_frame_sub = self._hand.tactile.subscribe_pressure_frame()
                else:
                    logger.info(
                        "Wuji hand ({}): no tactile glove attached — "
                        "hand_tactile disabled.", self._handedness,
                    )
        except Exception as e:
            logger.warning(
                "Wuji hand ({}): tactile setup failed ({}); continuing without "
                "tactile.", self._handedness, e,
            )
            for sub in self._tactile_subs.values():
                try:
                    sub.close()
                except Exception:
                    pass
            self._tactile_subs.clear()
            self._tactile_decoders.clear()
            self._glove_frame_sub = None

    @staticmethod
    def _make_aggregate_decoder(fmt: dict):
        """Decoder for a fingertip frame's aggregate [fx, fy, fz] (newtons).

        Layout comes from the self-describing info contract (never hardcoded);
        the aggregate block sits after point_count * point_stride bytes.
        """
        base = fmt["point_count"] * fmt["point_stride"]
        fields = {d["name"]: d for d in fmt["aggregate_fields"]}
        expect = base + fmt["aggregate_stride"]

        def decode(data: bytes) -> np.ndarray:
            if len(data) != expect:
                raise ValueError(f"fingertip frame length {len(data)} != {expect}")
            out = np.empty(3, dtype=np.float64)
            for k, name in enumerate(("fx", "fy", "fz")):
                d = fields[name]
                out[k] = struct.unpack_from(
                    _FIELD_FMT[d["type"]], data, base + d["offset"]
                )[0] * d.get("scale", 1.0)
            return out

        return decode

    # ------------------------------------------------------------------
    # Worker
    # ------------------------------------------------------------------

    def _worker_loop(self) -> None:
        """Background loop: republish the latest target, refresh state/tactile."""
        period = 1.0 / self._command_hz
        while not self._stop_event.is_set():
            t0 = time.monotonic()

            # Drain queue — only keep the newest command.
            try:
                target = self._command_queue.get(timeout=period)
                while True:
                    target = self._command_queue.get_nowait()
            except Empty:
                target = None
            if target is not None:
                self._latest_target = target

            # Republish the latest target (continuous stream keeps Hand 2 MIT
            # commands flowing and doubles as a keepalive). Skip while the
            # state stream is silent — don't spam a dead link.
            if self._latest_target is not None and self._connected:
                try:
                    self._send(self._latest_target)
                except Exception as e:
                    logger.warning("Wuji hand worker command error: {}", e)

            self._refresh_state()
            self._refresh_tactile()

            # Watchdog on the joint_states stream.
            stale = (time.monotonic() - self._last_state_time) > 1.0
            if stale and self._connected and self._last_state_time > 0.0:
                self._connected = False
                logger.warning(
                    "Wuji hand ({}): joint_states silent >1s — link lost? "
                    "Pausing command stream until frames resume.",
                    self._handedness,
                )
            elif not stale and not self._connected:
                self._connected = True
                logger.info("Wuji hand ({}): joint_states resumed.", self._handedness)

            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)

    def _send(self, target: np.ndarray) -> None:
        if self._is_hand2:
            self._publisher.send([JointCommand(float(p), 0.0, 0.0) for p in target])
        else:
            self._rt.set_target_position(target.tolist())

    def _refresh_state(self) -> None:
        """Drain joint_states, keep the newest frame in the cache."""
        try:
            latest = None
            while True:
                frame = self._state_sub.recv()
                if frame is None:
                    break
                latest = frame
            if latest is None:
                return
            if self._is_hand2:
                # Variable-length list of online joints; update per nid so a
                # briefly offline joint keeps its last value.
                with self._state_lock:
                    for entry in latest.joints:
                        if 0 <= entry.nid < _N_JOINTS:
                            self._cached_pos[entry.nid] = entry.position
            else:
                pos = np.asarray(latest.position, dtype=np.float64)
                if pos.size == _N_JOINTS:
                    with self._state_lock:
                        self._cached_pos = pos
            self._last_state_time = time.monotonic()
        except Exception as e:
            logger.warning("Wuji hand worker state refresh error: {}", e)

    def _refresh_tactile(self) -> None:
        if not (self._tactile_subs or self._glove_frame_sub):
            return
        try:
            if self._is_hand2:
                updated = False
                current = (
                    self._cached_tactile.copy()
                    if self._cached_tactile is not None
                    else np.zeros(3 * _N_FINGERS, dtype=np.float64)
                )
                for i, name in enumerate(_FINGERS):
                    latest = None
                    while True:
                        frame = self._tactile_subs[name].recv()
                        if frame is None:
                            break
                        latest = frame
                    if latest is None:
                        continue
                    current[3 * i : 3 * i + 3] = self._tactile_decoders[name](
                        bytes(latest.data)
                    )
                    updated = True
                if updated:
                    with self._state_lock:
                        self._cached_tactile = current
            else:
                latest = None
                while True:
                    frame = self._glove_frame_sub.recv()
                    if frame is None:
                        break
                    latest = frame
                if latest is None:
                    return
                raw = np.asarray(latest.pressure, dtype=np.float64)
                with self._state_lock:
                    self._cached_tactile = self._pool_pressure(raw)
                    if self._tactile_raw:
                        self._cached_tactile_raw = raw
        except Exception as e:
            logger.warning("Wuji hand worker tactile refresh error: {}", e)

    @staticmethod
    def _pool_pressure(raw: np.ndarray) -> np.ndarray:
        """Mean-pool the tactile-glove pressure frame to a fixed 5x8 grid.

        The wire frame is a row-major grid (20x31 via wuji_sdk; 24x32 raw) with
        non-finite values marking invalid taxels. Pooling keeps coarse spatial
        contact information at a fraction of the size; the exact cell->finger
        mapping is undocumented, so the raw frame can be recorded alongside
        with WUJI_TACTILE_RAW=1.
        """
        grid = None
        for rows, cols in ((20, 31), (24, 32)):
            if raw.size == rows * cols:
                grid = raw.reshape(rows, cols)
                break
        if grid is None:
            # Unknown layout: pool the flat vector into fixed-size chunks.
            chunks = np.array_split(raw, _GLOVE_POOL_ROWS * _GLOVE_POOL_COLS)
            return np.array(
                [np.nan_to_num(np.nanmean(c)) if c.size else 0.0 for c in chunks]
            )
        pooled = np.empty(_GLOVE_POOL_ROWS * _GLOVE_POOL_COLS, dtype=np.float64)
        row_groups = np.array_split(np.arange(grid.shape[0]), _GLOVE_POOL_ROWS)
        col_groups = np.array_split(np.arange(grid.shape[1]), _GLOVE_POOL_COLS)
        k = 0
        for rg in row_groups:
            for cg in col_groups:
                cell = grid[np.ix_(rg, cg)]
                pooled[k] = np.nan_to_num(np.nanmean(cell)) if np.isfinite(cell).any() else 0.0
                k += 1
        return pooled

    # ------------------------------------------------------------------
    # VegaRobot hand interface
    # ------------------------------------------------------------------

    def get_joint_pos(self) -> np.ndarray:
        """Return current measured joint positions (20-element array, radians)."""
        with self._state_lock:
            return self._cached_pos.copy()

    def set_joint_pos(self, joint_pos, wait_time: float = 0.0, **_kwargs) -> None:
        """Command the hand to absolute joint positions (non-blocking).

        Args:
            joint_pos: Target positions, 20 elements, radians. Clamped to the
                soft joint limits before sending.
            wait_time: Seconds to sleep after enqueuing the command.
        """
        target = np.asarray(joint_pos, dtype=np.float64).reshape(-1)
        if target.size != _N_JOINTS:
            raise ValueError(
                f"Wuji hand expects {_N_JOINTS} joint targets, got {target.size}"
            )
        target = np.clip(target, self._limits[:, 0], self._limits[:, 1])
        try:
            self._command_queue.put_nowait(target)
        except Full:
            try:
                self._command_queue.get_nowait()
            except Empty:
                pass
            try:
                self._command_queue.put_nowait(target)
            except Full:
                pass
        if wait_time > 0.0:
            time.sleep(wait_time)

    def open_hand(self, wait_time: float = 0.0, **_kwargs) -> None:
        """Move to the neutral extended pose (non-blocking)."""
        self.set_joint_pos(_POSE_POOL["open"], wait_time=wait_time)

    def close_hand(self, wait_time: float = 0.0, **_kwargs) -> None:
        """Move to the power-grasp curl pose (non-blocking)."""
        self.set_joint_pos(_POSE_POOL["close"], wait_time=wait_time)

    def get_predefined_pose(self, name: str) -> np.ndarray:
        """Return a predefined pose by name ('open' or 'close')."""
        if name not in _POSE_POOL:
            raise KeyError(f"Unknown predefined pose '{name}'. Available: {list(_POSE_POOL)}")
        return _POSE_POOL[name].copy()

    def get_hand_tactile(self) -> np.ndarray | None:
        """Latest tactile vector, or None when the hand has no tactile stream.

        Hand 2 → (15,) per-finger aggregate [fx, fy, fz] N. First gen with the
        tactile glove → (40,) mean-pooled pressure grid.
        """
        with self._state_lock:
            return None if self._cached_tactile is None else self._cached_tactile.copy()

    def get_hand_tactile_raw(self) -> np.ndarray | None:
        """Latest raw tactile-glove frame (WUJI_TACTILE_RAW=1 only)."""
        with self._state_lock:
            return (
                None if self._cached_tactile_raw is None
                else self._cached_tactile_raw.copy()
            )

    @property
    def joint_limits(self) -> np.ndarray:
        """(20, 2) [lower, upper] soft limits in radians."""
        return self._limits.copy()

    @property
    def model(self) -> str:
        """Detected hand model: 'wuji_hand' (first gen) or 'wuji_hand_2'."""
        return "wuji_hand_2" if self._is_hand2 else "wuji_hand"

    @property
    def has_tactile(self) -> bool:
        return bool(self._tactile_subs or self._glove_frame_sub)

    @property
    def is_connected(self) -> bool:
        """False while the joint_states stream has been silent >1s."""
        return self._connected

    @property
    def serial_number(self) -> str:
        return str(getattr(self._hand, "serial_number", "unknown"))

    def shutdown(self) -> None:
        """Stop the worker, close channels, and disable the motors."""
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        for sub in self._tactile_subs.values():
            try:
                sub.close()
            except Exception:
                pass
        if self._glove_frame_sub is not None:
            try:
                self._glove_frame_sub.close()
            except Exception:
                pass
        try:
            self._state_sub.close()
        except Exception:
            pass
        try:
            if self._publisher is not None:
                self._publisher.close()
            if self._rt_ctx is not None:
                self._rt_ctx.__exit__(None, None, None)
        except Exception:
            pass
        try:
            self._hand.disable()
        except Exception:
            pass
        try:
            self._manager.disconnect_all()
        except Exception:
            pass
        logger.info("Wuji hand ({}) shut down.", self._handedness)
