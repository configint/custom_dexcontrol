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

Tactile availability is recorded PER MODEL in TACTILE_BY_MODEL — the current
wuji v1/v2 units ship without tactile hardware (vendor-confirmed), so both
map to False; flip the entry when a tactile-equipped unit arrives. Setting
WUJI_TACTILE=on/off overrides the table (bench experiments). When enabled,
the tactile stream MUST come up (device present + first frame within the
init window) or construction fails loudly: a dataset must never be collected
believing tactile is on while the column silently never arrives.
Schema per hand model when enabled:
  - Wuji Hand v2: 15-vector — per-finger aggregate [fx, fy, fz] (newtons),
    finger-major, decoded from the self-describing fingertip info contract.
  - Wuji Hand v1 (plug-in tactile glove): 40-vector — the pressure frame
    mean-pooled to a fixed 5x8 grid (NaN-invalid cells excluded). Raw frames
    additionally cached when WUJI_TACTILE_RAW=1.

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
# This is the Wuji Hand 2 FIRMWARE convention, and the only thing the SDK's own
# `SdkManager.connect(handedness=...)` goes by — its Handedness docstring says
# devices whose SN does not follow it "will be filtered out at discovery". Our
# v1 hands ship plain STM32-UID serials (e.g. "367A39773134"), so connect-by-
# handedness cannot see them at all: v1 must be selected by SN and verified
# after connect via handedness_name() (the handedness SDO, 0=Right / 1=Left).
_SN_HANDEDNESS = {"J": "left", "K": "right"}

# Per-unit hand map: {"left": "<sn>", "right": "<sn>"}. Unit-specific values
# belong on the unit, never in the repo or a shared playbook, so the file lives
# on the robot host and is written once by examples/wuji/identify_hands.py.
# Override the location with WUJI_HAND_MAP.
_HAND_MAP_PATH = os.environ.get(
    "WUJI_HAND_MAP", os.path.expanduser("~/.wuji/hands.json")
)

# Tactile availability by detected hand model — the source of truth for
# whether a tactile stream is expected on this end-effector. Current wuji
# v1/v2 units ship WITHOUT tactile hardware (vendor-confirmed). Flip an entry
# when a tactile-equipped unit arrives; WUJI_TACTILE=on/off overrides the
# table without a code change. (vega_hand's dexbot fingertip force is a
# separate always-on path in VegaRobot, not governed by this table.)
TACTILE_BY_MODEL = {
    "wuji_hand_v1": False,
    "wuji_hand_v2": False,
}

# Accepted WUJI_TACTILE overrides. Anything else raises rather than being
# read as "off" — see the resolution in __init__.
_TACTILE_ENV_VALUES = {
    "on": True, "1": True, "true": True, "yes": True, "y": True,
    "enable": True, "enabled": True,
    "off": False, "0": False, "false": False, "no": False, "n": False,
    "disable": False, "disabled": False,
}


def _sn_handedness(sn: str) -> str | None:
    if len(sn) >= 4:
        return _SN_HANDEDNESS.get(sn[3].upper())
    return None


def _sn_from_hand_map(handedness: str) -> str | None:
    """Serial number pinned for this handedness on this unit, if configured.

    A missing file is normal (auto-selection still works). A malformed one is
    not: silently ignoring it would bind an arm to whichever hand answered
    first, which is the left/right swap this file exists to prevent.
    """
    try:
        with open(_HAND_MAP_PATH) as f:
            data = json.load(f)
    except FileNotFoundError:
        return None
    except (OSError, ValueError) as e:
        raise RuntimeError(f"Cannot read Wuji hand map {_HAND_MAP_PATH}: {e}") from e
    if not isinstance(data, dict):
        raise RuntimeError(f"Wuji hand map {_HAND_MAP_PATH} must be a JSON object")
    sn = data.get(handedness)
    if sn is None:
        return None
    if not isinstance(sn, str) or not sn:
        raise RuntimeError(
            f"Wuji hand map {_HAND_MAP_PATH}: {handedness!r} must be a non-empty string"
        )
    return sn


def _update_hand_map(handedness: str, sn: str) -> None:
    """Record a VERIFIED handedness->sn binding (best effort).

    Called only after the hand itself confirmed its side over the SDO, so a
    stale map (old serials after a hand swap) heals itself on the first
    successful start instead of paying the fallback grace forever. Never
    called for the unverified single-hand acceptance. Both arm servers may
    write concurrently — an exclusive flock serializes them.
    """
    import fcntl

    path = os.path.expanduser(_HAND_MAP_PATH)
    try:
        parent = os.path.dirname(path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(path, "a+") as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            f.seek(0)
            raw = f.read()
            try:
                data = json.loads(raw) if raw.strip() else {}
            except ValueError:
                data = {}
            if not isinstance(data, dict):
                data = {}
            if data.get(handedness) == sn:
                return
            data[handedness] = sn
            f.seek(0)
            f.truncate()
            json.dump(data, f, indent=2)
            f.write("\n")
        logger.info("Wuji hand map updated: {} -> sn={} ({}).",
                    handedness, sn, path)
    except OSError as e:
        logger.warning("Could not update Wuji hand map {}: {}", path, e)


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
        self._tactile_raw = os.environ.get("WUJI_TACTILE_RAW", "0").lower() in (
            "1", "on", "true",
        )

        self._command_hz = float(command_hz)
        self._manager = SdkManager.instance()
        self._hand = self._connect(sn, init_timeout)
        self._is_hand2 = isinstance(self._hand, WujiHand2)
        # Tactile: per-model default from TACTILE_BY_MODEL (resolved here,
        # after connect, once the model is known); WUJI_TACTILE=on/off is an
        # explicit override. When enabled, a missing/dead tactile stream is a
        # hard startup error (see _setup_tactile) — never a silent skip.
        _tactile_env = os.environ.get("WUJI_TACTILE", "").strip().lower()
        if _tactile_env in _TACTILE_ENV_VALUES:
            self._tactile_enabled = _TACTILE_ENV_VALUES[_tactile_env]
        elif _tactile_env:
            # Never fall back to the table on an unrecognised value: silently
            # reading "WUJI_TACTILE=yes" as OFF is exactly the "collected a
            # dataset believing tactile was on" failure this design forbids.
            raise ValueError(
                f"WUJI_TACTILE={_tactile_env!r} is not a recognised value "
                f"(use one of {sorted(_TACTILE_ENV_VALUES)}, or unset it to "
                f"follow the per-model default)"
            )
        else:
            self._tactile_enabled = TACTILE_BY_MODEL.get(self.model, False)

        # Everything below can touch live hardware; a failure part-way must
        # not leave motors enabled / channels open with no owner.
        self._publisher = None
        self._rt_ctx = None
        self._rt = None
        self._state_sub = None
        self._tactile_subs: dict[str, object] = {}
        self._tactile_decoders: dict[str, object] = {}
        self._glove_frame_sub = None
        try:
            self._limits = self._resolve_limits(joint_limits)

            # Locks & caches. _cached_pos is measured state only (worker-updated).
            self._state_lock = threading.Lock()
            self._cached_pos = np.zeros(_N_JOINTS, dtype=np.float64)
            self._cached_tactile: np.ndarray | None = None
            self._cached_tactile_raw: np.ndarray | None = None
            # Anchor the watchdog at construction: if the hand NEVER produces a
            # state frame, commands pause after the stale window instead of
            # republishing to a never-alive link forever.
            self._last_state_time = time.monotonic()
            # Hand 2 joint ids: assumed 0-based; auto-detected from the first
            # frame (a 1..20 range means 1-based) since the SDK contract is
            # not documented. None until detected.
            self._nid_offset: int | None = None

            # Command channel (model-specific) + state/tactile subscriptions.
            self._configure_control(effort_limit, mit_kp, mit_kd,
                                    lowpass_cutoff_hz, init_timeout)
            self._state_sub = self._hand.joint_states().subscribe()
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
            # Tactile opted in → it must actually flow before we report ready.
            if self._tactile_enabled:
                self._wait_first_tactile_frame()
        except Exception:
            # Stop the worker BEFORE tearing channels down. Once the worker is
            # running (the tactile first-frame wait is the one failure that can
            # happen after that), teardown nulls the very SDK handles it polls
            # 100x/s, so a live worker would either flood NoneType errors that
            # bury the real exception or race disable()/disconnect_all() inside
            # the native SDK.
            self._connected = False
            _stop = getattr(self, "_stop_event", None)
            if _stop is not None:
                _stop.set()
            _worker = getattr(self, "_worker", None)
            if _worker is not None and _worker.is_alive():
                _worker.join(timeout=2.0)
            self._teardown_channels()
            try:
                self._hand.disable()
            except Exception:
                pass
            try:
                self._manager.disconnect_all()
            except Exception:
                pass
            raise

        logger.info(
            "Wuji {} ({}) ready: sn={}, tactile={}, async worker started.",
            "Hand v2" if self._is_hand2 else "Hand v1",
            handedness,
            self.serial_number,
            "on" if self._tactile_enabled else "off",
        )

    # ------------------------------------------------------------------
    # Connection / configuration
    # ------------------------------------------------------------------

    def _connect(self, sn: str | None, init_timeout: float):
        """Scan and connect the requested hand device.

        Selection order: explicit sn > per-unit hand map (~/.wuji/hands.json) >
        SN handedness convention > any hand whose reported handedness matches.
        A stale map entry (serial no longer attached, e.g. after a hand swap)
        falls back to auto-selection after a short grace window.
        """
        sn_from_map = False
        if sn is None:
            sn = _sn_from_hand_map(self._handedness)
            if sn is not None:
                sn_from_map = True
                logger.info(
                    "Wuji hand ({}): using sn={} from {}.",
                    self._handedness, sn, _HAND_MAP_PATH,
                )
        deadline = time.monotonic() + init_timeout
        # A map entry is a hint, not a contract: after swapping hands (e.g.
        # v1 -> v2) the file still names serials that no longer exist, and
        # waiting for them would time the connect out. Give the mapped SN a
        # grace window to enumerate, then fall back to auto-selection once
        # OTHER hands are visible. An explicit --wuji-sn never falls back.
        map_grace_deadline = time.monotonic() + min(5.0, init_timeout / 2)
        hand_types = (DeviceType.WujiHand, DeviceType.WujiHand2)
        while True:
            devices = [d for d in self._manager.scan() if d.device_type in hand_types]
            if sn is not None:
                match = [d for d in devices if d.sn == sn]
                if match:
                    return self._manager.connect(
                        sn=sn, device_name=f"wuji_hand_{self._handedness}"
                    )
                if (sn_from_map and devices
                        and time.monotonic() >= map_grace_deadline):
                    logger.warning(
                        "Wuji hand map sn={} is not among the attached hands "
                        "{}; the map ({}) looks stale — falling back to "
                        "auto-selection. Refresh it with "
                        "`python examples/wuji/identify_hands.py --auto`.",
                        sn, [d.sn for d in devices], _HAND_MAP_PATH,
                    )
                    sn = None
                    continue
            else:
                # Prefer the SN handedness convention (4th char J=left/K=right);
                # verify after connect when the SDK reports handedness.
                candidates = [d for d in devices if _sn_handedness(d.sn) == self._handedness]
                if not candidates and devices:
                    candidates = devices
                for dev in candidates:
                    try:
                        hand = self._manager.connect(
                            sn=dev.sn, device_name=f"wuji_hand_{self._handedness}"
                        )
                    except Exception as e:
                        # Auto-selection has to open a hand to ask which side it
                        # is, so with both arm servers starting at once each can
                        # be probing the other's hand when it claims the USB
                        # device. That is transient: skip this candidate and let
                        # the retry loop pick it up once the owner settles.
                        logger.warning(
                            "Wuji hand sn={} could not be opened while looking for the "
                            "{} hand ({}); retrying.", dev.sn, self._handedness, e,
                        )
                        continue
                    # handedness_name() decodes the handedness SDO after
                    # connect (0=Right / 1=Left) and is the authority WHEN the
                    # device answers. Hand 2 does not implement it — its side
                    # lives in the SN convention instead (the same convention
                    # the SDK's own connect-by-handedness filters by).
                    reported = getattr(hand, "handedness_name", lambda: None)()
                    normalized = str(reported).lower() if reported is not None else ""
                    if normalized == self._handedness:
                        # SDO-verified: pin it in the hand map so the next
                        # start connects deterministically (heals stale maps).
                        _update_hand_map(self._handedness, dev.sn)
                        return hand
                    if normalized in ("", "unknown"):
                        if _sn_handedness(dev.sn) == self._handedness:
                            # No device report, but the SN convention already
                            # identifies the side (Hand 2). The convention is
                            # firmware-assigned, so this is a verified match,
                            # not a guess — accept and pin it.
                            logger.info(
                                "Wuji hand sn={} does not report handedness; "
                                "accepting it as the {} hand per the SN "
                                "convention (4th char J=left/K=right).",
                                dev.sn, self._handedness,
                            )
                            _update_hand_map(self._handedness, dev.sn)
                            return hand
                        # The hand cannot say which side it is and its SN
                        # carries no side either. With a single hand attached
                        # there is nothing to confuse it with, so take it;
                        # with several, guessing risks driving the arm with
                        # the wrong hand — demand an explicit mapping.
                        if len(devices) == 1:
                            logger.warning(
                                "Wuji hand sn={} reports handedness {!r}; accepting it "
                                "as the {} hand because it is the only one attached.",
                                dev.sn, reported, self._handedness,
                            )
                            return hand
                        self._manager.disconnect_all()
                        raise ConnectionError(
                            f"Wuji hand sn={dev.sn} reports handedness {reported!r} and "
                            f"{len(devices)} hands are attached, so the {self._handedness} "
                            f"hand cannot be identified. Record the mapping once with "
                            f"`python examples/wuji/identify_hands.py` (writes "
                            f"{_HAND_MAP_PATH}), or pass an explicit --wuji-sn."
                        )
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
                # Drain to the NEWEST diagnostics frame — recv() returns queued
                # frames one at a time, and judging enable state off a stale
                # frame can produce a spurious timeout.
                frame = None
                while True:
                    f = diag_sub.recv()
                    if f is None:
                        break
                    frame = f
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
        """Wire up the model's tactile stream.

        Only called when WUJI_TACTILE=on. Any failure — missing glove, absent
        fingertip sensors, unsupported format — RAISES so the server refuses
        to start: the operator asked for tactile data, so a session that
        silently records none of it must not come up.
        """
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
                    raise RuntimeError(
                        f"WUJI_TACTILE=on but finger '{name}' reports an "
                        f"unsupported format: {fmt.get('encoding')!r}"
                    )
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
            if not attached:
                raise RuntimeError(
                    "WUJI_TACTILE=on but no tactile glove is attached "
                    "(plug it in before connecting, or set WUJI_TACTILE=off)"
                )
            self._glove_frame_sub = self._hand.tactile.subscribe_pressure_frame()

    def _wait_first_tactile_frame(self, timeout: float = 3.0) -> None:
        """Block until the tactile cache holds a real frame, or raise.

        Guarantees that when tactile is enabled, hand_tactile is present from
        the very first observation — the recorder's hand columns must be
        complete in every row.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.get_hand_tactile() is not None:
                return
            time.sleep(0.05)
        raise RuntimeError(
            f"WUJI_TACTILE=on but no tactile frame arrived within {timeout}s"
        )

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

            # Drain queue — only keep the newest command. The drain loop always
            # ends by raising Empty, so it must NOT clobber a target that was
            # already dequeued (same pattern as RobotiqGripper._worker_loop).
            target = None
            try:
                target = self._command_queue.get(timeout=period)
                while True:
                    target = self._command_queue.get_nowait()
            except Empty:
                pass
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

            # Watchdog on the joint_states stream (anchored at construction, so
            # it also fires when the hand never produces a single frame).
            stale = (time.monotonic() - self._last_state_time) > 1.0
            if stale and self._connected:
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
                # briefly offline joint keeps its last value. The nid base is
                # undocumented — detect once from the first frame (M1 verifies
                # on hardware).
                if self._nid_offset is None and latest.joints:
                    nids = [e.nid for e in latest.joints]
                    if min(nids) >= 1 and max(nids) == _N_JOINTS:
                        self._nid_offset = 1
                        logger.warning(
                            "Wuji Hand 2 ({}): joint nids look 1-based; "
                            "applying -1 offset.", self._handedness,
                        )
                    else:
                        self._nid_offset = 0
                offset = self._nid_offset or 0
                with self._state_lock:
                    for entry in latest.joints:
                        idx = entry.nid - offset
                        if 0 <= idx < _N_JOINTS:
                            self._cached_pos[idx] = entry.position
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
        """Detected hand model: 'wuji_hand_v1' (gen 1) or 'wuji_hand_v2' (gen 2)."""
        return "wuji_hand_v2" if self._is_hand2 else "wuji_hand_v1"

    @property
    def reported_handedness(self) -> str | None:
        """Side this hand reports for itself: 'left' | 'right' | None.

        Decodes the device handedness SDO (firmware encoding 0=Right / 1=Left).
        None when the firmware answers "Unknown" or does not expose it — the
        case identify_hands.py falls back to asking the operator about.
        """
        name = getattr(self._hand, "handedness_name", lambda: None)()
        if name is None:
            return None
        side = str(name).lower()
        return side if side in ("left", "right") else None

    @property
    def has_tactile(self) -> bool:
        return bool(self._tactile_subs or self._glove_frame_sub)

    @property
    def is_connected(self) -> bool:
        """False while the joint_states stream has been silent >1s."""
        return self._connected

    @property
    def serial_number(self) -> str:
        sn = getattr(self._hand, "serial_number", "unknown")
        if callable(sn):
            try:
                sn = sn()
            except Exception:
                sn = "unknown"
        return str(sn)

    def _teardown_channels(self) -> None:
        """Best-effort close of every subscription/command channel."""
        for sub in self._tactile_subs.values():
            try:
                sub.close()
            except Exception:
                pass
        self._tactile_subs.clear()
        if self._glove_frame_sub is not None:
            try:
                self._glove_frame_sub.close()
            except Exception:
                pass
            self._glove_frame_sub = None
        if self._state_sub is not None:
            try:
                self._state_sub.close()
            except Exception:
                pass
            self._state_sub = None
        try:
            if self._publisher is not None:
                self._publisher.close()
        except Exception:
            pass
        self._publisher = None
        try:
            if self._rt_ctx is not None:
                self._rt_ctx.__exit__(None, None, None)
        except Exception:
            pass
        self._rt_ctx = None
        self._rt = None

    def shutdown(self) -> None:
        """Stop the worker, close channels, and disable the motors."""
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        self._teardown_channels()
        try:
            self._hand.disable()
        except Exception:
            pass
        try:
            self._manager.disconnect_all()
        except Exception:
            pass
        logger.info("Wuji hand ({}) shut down.", self._handedness)
