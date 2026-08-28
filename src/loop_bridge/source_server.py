"""Expose one bimanual Vega as a Loop Robot Node.

This is the Node-Graph migration of the previous ``LoopRobotClient`` bridge. It
keeps the device-side behavior in this repository: one shared physical Vega,
two per-arm RobotEnv services, one pre-action observation snapshot, and the
same per-arm ``Step`` calls.

- ``_LockedStepService`` is the upstream ``VegaRobotEnvService`` plus one fix: it
  serializes ``Step`` on the upstream ``_cmd_lock`` (upstream guards only ``Reset``),
  so the bus action lane can't race a Reset on shared IK/filter state.
- ``VegaRobotNode`` publishes one typed bimanual observation, receives one typed
  bimanual action, and dispatches each arm's slice to that arm's Step.

A bimanual robot is ONE ``Robot`` exposing both arms; two per-arm services share it
(``VegaRobot``/service take an injected ``robot``), reusing every per-arm
gain/frame/interpolation/IK/gripper path verbatim.
"""

from __future__ import annotations

import contextlib
import logging
import signal
import threading
import time
from collections.abc import Mapping
from importlib.metadata import PackageNotFoundError, version
from typing import Any, Literal, Sequence, cast

from loop_sdk import (
    LifecycleCallbacks,
    LifecycleState,
    NodeConnectionConfig,
    ReceivedMessage,
    RobotCommand,
    RobotNode,
    TensorValue,
)
from pydantic import BaseModel, ConfigDict, Field

# Importing the upstream module runs its sys.path setup and binds the proto stubs.
from dexcontrol.core.robotenv_vega import server as _vega_server
from loop_bridge.contracts import (
    LEFT_ARM,
    RIGHT_ARM,
    ROBOT_ACTION_CONTRACT,
    ROBOT_OBSERVATION_CONTRACT,
    action_info_shape,
    float64_tensor,
    float64_values,
    is_action_info_scalar,
)
from loop_bridge.obs_publisher import merge_observations
from loop_bridge.robot_obs import observation_state

LOGGER = logging.getLogger("loop_bridge.vega")

DEFAULT_ACTION_SPACE = "target_cartesian_delta"

# The installed dexcontrol build we run against. Advertised on ``robot-obs`` so a
# recording pins the robot-server version it was captured with (mirrors the HW
# ``robot_firmware_version`` axis, for the software side). Resolved from the
# installed package metadata so a release automatically flows through — no
# separate manual bump. Falls back to a sentinel when the package is not
# installed (e.g. running from an editable checkout without a wheel resolve).
try:
    _ROBOT_SERVER_VERSION = f"dexcontrol-{version('dexcontrol')}"
except PackageNotFoundError:
    _ROBOT_SERVER_VERSION = "dexcontrol-unknown"
# Fallback observation rate when the action lane is idle. Action-paired samples
# retain the legacy pre-apply snapshot semantics.
DEFAULT_HEARTBEAT_HZ = 20.0


class VegaRobotNodeConfig(BaseModel):
    """Graph-stored counterpart of the legacy negotiated RobotConfig."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    action_space: Literal["target_cartesian_delta"] = DEFAULT_ACTION_SPACE
    gripper_action_space: Literal["", "position"] = ""
    heartbeat_frequency_hz: float = Field(default=DEFAULT_HEARTBEAT_HZ, gt=0)
    gripper_type: str = "default"
    finger_type: str = "default"
    robot_type: str = "vega_1"
    robot_firmware_version: str = "v0.0.0"
    robot_server_version: str = _ROBOT_SERVER_VERSION
    teleop_server_version: str = ""


class _BusStepContext:
    """Minimal gRPC servicer context for replaying Step in-process.

    The Vega ``Step`` happy path never touches the context; we still surface an
    ``abort`` (which it would only call on a hard error) as an exception so the
    action lane skips that tick rather than silently succeeding.
    """

    def set_code(self, code: Any) -> None:
        self.code = code

    def set_details(self, details: str) -> None:
        self.details = details

    def abort(self, code: Any, details: str) -> None:
        raise RuntimeError(f"Step aborted: {code} {details}")


def _encode_pre_action_state(
    obs: Mapping[str, Any],
) -> dict[str, Any]:
    """Encode a loop-side obs dict into ``StepRequest.pre_action_state`` proto values.

    This is the bridge boundary where loop's ``obs`` becomes the Vega server's
    ``pre_action_state``. Mirrors ``VegaRobotEnvService._to_proto_value`` — list /
    ndarray → ``FloatArray``; int / bool / float → ``float_value``; otherwise
    stringify. Skips keys whose value is ``None`` so an absent reading doesn't
    spill onto the wire as a zero.
    """
    pb2: Any = _vega_server.robotenv_pb2
    encoded: dict[str, Any] = {}
    for key, value in obs.items():
        if value is None:
            continue
        if isinstance(value, (list, tuple)):
            encoded[key] = pb2.Value(
                float_array=pb2.FloatArray(values=[float(v) for v in value])
            )
            continue
        if isinstance(value, bool):
            # bool is int in Python — must precede the int branch to encode correctly.
            encoded[key] = pb2.Value(float_value=float(value))
            continue
        if isinstance(value, (int, float)):
            encoded[key] = pb2.Value(float_value=float(value))
            continue
        # ndarray path — mirror the Vega server's fallback shape.
        try:
            encoded[key] = pb2.Value(
                float_array=pb2.FloatArray(values=[float(v) for v in value])
            )
        except TypeError:
            encoded[key] = pb2.Value(string_value=str(value))
    return encoded


def _decode_action_info(
    action_info: Mapping[str, Any],
) -> dict[str, Any]:
    """Decode ``StepResponse.action_info`` proto values into a plain dict.

    The server computes these (desired_velocity, delta_action, resolved cartesian,
    ...) from the action against the pre-apply state and returns them per Step; the
    reverse of ``_encode_pre_action_state``. Drops the ``state.*`` entries the server
    flattens in — they duplicate the obs snapshot loop already publishes for this tick.
    """
    decoded: dict[str, Any] = {}
    for key, value in action_info.items():
        if key.startswith("state."):
            continue
        kind = value.WhichOneof("kind")
        if kind == "float_array":
            decoded[key] = [float(v) for v in value.float_array.values]
            continue
        if kind == "float_value":
            decoded[key] = float(value.float_value)
            continue
        if kind == "int_value":
            decoded[key] = int(value.int_value)
            continue
        if kind == "string_value":
            decoded[key] = value.string_value
    return decoded


class _StepApplier:
    """Adapts one arm service's ``Step`` to the action consumer's ``step(...)`` seam."""

    def __init__(self, service: Any) -> None:
        self._service = service

    def step(
        self,
        action: list[float],
        action_space: str,
        gripper_action_space: str,
        *,
        pre_apply_obs: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Step one arm and return the server-computed action info for this tick.

        The returned dict holds the auxiliary values the server derived while
        applying the action (desired_velocity, delta_action, resolved cartesian, ...),
        keyed by their bare names; the caller namespaces them per arm before merging
        onto the published robot-obs. Empty if the server sent no action info.
        """
        pb2: Any = _vega_server.robotenv_pb2
        request = pb2.StepRequest(
            action=list(action),
            action_space=action_space,
            gripper_action_space=gripper_action_space,
        )
        # Forward the loop-sdk-captured pre-apply obs into the Step RPC so the
        # server dispatches against the same snapshot the recorder will publish
        # for this action. Skipped when the caller didn't pre-read (server falls
        # back to its own state read).
        if pre_apply_obs is not None:
            for key, value in _encode_pre_action_state(pre_apply_obs).items():
                request.pre_action_state[key].CopyFrom(value)
        response = self._service.Step(request, _BusStepContext())
        # Step swallows hardware faults into a non-SUCCESS StepResponse.status rather
        # than raising, so the action lane would otherwise treat a stalled command
        # (joint limit, IK failure, comms) as success. Surface it.
        status = getattr(response, "status", "") or ""
        if status and status != "SUCCESS":
            LOGGER.warning(
                "robot-action Step returned %s: %s",
                status,
                getattr(response, "message", ""),
            )
            raise RuntimeError(
                f"robot-action Step returned {status}: {getattr(response, 'message', '')}"
            )
        return _decode_action_info(response.action_info)

    def home(self) -> None:
        """Home this arm: ``Reset(mode="home")`` — moves it to its home pose.

        The operational counterpart to ``step``; surfaces a non-SUCCESS reset the
        same way (the bus would otherwise treat a stalled home as success).
        """
        pb2: Any = _vega_server.robotenv_pb2
        request = pb2.ResetRequest(mode="home", params={})
        response = self._service.Reset(request, _BusStepContext())
        status = getattr(response, "status", "") or ""
        if status and status != "SUCCESS":
            raise RuntimeError(
                f"home Reset returned {status}: {getattr(response, 'message', '')}"
            )


class _LockedStepService(_vega_server.VegaRobotEnvService):
    """``VegaRobotEnvService`` whose ``Step`` is serialized on the upstream ``_cmd_lock``.

    Upstream takes ``_cmd_lock`` only in ``Reset`` — safe when Step had a single
    caller. The bus action lane is a second concurrent Step source, so without this
    an action-lane Step can race a Reset (or another Step) on shared IK / filter
    state and command a real arm a corrupted pose. ``Step`` never takes the lock
    itself, so this cannot self-deadlock.
    """

    def Step(self, request, context):
        with self._cmd_lock:
            return super().Step(request, context)


class VegaRobotNode(RobotNode[VegaRobotNodeConfig]):
    """One typed Robot Node backed by the existing two-arm RobotEnv services."""

    config_type = VegaRobotNodeConfig

    def __init__(self, arm_services: Sequence[tuple[str, Any]]) -> None:
        services = tuple(arm_services)
        arms = tuple(arm for arm, _service in services)
        if arms != (LEFT_ARM, RIGHT_ARM):
            raise ValueError(
                f"VegaRobotNode requires ordered left/right arm services, received {arms}"
            )
        self._arm_services = services
        self._appliers = {
            arm: _StepApplier(service) for arm, service in self._arm_services
        }
        self._device_lock = threading.RLock()
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread: threading.Thread | None = None
        self._heartbeat_hz = DEFAULT_HEARTBEAT_HZ
        self._action_space = DEFAULT_ACTION_SPACE
        self._gripper_action_space = ""
        self._last_publish_ns = 0
        self._active = False
        super().__init__(
            config_type=VegaRobotNodeConfig,
            observation_contract=ROBOT_OBSERVATION_CONTRACT,
            action_contract=ROBOT_ACTION_CONTRACT,
            action_handler=self._apply_action,
            command_handler=self._handle_command,
            input_capacity=32,
            lifecycle=LifecycleCallbacks(
                start=self._on_start,
                stop=self._on_stop,
                reset_fault=self._on_reset_fault,
            ),
        )

    def _on_start(self, config: VegaRobotNodeConfig) -> None:
        with self._device_lock:
            self._action_space = config.action_space
            self._gripper_action_space = config.gripper_action_space
            self._heartbeat_hz = config.heartbeat_frequency_hz
            self._heartbeat_stop.clear()
            self._active = True
            self._read_and_publish()
        thread = threading.Thread(
            target=self._heartbeat_loop,
            name="vega-robot-observation-heartbeat",
            daemon=True,
        )
        self._heartbeat_thread = thread
        thread.start()

    def _on_stop(self) -> None:
        with self._device_lock:
            self._active = False
            self._heartbeat_stop.set()
        thread = self._heartbeat_thread
        if thread is not None:
            thread.join()
        self._heartbeat_thread = None

    def _on_reset_fault(self) -> None:
        self._on_stop()

    def _heartbeat_loop(self) -> None:
        period_s = 1.0 / self._heartbeat_hz
        grace_ns = round(2.0 * period_s * 1_000_000_000)
        while not self._heartbeat_stop.wait(period_s):
            with self._device_lock:
                if not self._active:
                    return
                idle_ns = time.monotonic_ns() - self._last_publish_ns
                if idle_ns < grace_ns:
                    continue
                try:
                    self._read_and_publish()
                except Exception:
                    LOGGER.exception(
                        "heartbeat robot observation publish failed; retrying next period"
                    )

    def _read_observations(self) -> tuple[dict[str, Mapping[str, Any]], dict[str, Any]]:
        observations: dict[str, Mapping[str, Any]] = {}
        for arm, service in self._arm_services:
            observation, _sample_timestamp_us = service._create_observation()
            observations[arm] = observation
        return observations, merge_observations(observations)

    def _read_and_publish(self) -> None:
        _observations, payload = self._read_observations()
        self._publish_observation(payload)

    def _publish_observation(self, payload: Mapping[str, Any]) -> None:
        # Match the former LoopRobotClient stamp: wall-clock capture time on the
        # wire, while heartbeat scheduling remains on the monotonic clock.
        self.publish_observation(payload, timestamp_ns=time.time_ns())
        self._last_publish_ns = time.monotonic_ns()

    def _apply_action(self, message: ReceivedMessage) -> None:
        """Preserve the legacy read-before-Step and paired-publish ordering."""

        with self._device_lock:
            if not self._active:
                return
            observations, payload = self._read_observations()
            actions = _decode_bimanual_action(message)
            flat_action = (*actions[LEFT_ARM], *actions[RIGHT_ARM])
            payload["received_action"] = float64_tensor(flat_action, shape=(14,))
            for arm, applier in self._appliers.items():
                try:
                    action_info = applier.step(
                        actions[arm],
                        self._action_space,
                        self._gripper_action_space,
                        pre_apply_obs=observation_state(observations[arm]),
                    )
                except Exception as error:
                    LOGGER.warning(
                        "action Step failed for %s; skipping: %s",
                        arm,
                        error,
                    )
                    continue
                payload.update(_action_info_payload(arm, action_info))
            self._publish_observation(payload)

    def _handle_command(self, command: RobotCommand) -> None:
        if command is not RobotCommand.HOME:
            raise ValueError(f"unsupported robot command: {command.value!r}")
        with self._device_lock:
            if not self._active:
                raise RuntimeError("Vega Robot Node is not active")
            _observations, payload = self._read_observations()
            for arm, applier in self._appliers.items():
                try:
                    applier.home()
                except Exception as error:
                    LOGGER.warning("home failed for %s; skipping: %s", arm, error)
            self._publish_observation(payload)


def _decode_bimanual_action(message: ReceivedMessage) -> dict[str, list[float]]:
    actions: dict[str, list[float]] = {}
    for arm in (LEFT_ARM, RIGHT_ARM):
        field = f"{arm}.target_cartesian_delta"
        delta = float64_values(
            cast(TensorValue, message.payload[field]),
            field_name=field,
            shape=(6,),
        )
        gripper_value = message.payload[f"{arm}.gripper_position"]
        if isinstance(gripper_value, bool) or not isinstance(
            gripper_value, (int, float)
        ):
            raise TypeError(f"{arm}.gripper_position must be a scalar")
        gripper = float(gripper_value)
        actions[arm] = [*delta, gripper]
    return actions


def _action_info_payload(arm: str, action_info: Mapping[str, Any]) -> dict[str, Any]:
    payload: dict[str, Any] = {}
    for field, value in action_info.items():
        shape = action_info_shape(field)
        if shape is not None:
            payload[f"{arm}.action.{field}"] = float64_tensor(value, shape=shape)
            continue
        if is_action_info_scalar(field):
            payload[f"{arm}.action.{field}"] = float(value)
    return payload


_SERIAL_GRIPPERS = ("robotiq", "sr_gripper")


def _dual_arm_comports(
    service_kwargs: dict[str, Any],
    left_robotiq_comport: str | None,
    right_robotiq_comport: str | None,
) -> tuple[str | None, str | None]:
    """Resolve each arm's gripper comport for dual-arm, rejecting the same-port footgun.

    Per-arm overrides win; both fall back to the shared ``robotiq_comport``. A serial
    gripper (robotiq/sr_gripper) is one physical device per port — two arms on the
    SAME port would corrupt comms, so that is rejected. Distinct ports are fine: each
    arm's VegaRobot opens its own gripper independent of the shared arm hardware.
    """
    base = service_kwargs.get("robotiq_comport")
    left = left_robotiq_comport or base
    right = right_robotiq_comport or base
    gripper = service_kwargs.get("gripper_type", "default")
    if gripper == "default":
        gripper = service_kwargs.get("hand_type", "default")
    if gripper in _SERIAL_GRIPPERS and left == right:
        raise ValueError(
            f"dual-arm with a serial gripper ({gripper!r}) needs a DISTINCT comport per arm; "
            f"both arms resolved to {left!r}. Pass --robotiq-comport-left / --robotiq-comport-right."
        )
    return left, right


def serve_dual_arm(
    *,
    node_id: str,
    connection: NodeConnectionConfig,
    left_robotiq_comport: str | None = None,
    right_robotiq_comport: str | None = None,
    **service_kwargs: Any,
) -> None:
    """Run one external bimanual Robot Node over one shared physical Vega.

    Builds the left service (which constructs the one ``Robot`` with both arms), then
    a right service that SHARES that Robot (injected), so both arms run over one
    hardware connection. Each service keeps its own per-arm IK/filter/gripper.

    A serial gripper (robotiq/sr_gripper) is a separate device per arm — each arm's
    ``VegaRobot`` opens its OWN gripper on its OWN comport, independent of the shared
    arm hardware — so bimanual serial grippers work as long as each arm gets a
    DISTINCT comport (``left_robotiq_comport`` / ``right_robotiq_comport``). The same
    comport on both arms is the real footgun and is rejected.
    """
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s"
    )
    left_comport, right_comport = _dual_arm_comports(
        service_kwargs, left_robotiq_comport, right_robotiq_comport
    )
    left = _LockedStepService(
        arm_side="left", **{**service_kwargs, "robotiq_comport": left_comport}
    )
    shared_robot = left._robot.robot  # the one hardware unit (both arms) left built
    right = _LockedStepService(
        arm_side="right",
        robot=shared_robot,
        **{**service_kwargs, "robotiq_comport": right_comport},
    )

    node = VegaRobotNode([(LEFT_ARM, left), (RIGHT_ARM, right)])
    node.start(node_id=node_id, connection=connection)
    LOGGER.info(
        "Vega dual-arm Robot Node connected: node_id=%s arms=%s",
        node_id,
        [LEFT_ARM, RIGHT_ARM],
    )

    stop_requested = threading.Event()

    def request_stop(_signum: int, _frame: Any) -> None:
        stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while node.is_running and not stop_requested.is_set():
            node.process_control(timeout_s=0.1)
    finally:
        LOGGER.info("Shutting down Vega dual-arm Robot Node")
        # Stop Node data callbacks before releasing the shared hardware.
        with contextlib.suppress(Exception):
            if node.status.lifecycle is not LifecycleState.FINALIZED:
                node.shutdown()
        with contextlib.suppress(Exception):
            node.close()
        for service in (left, right):
            with contextlib.suppress(Exception):
                service._stop_control_loop()
        for service in (left, right):
            with contextlib.suppress(Exception):
                service._robot.close()
