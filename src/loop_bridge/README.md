# Dual-arm Vega Robot Node

This package migrates the former Loop Source Bus bridge on
`origin/feat/loop-source-robot-obs` to the Node Graph SDK. It presents one physical
Vega with two arms as one external `loop.robot@1` Node.

## Run

```bash
uv sync --extra loop --extra dev

uv run dexcontrol-loop-robot-node \
  --node-id robot \
  --loop-endpoint tcp/127.0.0.1:7448 \
  --gripper-type robotiq \
  --robotiq-comport-left /dev/ttyUSB0 \
  --robotiq-comport-right /dev/ttyUSB1
```

The process can start before Loop. Registration keeps retrying until the Loop
Orchestrator appears. The `--node-id` must match the external Robot Node ID in the
Cell Config.

Hardware, IK, interpolation, filtering, gains, and gripper settings remain CLI
arguments owned by this robot process. Run `uv run dexcontrol-loop-robot-node
--help` for the complete set.

## Ports

- `robot_observation` output: both arms' Cartesian pose, gripper position, joint
  position/velocity/torque, and wrench.
- `action_command` input: `left/right.target_cartesian_delta[6]` plus absolute
  `left/right.gripper_position`.
- `robot_command` request server: `home` homes both arms through the existing
  RobotEnv `Reset(mode="home")` paths.

## Preserved behavior

For every action, the Node:

1. reads both arm observations once;
2. passes each arm's exact pre-action state to its existing RobotEnv `Step`;
3. applies the left and right 7-value action blocks through the unchanged services;
4. publishes that paired pre-action observation after dispatch, including the
   received 14-value action and the fixed action-info fields returned by `Step`.

Before the first action it publishes one bootstrap observation. While actions are
flowing, their paired observations determine cadence. After two quiet heartbeat
periods it resumes state-only observations at `heartbeat_frequency_hz` (20 Hz by
default), matching the former `LoopRobotClient.run()` behavior.

The physical Vega and its per-arm services stay open while this external process is
alive. A Graph Stop only stops Node data activity; a later Graph Start reuses the
same hardware process. Process shutdown closes both per-arm control loops and both
gripper/robot wrappers.

## Intentional wire changes

The old Source Bus carried an opaque 14-value action and fields such as
`robot0.observation.state.cartesian_position`. Node Graph contracts make these
explicit:

- `robot0` maps to `left`, `robot1` maps to `right`;
- the action is four named fields rather than an out-of-band vector layout;
- tensor dtype and shape are advertised by `Describe` and validated before a Graph
  starts;
- the Graph's Main Node builds and records the final ControlStep.

These are transport/data-contract changes. They do not alter the values passed into
Vega `Step`, its pre-action state, or the low-level control path.
