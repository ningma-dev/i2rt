# YAM arms in an xpisode station

[xpisode](https://github.com/ningma-dev/xpisode) runs data collection and policy evaluation
on real robots. Devices join it through its device protocol, each in its own process;
`i2rt.xpisode` makes YAM arms such devices without reimplementing anything:

| | i2rt does | the xpisode adapter adds |
| --- | --- | --- |
| Connect | `get_yam_robot`: motor checks, wrap-around offsets, gripper calibration, control thread | nothing; the calibrated gripper limits and the rest of `get_robot_info()` go into each Episode |
| Follow | `command_joint_pos`: PD and gravity compensation at the control thread's rate, joint and gripper clipping, gripper force limit | targets arrive per station tick; the follower state is published at `poll_hz` (200 Hz) |
| Stop | | a lease that is not renewed (the station died, the controller starved) makes the arm hold where it is |
| Park | `move_joints` | `park` moves the joints back to the pose they had when connected; the gripper stays |
| Close | `close` | parks first: without power the arms have no brakes and fall |
| Leader | teaching handle on the same bus: joints, trigger, buttons | published as the stream `leader/input` |

## Install

The arms run in this checkout's environment, the station in xpisode's:

```bash
uv sync --extra xpisode
```

```bash
git clone -b v2 git@github.com:ningma-dev/xpisode.git ../xpisode && (cd ../xpisode && uv sync)
```

The `xpisode` extra pins xpisode's packages to one commit (`[tool.uv.sources]` in
`pyproject.toml`). Its repository is private, so it needs GitHub SSH access.

## Without hardware

```bash
cd ../xpisode && uv run xpisode run ../i2rt/examples/xpisode/sim_station.toml
```

The follower is i2rt's MuJoCo `SimRobot`; the leader is xpisode's fake handle, which sways
and presses SYNC after a second, so the follower aligns and follows on its own. In another
terminal (also from `../xpisode`): `uv run xpisode status --station yam_sim`,
`uv run xpisode record start --station yam_sim --task demo`, `... record stop ...`,
`... shutdown ...`. Episodes land in `examples/xpisode/episodes/`.

## On the arms

`yam_station.toml` expects the follower (with a `linear_4310` gripper) on `can1` and the
leader (teaching handle) on `can0`. Check both arms rest on the table and nothing is in
reach, then:

```bash
cd ../xpisode && uv run xpisode run ../i2rt/examples/xpisode/yam_station.toml
```

What happens, in order:

1. **Connecting moves the follower's gripper**: i2rt calibrates it by driving it to both
   stops (the wrist may twitch). The follower then holds its pose; the leader floats in
   gravity compensation.
2. Bring the leader near the follower's pose and press **button 0 (SYNC)**: the follower
   moves to the leader over 3 s, then follows it, the trigger driving the gripper. SYNC
   again: the follower holds where it is.
3. **Button 1** starts and stops a recording, as do `uv run xpisode record start --station yam`
   and `record stop`.
4. **Ctrl+C** (once) shuts down: the teleop client stops, the recording is sealed, then both
   arms move back to where they were when connected (3 s each) and are released. **Let go
   of the leader**: it moves too. A second Ctrl+C only reports that shutdown is under way.

If anything misbehaves: release the leader, then Ctrl+C; the follower holds within 200 ms of
the station losing control (its lease) and parks on shutdown. Pulling power drops the arms.

Watch i2rt's own log lines: `Grav Comp Control Frequency` should stay well above 100 Hz
(it warns below), and the motor chain reports steps longer than its expected period.
