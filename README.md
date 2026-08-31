# FAIRINO FR5 + Quest 3 + IH01 Teleoperation

[English](README.md) | [中文](README_CN.md)

A portable, safety-gated project for controlling one FAIRINO FR5 arm and one
IH01 dexterous hand from Meta Quest 3 hand tracking. Quest wrist motion drives
the arm; the 21 OpenXR hand landmarks drive the six active IH01 channels.

The default workflow is simulation-only and keeps FR5 real output locked. The
separate `IPE-quest-hand-teleop/` project retains IH01 EtherCAT checks and its
explicit arming workflow.

## Quick start (no hardware)

```bash
cd /home/hzm/yyy/fairino-fr5-vr
make setup       # first time only
make arm-sim     # open FR5 + IH01 simulation
```

Once the Quest app is installed, USB-authorized, and connected, run only:

```bash
make arm-teleop
```

It checks the Quest, sets up ADB reverse, starts the IPE Quest Hand Teleop app,
and opens the combined simulation. Select **TCP Wired / localhost / 8000** in
the headset. Right-wrist translation drives FR5 numerical IK and landmarks
drive the attached IH01-X1-R; orientation awaits measured registration.

The `make ui` entry screen offers right-hand (right wrist + fingers), left-hand
(left wrist + fingers), and bimanual (right wrist -> FR5, left fingers -> IH01)
routes. Advanced selection is also available as `make sim-teleop SIDE=left` or
`SIDE=both`.

## Project layout

```text
src/fairino_fr5_vr/    protocol, mappings, safety gate, runtime and adapters
config/teleop.yaml     portable Quest/FR5/IH01 mapping and safety settings
docs/                  architecture, calibration, operation and safety gates
tests/                 hardware-free protocol/mapping/safety tests
scripts/               retained FR5 teach UI and diagnostic tools
third_party/           FAIRINO Python SDK with upstream license
IPE-quest-hand-teleop/ retained Quest/IH01 integration snapshot
```

## Commands

```bash
make help
make setup         # install dependencies once
make check         # hardware-free checks
make arm-sim       # manual simulation only
make arm-teleop    # Quest + combined simulation
make hand-teleop   # Quest + IH01 simulation, E enables physical output
make ui            # graphical launcher and live telemetry
make hand-control  # retained IH01 hardware manual-control dashboard
```

The standalone viewer starts in manual mode: `q/a`, `w/s`, `e/d`, `r/f`, `t/g`,
`y/h` jog J1-J6 respectively, `0` resets the arm,
`o/c` open/close the hand, and Space pauses/resumes. The terminal prints joint
angles and IH01 position every 0.5 s. The UI can start/stop simulation, start Quest-linked simulation, show mapped
Quest target, simulated IH01 position, Cartesian error and IK error. Its
manual-hand button asks again for confirmation and then delegates to the
preserved IH01 console.

For hand teleoperation, use `make hand-teleop`; it opens the IH01 simulation
mirror, prompts for the hand, and keeps the physical output disarmed until `E`
is pressed. `make hand-control` remains the separate manual-control command.

For Quest-to-physical-IH01 teleoperation, use `make hand-teleop`. It prompts
for the connected hand and keeps output disarmed until `E` is pressed.

### Installing on a new Quest 3

`make install` installs the APK on the headset. `make reverse` only forwards
TCP `localhost:8000`; it does not install anything. Enable Developer Mode in
the Meta Horizon mobile app, connect and unlock the headset, then accept the
USB-debugging prompt. From `IPE-quest-hand-teleop/` run:

```bash
make status      # must show an authorized device
make install     # installs hand_tracking_streamer.apk
make reverse     # sets up the TCP reverse tunnel
```

`Success`/`PASS: installed ... APK` confirms installation; `PASS: Quest
localhost:8000 -> PC localhost:8000` confirms the tunnel. Repeat these steps
when replacing the headset; rebuilding the project is not required.

If you modify the Unity project, rebuild first with `make unity-build` (Unity
6 required), then run `make install` and `make reverse` again.

The `make hand-teleop` window keeps its HAND MODE control and `l`/`r`/`b`
shortcuts for switching the simulated hand view. Pressing `E` requests physical
IH01 output; if no hand is connected, the UI reports the error and remains in
simulation mode.

Read [docs/OPERATIONS.md](docs/OPERATIONS.md) before hardware use. The
integrated runtime intentionally defaults to dry-run. The FR5 SDK adapter is a
commissioning interface, not an enabled default. IH01 hardware remains in the
reviewed, independently armed runtime documented in
`IPE-quest-hand-teleop/README.md`.

## Validation boundary

- Supported now: parsing, frame assembly, wrist anchoring, Cartesian clamping
  and slew limiting, IH01 geometry mapping, timeout gate, record/replay, and
  deterministic dry-run verification. The combined MuJoCo model has 6 FR5
  joints, the attached IH01-X1-R joint tree, 12 actuators, and numerical
  translation IK for Quest simulation.
- The FR5 appearance uses only FAIRINO's official V6 link meshes in white; no
  protruding custom joint covers are added, and the official meshes are used for collision.
- Present but not commissioned here: FAIRINO SDK output adapter.
- Separate hardware path: IH01 EtherCAT runtime, which starts disarmed and
  requires OP/WKC/fault checks plus an operator `E` action.
- Not claimed: collision avoidance, certified functional safety, calibrated
  Quest-to-robot registration, or a completed physical teleoperation trial.

See [docs/PROVENANCE.md](docs/PROVENANCE.md) for upstream and license boundaries.
The FR5 code/config audit is recorded in [docs/FR5_AUDIT_CN.md](docs/FR5_AUDIT_CN.md).
