# Operations and commissioning

## Hardware-free gate

Run `make check`, then replay a recorded Quest session if available. Confirm
hand selection, coordinate directions, rate, dropouts, workspace and all six
IH01 channel directions before connecting hardware.

Run `make arm-sim` for the manual viewer, or `make arm-teleop` after ADB reverse to drive the
simulated arm and attached right hand from Quest. The simulator uses 6D pose IK.
Press `E` to start/pause/resume and Space to return to Ready; pause and resume always
re-anchor the current wrist and robot poses. Never use always-follow for
physical commissioning.

Use the panel's `Ready` button before a new trial or after an unexpected pose;
it disarms the panel, opens the simulated hand, and returns the simulation to
`ARM_HOME`. The wrist mapper forms targets from the current clutch anchor and
applies translation/rotation slew limits, so repeated Quest deltas cannot
accumulate into drift.

## Quest gate

For wired TCP run `cd IPE-quest-hand-teleop && make reverse`, start the Quest
application manually, and then launch `make arm-teleop`. A pass requires
complete wrist+landmark frames; ADB authorization or an open TCP socket alone
is not a pass.

`arm-teleop` does not launch `scrcpy`; use the headset view for operator
confirmation. This avoids treating protected immersive-compositor capture as a
teleoperation or TCP failure.

## FR5 read-only gate

Keep the arm disabled. Verify clearance, payload/TCP, E-stop, controller IP,
TCP 20003, pose, workspace and mapping direction. `make probe-fr5` only checks
reachability. The SDK adapter is not exposed as a casual Make target. Any
commissioning path must use `SafetyGate`, begin at measured TCP pose and call
`StopMotion` plus disable on timeout, error, Ctrl-C or tracking loss.

Physical motion additionally requires a measured combined hand+adapter mass,
flange-frame centre of mass and TCP. Until these values are filled and
`approved_for_motion` is explicitly set, `FR5Adapter` refuses to connect.

## IH01 gate

Follow `IPE-quest-hand-teleop/README.md`. Do not press `E` until EtherCAT is OP,
WKC is expected, feedback updates, faults are zero, the slave/side is correct
and the hand is unloaded. Tracking loss must produce STOP.

## Stop immediately

Use the physical E-stop if the arm jumps, moves opposite prediction, exceeds a
small step, reports error, loses tracking, or EtherCAT WKC/fault changes. Never
work around an interlock to continue.
