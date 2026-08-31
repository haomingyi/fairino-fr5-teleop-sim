# Operations and commissioning

## Hardware-free gate

Run `make verify`, `make demo`, then replay a recorded Quest session. Confirm
hand selection, coordinate directions, rate, dropouts, workspace and all six
IH01 channel directions before connecting hardware.

Run `make sim-check` for headless combined-model validation. Run `make arm-sim` for
the scripted viewer, or `make arm-teleop SIDE=right` after ADB reverse to drive
the simulated arm and attached right hand from Quest. The simulator uses
translation-only IK; orientation remains deliberately disabled.

## Quest gate

For wired TCP run `adb reverse tcp:8000 tcp:8000`, start `make listen`, then
stream. A pass requires complete wrist+landmark frames; ADB authorization or an
open TCP socket alone is not a pass.

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
