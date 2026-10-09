# Architecture

```text
Quest 3 / HTS (TCP CSV)
          |
  protocol + frame assembler
          |
          +--> wrist anchor / axis map / slew / workspace --> FR5 adapter
          +--> 21-point geometry / quality / slew ----------> IH01 adapter
          +--> JSONL recorder --> deterministic replay
```

Transport, mapping and output layers are separate. A tracker can emit
`HandFrame`; another hand model can emit six bounded channels; simulators or
ROS 2 nodes can replace hardware adapters.

The first selected-hand frame creates a relative wrist anchor. A Quest world
origin is never treated as a robot base pose. The example axis matrix and scale
in `config/teleop.yaml` must be calibrated before physical use.

The anchored target is the mounted IH01 palm-center TCP. `CombinedSimulation`
reports that same point through `tcp_pose_mm_deg()`; the FR5 six-joint IK then
solves for this Cartesian target rather than copying J2/J3/J4/J5/J6 from the
operator. The flange-to-hand transform in `config/teleop.yaml` is therefore a
mechanical mounting parameter and must be measured when the hand bracket changes.

`scripts/` contains the FR5 manual-control baseline and integrated launchers.
`IPE-quest-hand-teleop/` is a bundled project component supplying the Quest
app, MuJoCo hand model and independently armed IH01 EtherCAT path. Runtime
commands resolve it relative to this repository root; no sibling checkout is
part of the execution chain.

The repository boundary is intentional: reference projects outside
`fairino-fr5-teleop-sim/` may be inspected for comparison, but code, models, build
scripts and configuration needed at runtime must be copied or implemented
inside this tree. `make check` runs `scripts/check_portability.py` to reject
machine-specific paths, external symlinks and missing bundled dependencies.

`simulation/fr5_ih01_right.xml` is generated from the official FR5 joint
origins/limits and the retained IH01-X1-R mesh/joint tree. Arm link visuals use
official FR5 mesh collision and IH01 collision geometry; adjacent mating links
and internal hand pairs are excluded to avoid false contacts at structural
interfaces, while arm/hand/prop contacts remain active. A collidable floor and
two dynamic grasp props are included. It remains a simulation/control
integration model, not a certified safety model.

During dynamic teleoperation the simulated FR5 joint servos compensate the
six arm bias forces, matching an enabled robot's gravity-hold behavior. The
hand, contacts, floor and grasp props remain dynamic; this compensation is a
simulation behavior and is not a real-robot safety or payload configuration.
