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

`scripts/` is the existing FR5 manual-control baseline.
`IPE-quest-hand-teleop/` supplies the Quest app, MuJoCo model and independently
armed IH01 EtherCAT path. The new `src/` integration does not modify them.

`simulation/fr5_ih01_right.xml` is generated from the official FR5 joint
origins/limits and the retained IH01-X1-R mesh/joint tree. Arm link visuals use
official FR5 mesh collision and IH01 collision proxies; adjacent mating links
are excluded to avoid false contacts at structural interfaces. It remains a
simulation/control integration model, not a certified safety model.
