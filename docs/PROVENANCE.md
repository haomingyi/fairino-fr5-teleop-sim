# Provenance and boundaries

- `third_party/fairino-python-sdk/`: FAIRINO Python SDK snapshot with its
  upstream license retained beside it.
- `IPE-quest-hand-teleop/`: portable Quest/IH01 snapshot. Its Quest app derives
  from Hand Tracking Streamer v1.1.0 commit `5ff7c1c` under Apache-2.0; see its
  `UPSTREAM.md` and `LICENSE`.
- The six-channel geometry in `hand_mapping.py` was reorganized from the
  retained IH01 adapter for this integration layer.
- `scripts/`: pre-existing FAIRINO teach, soft-zero, frame and joint tools.
- FR5 kinematic origins, joint limits, masses and inertias were checked against
  FAIR-INNOVATION's official `frcobot_ros2/fairino_description/urdf/fairino5_v6.urdf`.
- FR5 V6 visual meshes under `simulation/meshes/fr5_v6/` are copied from the
  same official repository; their material is recolored blue locally. The
  simulation keeps simplified analytic collision geoms for performance.
- Product-level payload, reach, repeatability and axis limits were checked
  against FAIRINO's official FR5 product page.

Large APKs, Unity caches, binaries, virtual environments and logs are local or
generated artifacts, not required for the new Python integration tests.
