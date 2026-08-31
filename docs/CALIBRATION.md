# Calibration

1. Record a hardware-free Quest session from a repeatable neutral pose.
2. Move one Quest axis at a time; update `arm.axis_matrix` with `-1/0/1` first.
3. Measure a known displacement and update `translation_scale_mm_per_m`.
4. Keep orientation disabled until translation passes replay and low-speed
   tests. Quaternion registration needs a measured Quest-to-base rotation.
5. Make the workspace a conservative box inside the cleared physical region.
6. Validate IH01 open, half-close, close, pinch and thumb opposition. Include
   latency, jitter, pinch stability and occlusion recovery; RMSE alone is not a
   promotion criterion.
