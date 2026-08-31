import json

from fairino_fr5_vr.cli import synthetic_frame
from fairino_fr5_vr.runtime import Pipeline, replay
from fairino_fr5_vr.protocol import HandFrame


def _config(mode: str):
    return {
        "quest": {"mapping_mode": mode, "right_handed_position_sign": [1, -1, 1],
                  "left_handed_position_sign": [-1, -1, 1]},
        "arm": {"translation_scale_mm_per_m": [500, 500, 500],
                "axis_matrix": [[1, 0, 0], [0, 0, 1], [0, -1, 0]],
                "max_translation_step_mm": 4,
                "workspace_mm": {"x": [-700, 700], "y": [-700, 700], "z": [100, 1000]}},
        "hand": {"channel_max_steps": [1700, 1700, 1700, 1700, 1700, 1300], "max_step_delta_per_frame": 81},
    }


def _side(frame: HandFrame, side: str) -> HandFrame:
    return HandFrame(side, frame.wrist_position, frame.wrist_quaternion_xyzw, frame.landmarks, frame.timestamp_s)


def test_record_and_replay(tmp_path):
    cfg = _config("right")
    log = tmp_path / "session.jsonl"
    writer = Pipeline(cfg, record=log)
    writer.process(synthetic_frame(0.0, 1.0))
    assert json.loads(log.read_text())["type"] == "frame"
    assert replay(log, Pipeline(cfg)) == 1


def test_left_and_bimanual_routes_are_explicit():
    frame = synthetic_frame(0.0, 1.0)
    assert Pipeline(_config("left")).process(_side(frame, "left")) is not None
    bimanual = Pipeline(_config("both"))
    # In bimanual mode, the right wrist establishes FR5 control while the left
    # landmarks establish IH01 control; neither partial frame emits a command.
    assert bimanual.process(_side(frame, "right")) is None
    target = bimanual.process(_side(frame, "left"))
    assert target is not None and target["side"] == "both"
