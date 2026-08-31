from collections import deque

from hts_ih01.protocol import HandStreamAssembler, parse_hts_line
from hts_ih01.retarget import InitialRetargeter, _finger_flex, _looks_like_fist, _thumb_flex, _thumb_opposition


def test_parse_and_assemble_left_hand() -> None:
    assembler = HandStreamAssembler()
    wrist = "Left wrist:," + ",".join(str(value) for value in (1, 2, 3, 0, 0, 0, 1))
    landmarks = "Left landmarks:," + ",".join(str(index / 100) for index in range(63))
    assert assembler.feed(wrist) is None
    frame = assembler.feed(landmarks, now=10.0)
    assert frame is not None
    assert frame.side == "left"
    assert len(frame.landmarks) == 21
    assert frame.timestamp_s == 10.0


def test_invalid_landmark_count_is_rejected() -> None:
    try:
        parse_hts_line("Right landmarks:,1,2,3")
    except ValueError as exc:
        assert "needs 63 floats" in str(exc)
    else:
        raise AssertionError("short landmark packet was accepted")


def test_retarget_result_stays_in_channel_ranges() -> None:
    assembler = HandStreamAssembler()
    assembler.feed("Left wrist:,0,0,0,0,0,0,1")
    points = []
    for index in range(21):
        points.extend((index * 0.01, (index % 4) * 0.01, 0.0))
    frame = assembler.feed("Left landmarks:," + ",".join(map(str, points)))
    assert frame is not None
    result = InitialRetargeter().map(frame)
    assert all(0 <= value <= maximum for value, maximum in zip(result.steps, (1700,) * 5 + (1300,), strict=True))


def test_fixed_finger_geometry_reaches_both_endpoints() -> None:
    open_points = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
                   (2.0, 0.0, 0.0), (3.0, 0.0, 0.0))
    closed_points = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0),
                     (1.0, 1.0, 0.0), (0.0, 1.0, 0.0))
    assert _finger_flex(open_points, (0, 1, 2, 3)) == 0.0
    assert _finger_flex(closed_points, (0, 1, 2, 3)) == 1.0


def test_thumb_palm_closure_can_reach_full_flex() -> None:
    points = [(0.0, 0.0, 0.0)] * 21
    points[1:5] = [(-0.8, -0.5, 0.0), (-0.4, -0.5, 0.0),
                   (-0.2, -0.2, 0.0), (0.0, 0.0, 0.0)]
    points[5] = (-0.5, 0.0, 0.0)
    points[9] = (-0.15, 0.0, 0.0)
    points[13] = (0.15, 0.0, 0.0)
    points[17] = (0.5, 0.0, 0.0)
    assert _thumb_flex(tuple(points), 1.0) == 1.0


def test_ok_pose_uses_thumb_tip_to_index_tip_distance() -> None:
    points = [(0.0, 0.0, 0.0)] * 21
    points[4] = (0.0, 0.0, 0.0)   # thumb tip
    points[5] = (-0.5, 0.0, 0.0)  # index MCP
    points[8] = (0.0, 0.0, 0.0)   # index tip touching thumb tip
    points[17] = (0.5, 0.0, 0.0)
    assert _thumb_opposition(tuple(points), 1.0) == 1.0


def test_fist_override_does_not_depend_on_thumb_tracking() -> None:
    assert _looks_like_fist((0.8, 0.7, 0.68, 0.4, 0.05, 0.1))
    assert not _looks_like_fist((0.8, 0.1, 0.1, 0.1, 0.8, 0.8))


def test_single_frame_tracking_spike_is_rejected() -> None:
    retargeter = InitialRetargeter()
    open_pose = (0.1,) * 6
    spike = (0.1, 0.1, 0.1, 1.0, 0.1, 0.1)
    assert retargeter._stabilize("right", open_pose, 1.00) == open_pose
    assert retargeter._stabilize("right", open_pose, 1.01) == open_pose
    assert retargeter._stabilize("right", spike, 1.02)[3] == 0.1


def test_personal_open_fist_calibration_is_saved_and_applied(tmp_path) -> None:
    path = tmp_path / "quest_hand_personal.json"
    retargeter = InitialRetargeter(path)
    retargeter._recent_raw["right"] = deque([(0.2,) * 6] * 10, maxlen=20)
    assert retargeter.capture_pose("open", ("right",)) == "captured open: right"
    retargeter._recent_raw["right"] = deque([(0.8,) * 6] * 10, maxlen=20)
    assert retargeter.capture_pose("fist", ("right",)) == "captured fist: right=6/6 calibrated channels"

    loaded = InitialRetargeter(path)
    assert loaded.calibration_status("right") == "calibration R:6/6"
    calibrated = loaded._apply_calibration("right", (0.5,) * 6)
    assert all(abs(value - 0.5) < 1e-9 for value in calibrated)
