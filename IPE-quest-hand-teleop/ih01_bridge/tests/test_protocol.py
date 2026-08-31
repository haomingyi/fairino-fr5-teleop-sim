from hts_ih01.protocol import HandStreamAssembler, parse_hts_line
from hts_ih01.retarget import InitialRetargeter


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
