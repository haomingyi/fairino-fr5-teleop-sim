import pytest
from fairino_fr5_vr.protocol import HandStreamAssembler, parse_hts_line
def test_assembler_builds_frame():
    a=HandStreamAssembler(); assert a.feed("Right wrist:,1,2,3,0,0,0,1",now=1.0) is None
    frame=a.feed("Right landmarks:,"+",".join(str(i/100) for i in range(63)),now=1.01)
    assert frame is not None and frame.side=="right" and len(frame.landmarks)==21
def test_stale_pair_is_dropped():
    a=HandStreamAssembler(pair_timeout_s=0.1); a.feed("Left wrist:,1,2,3,0,0,0,1",now=1.0)
    assert a.feed("Left landmarks:,"+",".join("0" for _ in range(63)),now=1.2) is None
def test_bad_length_rejected():
    with pytest.raises(ValueError): parse_hts_line("Right wrist:,1,2")
