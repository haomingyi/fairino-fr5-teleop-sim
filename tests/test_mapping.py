from fairino_fr5_vr.arm_mapping import WristArmMapper
from fairino_fr5_vr.hand_mapping import IH01Mapper
from fairino_fr5_vr.protocol import HandFrame
def frame(x=0.0):
    points=tuple((0.01*(i%4),0.02*(i//4),0.003*i) for i in range(21))
    return HandFrame("right",(x,0.0,0.0),(0,0,0,1),points,1.0)
def test_arm_mapping_limits_step_and_workspace():
    m=WristArmMapper({"quest_sign":[1,-1,1],"translation_scale_mm_per_m":[1000]*3,"axis_matrix":[[1,0,0],[0,1,0],[0,0,1]],"max_translation_step_mm":4,"workspace_mm":{"x":[-10,10],"y":[-10,10],"z":[0,20]}})
    m.anchor(frame(),(0,0,10,0,0,0)); target=m.map(frame(1.0)); assert target.pose_mm_deg[0]==4
    for _ in range(4): target=m.map(frame(1.0))
    assert target.pose_mm_deg[0]==10 and target.clamped
def test_hand_steps_stay_in_ranges():
    m=IH01Mapper([1700]*5+[1300],81); first=m.map(frame()); second=m.map(frame())
    assert all(0<=v<=limit for v,limit in zip(second.steps,[1700]*5+[1300]))
    assert all(abs(a-b)<=81 for a,b in zip(first.steps,second.steps))
