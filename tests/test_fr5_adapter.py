from fairino_fr5_vr.fr5_adapter import validate_end_effector

def test_unmeasured_hand_mount_is_rejected():
    cfg={"tool_id":1,"end_effector":{"model":"IH01-X1-R","approved_for_motion":False,"mass_kg":None,"center_of_mass_mm":None,"flange_to_tcp_mm_deg":None}}
    assert len(validate_end_effector(cfg)) == 4

def test_measured_and_approved_hand_mount_passes():
    cfg={"tool_id":1,"end_effector":{"model":"IH01-X1-R","approved_for_motion":True,"mass_kg":1.2,"center_of_mass_mm":[0,0,70],"flange_to_tcp_mm_deg":[0,0,155,0,0,0]}}
    assert validate_end_effector(cfg) == []
