from pathlib import Path
from xml.etree import ElementTree as ET

MODEL=Path(__file__).resolve().parents[1]/"simulation"/"fr5_ih01_right.xml"

def test_combined_model_has_arm_and_attached_right_hand():
    root=ET.parse(MODEL).getroot(); world=root.find("worldbody")
    assert world.find(".//body[@name='rh_hand']") is not None
    assert world.find(".//body[@name='rh_hand']").get("mocap") is None
    for index in range(1,7): assert world.find(f".//joint[@name='j{index}']") is not None
    names={item.get("name") for item in root.find("actuator")}
    assert {f"fr5_j{i}" for i in range(1,7)} <= names
    assert "rh_thumb_flex" in names and "rh_index_flex" in names


def test_arm_visual_is_white_without_orange_or_blue_overlays():
    root = ET.parse(MODEL).getroot()
    colors = {item.get("rgba") for item in root.findall(".//geom")}
    assert "0.88 0.90 0.94 1" in colors
    assert "0.04 0.29 0.72 1" not in colors
    assert "0.92 0.42 0.04 1" not in colors


def test_official_fr5_visual_meshes_are_loaded():
    root = ET.parse(MODEL).getroot()
    meshes = {item.get("name") for item in root.find("asset").findall("mesh")}
    assert {f"fr5_{name}" for name in ("base_link", "shoulder_link", "upperarm_link",
                                        "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link")} <= meshes


def test_fr5_uses_official_meshes_without_duplicate_base_primitives():
    root = ET.parse(MODEL).getroot()
    arm_geoms = [item for item in root.findall('.//geom')
                 if (item.get('name') or '').startswith('fr5_')]
    assert {item.get('name') for item in arm_geoms if item.get('name', '').endswith('_official')} == {
        'fr5_base_official', 'fr5_shoulder_official', 'fr5_upperarm_official',
        'fr5_forearm_official', 'fr5_wrist1_official', 'fr5_wrist2_official',
        'fr5_wrist3_official'
    }
    assert all(item.get('type') == 'mesh' for item in arm_geoms if item.get('name', '').endswith('_official'))
    assert not any(item.get('name', '').endswith('_cover') for item in arm_geoms)


def test_attached_hand_collision_uses_arm_collision_group():
    root = ET.parse(MODEL).getroot()
    palm = root.find(".//geom[@name='rh_palm_collision']")
    assert palm is not None
    assert palm.get("contype") == "1"
    assert palm.get("conaffinity") == "7"
    visual_mesh = root.find(".//geom[@mesh='rh_base_link']")
    assert visual_mesh is not None and visual_mesh.get("contype") == "0"


def test_palm_camera_is_forward_and_present():
    root = ET.parse(MODEL).getroot()
    camera = root.find(".//camera[@name='ih01_palm_camera']")
    assert camera is not None
    assert camera.get("quat") is not None

def test_scene_has_collidable_ground_and_grasp_props():
    root = ET.parse(MODEL).getroot()
    ground = root.find(".//geom[@name='ground']")
    assert ground is not None and ground.get("contype") == "2" and ground.get("conaffinity") == "7"
    assert root.find(".//body[@name='grasp_bottle']") is not None
    assert root.find(".//body[@name='grasp_bottle_green']") is not None
    assert root.find(".//body[@name='grasp_box_yellow']") is not None
    for name in ("grasp_bottle_body_geom", "grasp_bottle_neck_geom", "grasp_bottle_cap_geom",
                 "grasp_bottle_green_body_geom", "grasp_bottle_green_neck_geom",
                 "grasp_bottle_green_cap_geom",
                 "grasp_box_yellow_geom"):
        geom = root.find(f".//geom[@name='{name}']")
        assert geom is not None and geom.get("contype") == "4" and geom.get("conaffinity") == "7"


def test_grasp_contact_and_force_limits_are_not_silently_clipped():
    root = ET.parse(MODEL).getroot()
    option = root.find("option")
    assert option.get("noslip_iterations") == "10"
    for name in ("r_index1_joint", "r_middle1_joint", "r_ring1_joint",
                 "r_little1_joint", "r_thumb1_joint", "r_thumb2_joint"):
        assert root.find(f".//joint[@name='{name}']").get("actuatorfrcrange") == "-6 6"
    for actuator in root.find("actuator"):
        if actuator.get("name", "").startswith("rh_"):
            assert actuator.get("forcerange") == "-6 6"
