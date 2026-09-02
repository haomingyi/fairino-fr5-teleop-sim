#!/usr/bin/env python3
"""Generate a MuJoCo FR5 + flange-mounted IH01-X1-R model.

FR5 joint origins/limits follow FAIR-INNOVATION/frcobot_ros2
fairino_description/urdf/fairino5_v6.urdf. The arm uses detailed analytic
official white link meshes; the retained IH01 right-hand
mesh/joint tree is copied unchanged.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
SOURCE_HAND = ROOT / "IPE-quest-hand-teleop" / "simulation" / "ih01_x1_dual.xml"
OUTPUT = ROOT / "simulation" / "fr5_ih01_right.xml"
RESOLVED_CONFIG = ROOT / "simulation" / "teleop_resolved.json"
FR5_MESHES = ("base_link", "shoulder_link", "upperarm_link", "forearm_link",
              "wrist1_link", "wrist2_link", "wrist3_link")

LIMITS = (
    (-3.0543, 3.0543), (-4.6251, 1.4835), (-2.8274, 2.8274),
    (-4.6251, 1.4835), (-3.0543, 3.0543), (-3.0543, 3.0543),
)

def element(parent, tag: str, **attrs):
    return ET.SubElement(parent, tag, {key: str(value) for key, value in attrs.items()})

def add_arm(worldbody: ET.Element, mount_xyz: list[float], mount_rpy: list[float], right_hand: ET.Element) -> None:
    white = "0.88 0.90 0.94 1"
    base = element(worldbody,"body",name="base_link",pos="0 0 0")
    element(base,"geom",name="fr5_base_official",type="mesh",mesh="fr5_base_link",rgba=white,contype="1",conaffinity="1")
    shoulder = element(base,"body",name="shoulder_link")
    element(shoulder,"geom",name="fr5_shoulder_official",type="mesh",mesh="fr5_shoulder_link",rgba=white,contype="1",conaffinity="1")
    element(shoulder,"joint",name="j1",type="hinge",axis="0 0 1",range=f"{LIMITS[0][0]} {LIMITS[0][1]}",damping="2")
    upper = element(shoulder,"body",name="upperarm_link",pos="0 0 0.152",euler=f"{math.pi/2} 0 0")
    element(upper,"geom",name="fr5_upperarm_official",type="mesh",mesh="fr5_upperarm_link",rgba=white,contype="1",conaffinity="1")
    element(upper,"joint",name="j2",type="hinge",axis="0 0 1",range=f"{LIMITS[1][0]} {LIMITS[1][1]}",damping="2")
    forearm = element(upper,"body",name="forearm_link",pos="-0.425 0 0")
    element(forearm,"geom",name="fr5_forearm_official",type="mesh",mesh="fr5_forearm_link",rgba=white,contype="1",conaffinity="1")
    element(forearm,"joint",name="j3",type="hinge",axis="0 0 1",range=f"{LIMITS[2][0]} {LIMITS[2][1]}",damping="1.5")
    
    wrist1 = element(forearm,"body",name="wrist1_link",pos="-0.39501 0 0")
    element(wrist1,"geom",name="fr5_wrist1_official",type="mesh",mesh="fr5_wrist1_link",rgba=white,contype="1",conaffinity="1")
    element(wrist1,"joint",name="j4",type="hinge",axis="0 0 1",range=f"{LIMITS[3][0]} {LIMITS[3][1]}",damping="1")
    wrist2 = element(wrist1,"body",name="wrist2_link",pos="0 0 0.1021",euler=f"{math.pi/2} 0 0")
    element(wrist2,"geom",name="fr5_wrist2_official",type="mesh",mesh="fr5_wrist2_link",rgba=white,contype="1",conaffinity="1")
    element(wrist2,"joint",name="j5",type="hinge",axis="0 0 1",range=f"{LIMITS[4][0]} {LIMITS[4][1]}",damping="0.8")
    wrist3 = element(wrist2,"body",name="wrist3_link",pos="0 0 0.102",euler=f"{-math.pi/2} 0 0")
    element(wrist3,"geom",name="fr5_wrist3_official",type="mesh",mesh="fr5_wrist3_link",rgba=white,contype="1",conaffinity="1")
    element(wrist3,"joint",name="j6",type="hinge",axis="0 0 1",range=f"{LIMITS[5][0]} {LIMITS[5][1]}",damping="0.6")
    right_hand.attrib.pop("mocap",None)
    right_hand.set("pos"," ".join(str(float(v)) for v in mount_xyz))
    right_hand.set("euler"," ".join(str(math.radians(float(v))) for v in mount_rpy))
    wrist3.append(right_hand)

def generate(config_path: Path, output: Path, resolved_config: Path = RESOLVED_CONFIG) -> None:
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    ee = cfg["fr5"]["end_effector"]
    source = ET.parse(SOURCE_HAND).getroot()
    root = ET.Element("mujoco",{"model":"fr5_ih01_x1_right"})
    element(root,"compiler",angle="radian",autolimits="true",balanceinertia="true")
    element(root,"option",timestep="0.002",gravity="0 0 -9.81",integrator="implicitfast",
            iterations="100",noslip_iterations="10",cone="elliptic",impratio="10")
    visual = element(root,"visual"); element(visual,"global",offwidth="1280",offheight="720")
    asset = element(root,"asset")
    element(asset, "texture", name="floor_tex", type="2d", builtin="checker",
            width="512", height="512", rgb1="0.08 0.10 0.14", rgb2="0.16 0.19 0.24",
            mark="edge", markrgb="0.30 0.34 0.40")
    element(asset, "material", name="floor", texture="floor_tex", texrepeat="5 5",
            reflectance="0")
    for mesh_name in FR5_MESHES:
        element(asset, "mesh", name=f"fr5_{mesh_name}", file=f"meshes/fr5_v6/{mesh_name}.STL")
    for material in source.find("asset").findall("material"):
        if material.get("name") in {"shell_white","shell_blue","shell_blue_tip","joint_metal"}: asset.append(copy.deepcopy(material))
    for mesh in source.find("asset").findall("mesh"):
        if mesh.get("name","").startswith("rh_"):
            item=copy.deepcopy(mesh); item.set("file",item.get("file").replace("../simulation/meshes","../IPE-quest-hand-teleop/simulation/meshes")); asset.append(item)
    default=element(root,"default"); element(default,"joint",damping="0.12",armature="0.0002",frictionloss="0.002")
    world=element(root,"worldbody"); element(world,"light",pos="0 -1 1.4",dir="0 1 -1",diffuse="0.9 0.9 0.9")
    element(world,"geom",name="ground",type="plane",pos="0 0 -0.01",size="1.2 1.2 0.01",
            material="floor",contype="2",conaffinity="7",condim="3",friction="0.8 0.1 0.01")
    right=next(body for body in source.find("worldbody").findall("body") if body.get("name")=="rh_hand")
    right = copy.deepcopy(right)
    hand_body_names = [body.get("name") for body in right.iter("body") if body.get("name")]
    # The source hand reserves collision bit 2 for hand-only self-collision.
    # Use the arm collision bit as well so the palm/fingers collide with FR5.
    for geom in right.iter("geom"):
        if geom.get("type") == "mesh":
            # Keep vendor meshes visual-only. The simplified capsules and
            # explicit palm collision box provide stable contact surfaces;
            # mesh-vs-bottle contacts allowed deep back-of-hand penetration
            # and produced an unrealistic suction effect.
            geom.set("contype", "0")
            geom.set("conaffinity", "0")
            continue
        if geom.get("name", "").endswith("_collision"):
            geom.set("contype", "1")
            geom.set("conaffinity", "7")
        # Grasp contacts need enough tangential and torsional friction to hold
        # a bottle while the arm accelerates. condim=4 adds torsional friction.
        geom.set("condim", "4")
        geom.set("friction", "1.0 0.08 0.01")
        if geom.get("name") == "rh_palm_collision":
            # The palm is a support surface, not a suction pad. Let a bottle
            # resting on the back slide away unless the fingers pinch it.
            geom.set("condim", "3")
            geom.set("friction", "0.25 0.03 0.005")
    for joint in right.iter("joint"):
        if joint.get("actuatorfrcrange"):
            # The source model's joint-side +/-2 limit silently clipped the
            # stronger actuator range, so the previous grip-force change had
            # no effect. Keep both sides consistent.
            joint.set("actuatorfrcrange", "-6 6")
    add_arm(world,ee["flange_to_hand_sim_xyz_m"],ee["flange_to_hand_sim_rpy_deg"],right)
    # Palm-facing eye-in-hand view. Extend the camera above and slightly
    # forward of the end-effector, then aim it back/down at the palm and
    # fingertips; the earlier side-mounted pose was occluded by the hand back.
    # The camera is above the palm but looks toward the grasp scene (+X), not
    # back into the robot body. Its elevated position keeps the palm visible
    # while the bottle remains in front of the view.
    element(right, "camera", name="ih01_palm_camera", pos="-0.20 -0.10 0.0",
            # Camera -Z aims down and forward toward the grasp table.
            quat="-0.472202 -0.526332 0.234616 0.667050", fovy="84")
    # Three dynamic grasp-test props. They collide with the arm, hand and floor
    # and can be pushed/lifted by the IH01 fingers in the simulator.
    # At ARM_HOME the IH01 fingers point toward +X; place the grasp props on
    # that forward side so the scene matches the operator's view.
    def add_bottle(name, pos, body_rgba):
        bottle = element(world, "body", name=name, pos=pos)
        element(bottle, "freejoint", name=f"{name}_free")
        # 46 mm square body: narrow enough for a stable thumb/finger pinch.
        element(bottle, "geom", name=f"{name}_body_geom", type="box",
                pos="0 0 0.09", size="0.023 0.023 0.09", rgba=body_rgba,
                density="250", contype="4", conaffinity="7", condim="3",
                friction="2.5 0.18 0.03", solref="0.012 1", solimp="0.92 0.99 0.001")
        element(bottle, "geom", name=f"{name}_neck_geom", type="cylinder",
                pos="0 0 0.195", size="0.012 0.022", rgba="0.72 0.86 0.98 0.88",
                density="250", contype="4", conaffinity="7", condim="3",
                friction="2.5 0.18 0.03", solref="0.012 1", solimp="0.92 0.99 0.001")
        element(bottle, "geom", name=f"{name}_cap_geom", type="cylinder",
                pos="0 0 0.225", size="0.015 0.006", rgba="0.95 0.95 0.95 1",
                density="250", contype="4", conaffinity="7", condim="3",
                friction="2.5 0.18 0.03", solref="0.012 1", solimp="0.92 0.99 0.001")

    # Same X coordinate and separated Y coordinates keep the bottles parallel
    # and side-by-side in front of the robot.
    add_bottle("grasp_bottle", "0.72 -0.18 0.0", "0.20 0.52 0.86 0.82")
    add_bottle("grasp_bottle_green", "0.72 -0.06 0.0", "0.20 0.72 0.42 0.82")
    box = element(world, "body", name="grasp_box_yellow", pos="0.84 0.06 0.025", euler="0 0 0.25")
    element(box, "freejoint", name="grasp_box_yellow_free")
    element(box, "geom", name="grasp_box_yellow_geom", type="box", size="0.045 0.03 0.025",
            rgba="0.95 0.66 0.08 1", density="450", contype="4", conaffinity="7",
            condim="3", friction="0.8 0.1 0.01", solref="0.015 1", solimp="0.90 0.98 0.002")
    actuator=element(root,"actuator")
    for index,(lo,hi) in enumerate(LIMITS,1): element(actuator,"position",name=f"fr5_j{index}",joint=f"j{index}",kp="60",dampratio="1",ctrlrange=f"{lo} {hi}",forcerange="-150 150")
    for item in source.find("actuator"):
        if item.get("name","").startswith("rh_"):
            hand_actuator = copy.deepcopy(item)
            # The physical hand can maintain a firm pinch.  The original
            # 2 N proxy force was insufficient to lift even the light bottle.
            if hand_actuator.get("forcerange"):
                hand_actuator.set("forcerange", "-6 6")
            actuator.append(hand_actuator)
    equality=element(root,"equality")
    for item in source.find("equality"):
        if item.get("name","").startswith("rh_"): equality.append(copy.deepcopy(item))
    contact = element(root, "contact")
    # The official meshes overlap at their mating faces by design. Exclude
    # only adjacent structural pairs; non-adjacent arm links and the hand
    # collision proxies remain active for self-collision response.
    for first, second in (("base_link", "shoulder_link"),
                          ("shoulder_link", "upperarm_link"),
                          ("upperarm_link", "forearm_link"),
                          ("forearm_link", "wrist1_link"),
                          ("wrist1_link", "wrist2_link"),
                          ("wrist2_link", "wrist3_link")):
        element(contact, "exclude", body1=first, body2=second)
    # The hand base is bolted to the flange. Exclude only that mating pair;
    # fingers and palm still collide with every other arm link and with props.
    element(contact, "exclude", body1="wrist3_link", body2="rh_hand")
    # Vendor visual meshes are used as accurate external collision surfaces.
    # Exclude only internal hand-vs-hand pairs to avoid false self contacts at
    # the authored folded thumb/palm geometry; hand-vs-arm and hand-vs-prop
    # contacts remain enabled.
    for index, first in enumerate(hand_body_names):
        for second in hand_body_names[index + 1:]:
            element(contact, "exclude", body1=first, body2=second)
    output.parent.mkdir(parents=True,exist_ok=True)
    ET.indent(root,space="  "); ET.ElementTree(root).write(output,encoding="utf-8",xml_declaration=True)
    resolved_config.write_text(json.dumps(cfg,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print(f"PASS generated {output}")

def main() -> int:
    parser=argparse.ArgumentParser(); parser.add_argument("--config",type=Path,default=ROOT/"config"/"teleop.yaml"); parser.add_argument("--output",type=Path,default=OUTPUT)
    args=parser.parse_args(); generate(args.config,args.output); return 0
if __name__=="__main__": raise SystemExit(main())
