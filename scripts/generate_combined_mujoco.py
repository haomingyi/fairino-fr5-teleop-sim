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
    element(root,"option",timestep="0.002",gravity="0 0 0",integrator="implicitfast")
    visual = element(root,"visual"); element(visual,"global",offwidth="1280",offheight="720")
    asset = element(root,"asset")
    for mesh_name in FR5_MESHES:
        element(asset, "mesh", name=f"fr5_{mesh_name}", file=f"meshes/fr5_v6/{mesh_name}.STL")
    for material in source.find("asset").findall("material"):
        if material.get("name") in {"shell_white","shell_blue","shell_blue_tip","joint_metal"}: asset.append(copy.deepcopy(material))
    for mesh in source.find("asset").findall("mesh"):
        if mesh.get("name","").startswith("rh_"):
            item=copy.deepcopy(mesh); item.set("file",item.get("file").replace("../simulation/meshes","../IPE-quest-hand-teleop/simulation/meshes")); asset.append(item)
    default=element(root,"default"); element(default,"joint",damping="0.12",armature="0.0002",frictionloss="0.002")
    world=element(root,"worldbody"); element(world,"light",pos="0 -1 1.4",dir="0 1 -1",diffuse="0.9 0.9 0.9")
    element(world,"geom",type="plane",size="1.2 1.2 0.01",rgba="0.08 0.10 0.13 1",contype="0",conaffinity="0")
    right=next(body for body in source.find("worldbody").findall("body") if body.get("name")=="rh_hand")
    right = copy.deepcopy(right)
    # The source hand reserves collision bit 2 for hand-only self-collision.
    # Use the arm collision bit as well so the palm/fingers collide with FR5.
    for geom in right.iter("geom"):
        if geom.get("name", "").endswith("_collision"):
            geom.set("contype", "1")
            geom.set("conaffinity", "1")
    add_arm(world,ee["flange_to_hand_sim_xyz_m"],ee["flange_to_hand_sim_rpy_deg"],right)
    actuator=element(root,"actuator")
    for index,(lo,hi) in enumerate(LIMITS,1): element(actuator,"position",name=f"fr5_j{index}",joint=f"j{index}",kp="120",ctrlrange=f"{lo} {hi}",forcerange="-150 150")
    for item in source.find("actuator"):
        if item.get("name","").startswith("rh_"): actuator.append(copy.deepcopy(item))
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
    output.parent.mkdir(parents=True,exist_ok=True)
    ET.indent(root,space="  "); ET.ElementTree(root).write(output,encoding="utf-8",xml_declaration=True)
    resolved_config.write_text(json.dumps(cfg,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    print(f"PASS generated {output}")

def main() -> int:
    parser=argparse.ArgumentParser(); parser.add_argument("--config",type=Path,default=ROOT/"config"/"teleop.yaml"); parser.add_argument("--output",type=Path,default=OUTPUT)
    args=parser.parse_args(); generate(args.config,args.output); return 0
if __name__=="__main__": raise SystemExit(main())
