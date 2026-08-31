"""Combined dry-run pipeline and JSONL record/replay."""
from __future__ import annotations
import json
from pathlib import Path
import socket
import time
from .arm_mapping import WristArmMapper
from .hand_mapping import IH01Mapper
from .protocol import HandFrame, HandStreamAssembler

class Pipeline:
    def __init__(self, cfg: dict, *, robot_anchor=(400.0,0.0,400.0,180.0,0.0,0.0), record: Path | None=None) -> None:
        quest = cfg["quest"]
        self.mapping_mode = str(quest.get("mapping_mode", quest.get("side", "right"))).lower()
        routes = {"right": ("right", "right"), "left": ("left", "left"), "both": ("right", "left")}
        if self.mapping_mode not in routes:
            raise ValueError("Quest mapping mode must be right, left, or both")
        self.arm_side, self.hand_side = routes[self.mapping_mode]
        arm_cfg = dict(cfg["arm"])
        arm_cfg["quest_sign"] = quest[f"{self.arm_side}_handed_position_sign"]
        self.arm = WristArmMapper(arm_cfg); hand = cfg["hand"]
        self.hand = IH01Mapper(hand["channel_max_steps"], hand["max_step_delta_per_frame"])
        self.robot_anchor, self.record_path, self._anchored = tuple(robot_anchor), record, False
        self._last_arm = None
        self._last_hand = None
    def process(self, frame: HandFrame) -> dict | None:
        if frame.side == self.arm_side:
            if not self._anchored: self.arm.anchor(frame, self.robot_anchor); self._anchored = True
            self._last_arm = self.arm.map(frame)
        if frame.side == self.hand_side:
            self._last_hand = self.hand.map(frame)
        if self._last_arm is None or self._last_hand is None:
            return None
        result = {"type":"targets","timestamp_s":frame.timestamp_s,"side":self.mapping_mode,
                  "arm_pose_mm_deg":list(self._last_arm.pose_mm_deg),"arm_clamped":bool(self._last_arm.clamped),
                  "ih01_steps":list(self._last_hand.steps),"ih01_quality":self._last_hand.quality}
        if self.record_path is not None:
            self.record_path.parent.mkdir(parents=True, exist_ok=True)
            with self.record_path.open("a",encoding="utf-8") as stream: stream.write(json.dumps({"type":"frame","frame":frame.to_dict()},separators=(",",":"))+"\n")
        return result

def listen_tcp(host: str, port: int, pipeline: Pipeline) -> None:
    assembler = HandStreamAssembler()
    with socket.socket(socket.AF_INET,socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1); server.bind((host,port)); server.listen(1)
        print(f"LISTEN tcp://{host}:{port} output=dry-run",flush=True); conn,address=server.accept(); print(f"CONNECTED source={address}",flush=True)
        with conn:
            buffer=""
            while True:
                data=conn.recv(65536)
                if not data: return
                buffer += data.decode("utf-8"); lines=buffer.split("\n"); buffer=lines.pop()
                for line in lines:
                    frame=assembler.feed(line)
                    if frame is not None:
                        result=pipeline.process(frame)
                        if result: print(json.dumps(result,separators=(",",":")),flush=True)

def replay(path: Path, pipeline: Pipeline, real_time: bool=False) -> int:
    count, previous = 0, None
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            item=json.loads(line)
            if item.get("type") != "frame": continue
            frame=HandFrame.from_dict(item["frame"])
            if real_time and previous is not None: time.sleep(max(0.0,min(0.25,frame.timestamp_s-previous)))
            previous=frame.timestamp_s; result=pipeline.process(frame)
            if result: print(json.dumps(result,separators=(",",":"))); count += 1
    return count
