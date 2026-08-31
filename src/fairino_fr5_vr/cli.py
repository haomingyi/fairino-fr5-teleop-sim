"""Command line interface for the portable teleoperation project."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import socket
import yaml
from .protocol import HandFrame
from .runtime import Pipeline, listen_tcp, replay

ROOT = Path(__file__).resolve().parents[2]
def load_config(path: Path, side: str | None=None) -> dict:
    cfg=yaml.safe_load(path.read_text(encoding="utf-8"))
    if side: cfg["quest"]["mapping_mode"]=side
    return cfg
def synthetic_frame(offset: float, stamp: float) -> HandFrame:
    points=tuple((0.01*(i%4),0.02*(i//4),0.002*i) for i in range(21))
    return HandFrame("right",(0.2+offset,1.0,0.3),(0.0,0.0,0.0,1.0),points,stamp)
def main(argv: list[str] | None=None) -> int:
    parser=argparse.ArgumentParser(prog="fr5-vr"); sub=parser.add_subparsers(dest="command",required=True)
    for name in ("demo","listen","replay","probe-fr5"):
        item=sub.add_parser(name); item.add_argument("--config",type=Path,default=ROOT/"config"/"teleop.yaml")
        if name=="listen":
            item.add_argument("--host",default=None); item.add_argument("--port",type=int,default=None)
            item.add_argument("--side",choices=("left","right","both"),default=None); item.add_argument("--record",type=Path)
        elif name=="replay": item.add_argument("log",type=Path); item.add_argument("--real-time",action="store_true")
        elif name=="probe-fr5": item.add_argument("--timeout",type=float,default=1.0)
    args=parser.parse_args(argv); cfg=load_config(args.config,getattr(args,"side",None))
    if args.command=="demo":
        pipeline=Pipeline(cfg)
        for i in range(4): print(json.dumps(pipeline.process(synthetic_frame(i*0.01,i*0.02)),separators=(",",":")))
        print("PASS demo output=dry-run frames=4"); return 0
    if args.command=="listen":
        listen_tcp(args.host or cfg["quest"]["tcp_host"],args.port or int(cfg["quest"]["tcp_port"]),Pipeline(cfg,record=args.record)); return 0
    if args.command=="replay":
        count=replay(args.log,Pipeline(cfg),args.real_time); print(f"PASS replay frames={count}"); return 0 if count else 1
    if args.command=="probe-fr5":
        ip=str(cfg["fr5"]["ip"]); result={}
        for port in (20003,20005,8080,8083):
            try:
                with socket.create_connection((ip,port),timeout=args.timeout): result[port]=True
            except OSError: result[port]=False
        print(json.dumps({"ip":ip,"tcp_ports":result,"motion":False})); return 0 if result[20003] else 1
    return 2
if __name__=="__main__": raise SystemExit(main())
