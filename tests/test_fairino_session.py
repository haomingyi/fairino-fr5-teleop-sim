from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from fairino_session import SAFETY_STATE_UNAVAILABLE, patch_degraded_robot, prepare_motion

class Proxy:
    def GetRobotErrorCode(self): return [0,0,0]
    def GetRobotEmergencyStopState(self): return [0,0]
    def GetSafetyStopState(self): return [0,0,0]
class Robot:
    def __init__(self,proxy=None): self.robot=proxy or Proxy(); self.enable_calls=[]
    def Mode(self,_state): return 0
    def SetSpeed(self,_speed): return 0
    def RobotEnable(self,state): self.enable_calls.append(state); return 0

def test_degraded_safety_uses_controller_state():
    robot=Robot(); patch_degraded_robot(robot); assert robot.GetSafetyCode()==0
def test_degraded_safety_fails_closed_when_rpc_missing():
    robot=Robot(object()); patch_degraded_robot(robot); assert robot.GetSafetyCode()==SAFETY_STATE_UNAVAILABLE
    result=prepare_motion(robot,enable=True); assert result["enable"]==SAFETY_STATE_UNAVAILABLE; assert robot.enable_calls==[]
