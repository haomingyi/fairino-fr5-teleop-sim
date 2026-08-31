"""Output safety state machine."""
from dataclasses import dataclass

@dataclass
class SafetyGate:
    timeout_s: float
    armed: bool = False
    last_frame_s: float | None = None
    reason: str = "DISARMED"
    def observe(self, timestamp_s: float) -> None: self.last_frame_s = float(timestamp_s)
    def arm(self, *, confirmed: bool, anchored: bool) -> bool:
        if not confirmed: self.reason = "confirmation missing"; return False
        if not anchored: self.reason = "fresh wrist anchor missing"; return False
        self.armed, self.reason = True, "ARMED"; return True
    def check(self, now_s: float) -> bool:
        if self.armed and (self.last_frame_s is None or now_s - self.last_frame_s > self.timeout_s): self.disarm("tracking timeout")
        return self.armed
    def disarm(self, reason: str) -> None: self.armed, self.reason = False, reason
