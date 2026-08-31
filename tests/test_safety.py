from fairino_fr5_vr.safety import SafetyGate
def test_gate_requires_both_gates():
    g=SafetyGate(0.3); assert not g.arm(confirmed=False,anchored=True); assert not g.arm(confirmed=True,anchored=False); assert g.arm(confirmed=True,anchored=True)
def test_timeout_disarms():
    g=SafetyGate(0.3); g.observe(1.0); g.arm(confirmed=True,anchored=True); assert not g.check(1.31); assert g.reason=="tracking timeout"
