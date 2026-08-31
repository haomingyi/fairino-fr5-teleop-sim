from ih01_runtime.hardware_control import ConsoleState, render_dashboard


def test_contact_feedback_is_visible_to_console():
    state = ConsoleState()
    state.update({
        "format": "ih01_dual_hand_control_v1",
        "state": "OP",
        "wkc": 3,
        "expected_wkc": 3,
        "cycle": 10,
        "hands": [{
            "side": "right",
            "slave": 1,
            "command_active": True,
            "target_steps": [100] * 6,
            "position_steps": [90] * 6,
            "current_ma": [0] * 6,
            "force_raw": [0] * 6,
            "temperature_c": [30] * 6,
            "fault_code": [0] * 6,
            "status_word": 0,
            "contact_hold": [False, False, False, True, False, False],
            "grasp_hold": True,
            "fault_latched": False,
        }],
    })
    hand = state.hands["right"]
    assert hand.contact_hold[3]
    assert hand.grasp_hold
    assert render_dashboard(state).shape == (900, 1600, 3)
