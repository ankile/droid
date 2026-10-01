"""Optional Polymetis RobotState fields.

The end-effector wrench (libfranka O_F_ext_hat_K) and the libfranka inertial parameters are not
part of stock Polymetis' RobotState; they exist only in a patched Polymetis (proto + server).
get_robot_state() reports each field when the RobotState has it and omits the key otherwise, so
the same code runs against stock and patched Polymetis.

Kept free of polymetis/torch imports so it can be checked without a robot stack.
"""

# (state key, is a repeated field), in the order get_robot_state() reports them.
PATCHED_POLYMETIS_FIELDS = (
    ("ee_wrench", True),
    ("m_ee", False),
    ("f_x_cee", True),
    ("m_load", False),
    ("f_x_cload", True),
    ("m_total", False),
    ("f_x_ctotal", True),
)


def patched_polymetis_fields(robot_state):
    """Return [(key, value)] for the patched-Polymetis fields present on robot_state."""
    fields = []
    for name, is_repeated in PATCHED_POLYMETIS_FIELDS:
        value = getattr(robot_state, name, None)
        if value is None:
            continue
        fields.append((name, list(value) if is_repeated else value))
    return fields
