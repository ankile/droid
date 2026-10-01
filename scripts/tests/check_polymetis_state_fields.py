"""Check FrankaRobot.get_robot_state() against stock and patched Polymetis RobotStates.

Runs without a robot: polymetis, grpc, the IK solver and the station parameters are stubbed, and
RobotState is a fake object with or without the patched fields (ee_wrench, inertial parameters).
Needs numpy, scipy, torch and protobuf (all present on the NUC).

    python scripts/tests/check_polymetis_state_fields.py
"""

import sys
import types

import numpy as np
import torch
from google.protobuf.timestamp_pb2 import Timestamp


def _stub_module(name, **attrs):
    module = types.ModuleType(name)
    for key, value in attrs.items():
        setattr(module, key, value)
    sys.modules[name] = module


_stub_module("grpc")
_stub_module("polymetis", GripperInterface=object, RobotInterface=object)
_stub_module("droid.misc.parameters", sudo_password="")
_stub_module("droid.robot_ik.robot_ik_solver", RobotIKSolver=object)

from droid.franka.polymetis_state import PATCHED_POLYMETIS_FIELDS, patched_polymetis_fields  # noqa: E402
from droid.franka.robot import FrankaRobot  # noqa: E402

STOCK_KEYS = [
    "cartesian_position",
    "cartesian_velocity",
    "gripper_position",
    "joint_positions",
    "joint_velocities",
    "joint_torques_computed",
    "prev_joint_torques_computed",
    "prev_joint_torques_computed_safened",
    "motor_torques_measured",
    "motor_torques_external",
]
TAIL_KEYS = ["prev_controller_latency_ms", "prev_command_successful"]
PATCHED_VALUES = {
    "ee_wrench": (1.0, -2.0, 3.0, -0.4, 0.5, -0.6),
    "m_ee": 0.73,
    "f_x_cee": (0.0, 0.0, 0.1),
    "m_load": 0.0,
    "f_x_cload": (0.0, 0.0, 0.0),
    "m_total": 0.73,
    "f_x_ctotal": (0.0, 0.0, 0.1),
}


def fake_robot_state(patched):
    state = types.SimpleNamespace(
        joint_positions=(0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.8),
        joint_velocities=(0.01,) * 7,
        joint_torques_computed=(0.1,) * 7,
        prev_joint_torques_computed=(0.2,) * 7,
        prev_joint_torques_computed_safened=(0.3,) * 7,
        motor_torques_measured=(0.4,) * 7,
        motor_torques_external=(0.5,) * 7,
        prev_controller_latency_ms=1.25,
        prev_command_successful=True,
        timestamp=types.SimpleNamespace(seconds=12, nanos=34),
    )
    if patched:
        for name, value in PATCHED_VALUES.items():
            setattr(state, name, value)
    return state


class FakeRobotModel:
    def forward_kinematics(self, joint_positions):
        return torch.tensor([0.4, 0.0, 0.3]), torch.tensor([0.0, 0.0, 0.0, 1.0])

    def compute_jacobian(self, joint_positions):
        return torch.ones(6, 7)


class FakeRobotInterface:
    def __init__(self, robot_state):
        self.robot_model = FakeRobotModel()
        self._robot_state = robot_state

    def get_robot_state(self):
        return self._robot_state


def make_robot(robot_state):
    robot = FrankaRobot.__new__(FrankaRobot)
    robot._robot = FakeRobotInterface(robot_state)
    robot._gripper = types.SimpleNamespace(get_state=lambda: types.SimpleNamespace(width=0.02))
    robot._max_gripper_width = 0.08
    return robot


def check_stock():
    state_dict, timestamp_dict = make_robot(fake_robot_state(patched=False)).get_robot_state()
    assert list(state_dict) == STOCK_KEYS + TAIL_KEYS, list(state_dict)
    assert timestamp_dict == {"robot_timestamp_seconds": 12, "robot_timestamp_nanos": 34}
    print("stock RobotState: ok (patched keys omitted)")


def check_patched():
    state_dict, _ = make_robot(fake_robot_state(patched=True)).get_robot_state()
    patched_keys = [name for name, _ in PATCHED_POLYMETIS_FIELDS]
    # Same keys, order and values as the unguarded code.
    assert list(state_dict) == STOCK_KEYS + patched_keys + TAIL_KEYS, list(state_dict)
    for name, is_repeated in PATCHED_POLYMETIS_FIELDS:
        expected = list(PATCHED_VALUES[name]) if is_repeated else PATCHED_VALUES[name]
        assert state_dict[name] == expected, (name, state_dict[name])
        assert isinstance(state_dict[name], list) == is_repeated, name
    assert np.allclose(state_dict["cartesian_velocity"], [0.07] * 6)
    print("patched RobotState: ok (all patched keys reported)")


def check_partial():
    # A Polymetis with only the wrench patch (no inertial parameters).
    state = fake_robot_state(patched=False)
    state.ee_wrench = PATCHED_VALUES["ee_wrench"]
    assert patched_polymetis_fields(state) == [("ee_wrench", list(PATCHED_VALUES["ee_wrench"]))]
    print("wrench-only RobotState: ok")


def check_protobuf_getattr():
    # Polymetis' RobotState is a protobuf message; an unknown field must read as absent.
    assert patched_polymetis_fields(Timestamp(seconds=1)) == []
    print("protobuf message without the fields: ok")


if __name__ == "__main__":
    check_stock()
    check_patched()
    check_partial()
    check_protobuf_getattr()
    print("all checks passed")
