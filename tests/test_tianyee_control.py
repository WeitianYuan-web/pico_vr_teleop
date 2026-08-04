from __future__ import annotations

import json
import math
import types
import unittest

import numpy as np

from backends.tianyee.udp_ros_bridge import TeleopHoldGuard, _tf_reply
from backends.tianyee.vr_teleop_dual import DualTianyiVrTeleop, SideState
from backends.tianyee.config import DEFAULT_HOME_OFFSET_XYZ
from backends.tianyee.ros_endpose import TianyiRosEndpose
from common.math_quat import quat_inverse_wxyz, quat_multiply_wxyz
from common.vr_input import rotation_enabled
from publisher.teleop_state_bridge import normalize_arm_joints


def _axis_angle(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    vec = np.asarray(axis, dtype=float)
    vec /= np.linalg.norm(vec)
    return np.array([math.cos(angle / 2.0), *(vec * math.sin(angle / 2.0))])


def _same_rotation(actual: np.ndarray, expected: np.ndarray) -> bool:
    return bool(abs(float(np.dot(actual, expected))) > 1.0 - 1e-6)


class _FakeRos:
    def __init__(self) -> None:
        self.actual = {
            "left": (np.array([0.31, 0.22, 0.53]), np.array([1.0, 0.0, 0.0, 0.0])),
            "right": (np.array([0.32, -0.22, 0.53]), np.array([1.0, 0.0, 0.0, 0.0])),
        }
        self.published: list[tuple[str, np.ndarray, np.ndarray]] = []

    def lookup_tcp(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        pos, quat = self.actual[side]
        return pos.copy(), quat.copy()

    def publish_pose(self, side: str, pos: np.ndarray, quat: np.ndarray) -> None:
        self.published.append((side, np.asarray(pos).copy(), np.asarray(quat).copy()))


class _FakeSocket:
    def __init__(self) -> None:
        self.payloads: list[bytes] = []

    def sendto(self, payload: bytes, _address: tuple[str, int]) -> None:
        self.payloads.append(payload)


class TianyeeControlTests(unittest.TestCase):
    def test_endpose_handoff_holds_one_frozen_tcp_snapshot(self) -> None:
        ros = object.__new__(TianyiRosEndpose)
        lookup_count = {"left": 0, "right": 0}
        published: list[tuple[str, np.ndarray]] = []

        def lookup(side: str) -> tuple[np.ndarray, np.ndarray]:
            lookup_count[side] += 1
            # A second lookup would simulate the arm already springing upward.
            z = 0.15 + 0.05 * (lookup_count[side] - 1)
            return np.array([0.5, 0.2 if side == "left" else -0.2, z]), np.array(
                [1.0, 0.0, 0.0, 0.0]
            )

        ros.lookup_tcp = lookup  # type: ignore[method-assign]
        ros.publish_pose = (  # type: ignore[method-assign]
            lambda side, pos, _quat: published.append((side, np.asarray(pos).copy()))
        )
        ros.spin_once = lambda _timeout=0.0: None  # type: ignore[method-assign]

        ros.hold_current_endpose(seconds=0.01, hz=100.0)

        self.assertEqual(lookup_count, {"left": 1, "right": 1})
        self.assertGreater(len(published), 2)
        self.assertTrue(all(abs(float(pos[2]) - 0.15) < 1e-9 for _, pos in published))

    def test_default_home_uses_fixed_tcp_not_startup_pose(self) -> None:
        from backends.tianyee.config import DEFAULT_HOME_XYZ_LEFT, DEFAULT_HOME_XYZ_RIGHT

        self.assertEqual(DEFAULT_HOME_OFFSET_XYZ, (0.0, 0.0, 0.0))
        self.assertEqual(DEFAULT_HOME_XYZ_LEFT, (0.35, 0.35, 0.08))
        self.assertEqual(DEFAULT_HOME_XYZ_RIGHT, (0.35, -0.35, 0.08))

        teleop = object.__new__(DualTianyiVrTeleop)
        teleop.args = types.SimpleNamespace(
            home_offset_xyz=DEFAULT_HOME_OFFSET_XYZ,
            home_rpy_offset_deg=(0.0, 0.0, 0.0),
            home_rpy_left_deg=None,
            home_rpy_right_deg=None,
            home_xyz_left=list(DEFAULT_HOME_XYZ_LEFT),
            home_xyz_right=list(DEFAULT_HOME_XYZ_RIGHT),
        )
        teleop.active_hands = ("left", "right")
        teleop.sides = {
            "left": SideState(
                side="left",
                name="left",
                hold_pos=np.array([0.31, 0.22, 0.10]),
                hold_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            ),
            "right": SideState(
                side="right",
                name="right",
                hold_pos=np.array([0.32, -0.22, 0.10]),
                hold_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            ),
        }

        teleop._set_home_targets(prefer_absolute=True)

        np.testing.assert_allclose(teleop.sides["left"].home_pos, DEFAULT_HOME_XYZ_LEFT)
        np.testing.assert_allclose(teleop.sides["right"].home_pos, DEFAULT_HOME_XYZ_RIGHT)

    def test_home_sequence_skips_motion_when_already_near_fixed_home(self) -> None:
        from backends.tianyee.config import (
            DEFAULT_HOME_RPY_DEG_LEFT,
            DEFAULT_HOME_RPY_DEG_RIGHT,
            DEFAULT_HOME_XYZ_LEFT,
            DEFAULT_HOME_XYZ_RIGHT,
        )
        from common.math_euler import euler_xyz_to_quat_wxyz

        left_q = euler_xyz_to_quat_wxyz(
            *[float(v) * np.pi / 180.0 for v in DEFAULT_HOME_RPY_DEG_LEFT]
        )
        right_q = euler_xyz_to_quat_wxyz(
            *[float(v) * np.pi / 180.0 for v in DEFAULT_HOME_RPY_DEG_RIGHT]
        )
        teleop = object.__new__(DualTianyiVrTeleop)
        teleop.args = types.SimpleNamespace(
            no_home=False,
            home_use_joints=False,
            home_offset_xyz=(0.0, 0.0, 0.0),
            home_rpy_offset_deg=(0.0, 0.0, 0.0),
            home_rpy_left_deg=list(DEFAULT_HOME_RPY_DEG_LEFT),
            home_rpy_right_deg=list(DEFAULT_HOME_RPY_DEG_RIGHT),
            home_xyz_left=list(DEFAULT_HOME_XYZ_LEFT),
            home_xyz_right=list(DEFAULT_HOME_XYZ_RIGHT),
            home_skip_tol_m=0.04,
            home_skip_tol_deg=12.0,
            release_freeze_s=0.05,
        )
        teleop.active_hands = ("left", "right")
        teleop.sides = {
            "left": SideState(
                side="left",
                name="left",
                hold_pos=np.array(DEFAULT_HOME_XYZ_LEFT, dtype=float),
                hold_quat_wxyz=left_q.copy(),
            ),
            "right": SideState(
                side="right",
                name="right",
                hold_pos=np.array(DEFAULT_HOME_XYZ_RIGHT, dtype=float),
                hold_quat_wxyz=right_q.copy(),
            ),
        }
        calls: list[str] = []
        teleop._go_home_joints = lambda: calls.append("joints") or True
        teleop._move_to_home = lambda **_k: calls.append("move")
        teleop._send_inactive_poses = lambda _poses: calls.append("freeze")

        teleop._apply_home_sequence(seeded_ok=True, label="启动初始化")

        self.assertNotIn("joints", calls)
        self.assertNotIn("move", calls)
        self.assertIn("freeze", calls)

    def test_home_sequence_moves_when_orientation_wrong(self) -> None:
        from backends.tianyee.config import (
            DEFAULT_HOME_RPY_DEG_LEFT,
            DEFAULT_HOME_RPY_DEG_RIGHT,
            DEFAULT_HOME_XYZ_LEFT,
            DEFAULT_HOME_XYZ_RIGHT,
        )

        teleop = object.__new__(DualTianyiVrTeleop)
        teleop.args = types.SimpleNamespace(
            no_home=False,
            home_use_joints=False,
            home_offset_xyz=(0.0, 0.0, 0.0),
            home_rpy_offset_deg=(0.0, 0.0, 0.0),
            home_rpy_left_deg=list(DEFAULT_HOME_RPY_DEG_LEFT),
            home_rpy_right_deg=list(DEFAULT_HOME_RPY_DEG_RIGHT),
            home_xyz_left=list(DEFAULT_HOME_XYZ_LEFT),
            home_xyz_right=list(DEFAULT_HOME_XYZ_RIGHT),
            home_skip_tol_m=0.04,
            home_skip_tol_deg=12.0,
            release_freeze_s=0.05,
        )
        teleop.active_hands = ("left", "right")
        identity = np.array([1.0, 0.0, 0.0, 0.0])
        teleop.sides = {
            "left": SideState(
                side="left",
                name="left",
                hold_pos=np.array(DEFAULT_HOME_XYZ_LEFT, dtype=float),
                hold_quat_wxyz=identity.copy(),
            ),
            "right": SideState(
                side="right",
                name="right",
                hold_pos=np.array(DEFAULT_HOME_XYZ_RIGHT, dtype=float),
                hold_quat_wxyz=identity.copy(),
            ),
        }
        calls: list[str] = []
        teleop._move_to_home = lambda **_k: calls.append("move")
        teleop._send_inactive_poses = lambda _poses: calls.append("freeze")

        teleop._apply_home_sequence(seeded_ok=True, label="启动初始化")

        self.assertEqual(calls, ["move"])

    def test_collection_joint_schema_is_always_seven_axes(self) -> None:
        self.assertEqual(
            normalize_arm_joints([1, 2, 3, 4, 5, 6]),
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0],
        )
        self.assertEqual(
            normalize_arm_joints([1, 2, 3, 4, 5, 6, 7]),
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
        )

    def test_tianyee_collection_reply_contains_real_seven_axis_state(self) -> None:
        class FakeStateRos:
            def arm_joint_names(self, side: str) -> list[str]:
                suffix = "l" if side == "left" else "r"
                return [f"joint_{i}_{suffix}" for i in range(7)]

            def arm_q_snapshot(self, side: str) -> list[float]:
                offset = 0.0 if side == "left" else 10.0
                return [offset + float(i) for i in range(7)]

            def arm_dq_snapshot(self, side: str) -> list[float]:
                offset = 0.0 if side == "left" else 1.0
                return [offset + 0.1 * float(i) for i in range(7)]

            def lookup_tcp(self, side: str) -> tuple[np.ndarray, np.ndarray]:
                y = 0.2 if side == "left" else -0.2
                return np.array([0.4, y, 0.5]), np.array([1.0, 0.0, 0.0, 0.0])

        reply = _tf_reply(FakeStateRos())  # type: ignore[arg-type]
        self.assertEqual(len(reply["left"]["joints"]), 7)
        self.assertEqual(len(reply["right"]["joints"]), 7)
        self.assertEqual(len(reply["left"]["joint_velocities"]), 7)
        side = DualTianyiVrTeleop._collection_side_from_reply(reply["left"])
        self.assertIsNotNone(side)
        assert side is not None
        self.assertTrue(side["arm_valid"])
        self.assertEqual(side["arm_joints"], [float(i) for i in range(7)])
        self.assertEqual(side["arm_velocities"], [0.1 * float(i) for i in range(7)])

    def test_external_motion_drops_old_hold_and_reseeds_measured_tcp(self) -> None:
        ros = _FakeRos()
        guard = TeleopHoldGuard(ros, watchdog_timeout_s=0.15)
        quat = np.array([1.0, 0.0, 0.0, 0.0])
        guard.accept_pose("left", active=True, xyz=np.array([0.8, 0.2, 0.7]), quat=quat)
        guard.accept_pose("right", active=True, xyz=np.array([0.8, -0.2, 0.7]), quat=quat)

        guard.begin_external_motion()

        for state in guard.states.values():
            self.assertFalse(state.active)
            self.assertIsNone(state.hold_pos)
            self.assertIsNone(state.hold_quat)
        published_before_tick = len(ros.published)
        guard.tick()
        self.assertEqual(len(ros.published), published_before_tick)

        guard.hold_current_all(reason="test home complete")

        np.testing.assert_allclose(guard.states["left"].hold_pos, ros.actual["left"][0])
        np.testing.assert_allclose(guard.states["right"].hold_pos, ros.actual["right"][0])

    def test_home_button_is_one_global_edge_for_both_hands(self) -> None:
        teleop = object.__new__(DualTianyiVrTeleop)
        teleop.args = types.SimpleNamespace(home_cooldown_s=2.0)
        teleop._homing = False
        teleop._home_button_pressed = False
        teleop._last_home_time = 0.0
        teleop.active_hands = ("left", "right")
        teleop.sides = {
            "left": SideState(
                side="left",
                name="left",
                home_pos=np.array([0.4, 0.2, 0.5]),
                home_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            ),
            "right": SideState(
                side="right",
                name="right",
                home_pos=np.array([0.4, -0.2, 0.5]),
                home_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            ),
        }
        calls: list[str] = []
        teleop._set_home_targets = lambda **_kwargs: calls.append("set-home")
        teleop._move_to_home = lambda **_kwargs: calls.append("saved-home")
        teleop._apply_home_sequence = lambda **_kwargs: calls.append("joint-home")

        def payload(pressed: bool) -> dict:
            buttons = [{"pressed": False} for _ in range(6)]
            buttons[5]["pressed"] = pressed
            return {
                "controllers": [
                    {"handedness": "left", "buttons": buttons},
                    {"handedness": "right", "buttons": buttons},
                ]
            }

        teleop._latest_vr_data = payload(True)
        teleop._consume_latest_vr_data()
        teleop._consume_latest_vr_data()

        self.assertEqual(calls, ["set-home", "saved-home"])
        self.assertEqual(
            teleop.sides["left"].last_home_time,
            teleop.sides["right"].last_home_time,
        )

    def test_home_button_falls_back_only_when_startup_target_is_missing(self) -> None:
        teleop = object.__new__(DualTianyiVrTeleop)
        teleop.args = types.SimpleNamespace(home_cooldown_s=2.0)
        teleop._homing = False
        teleop._last_home_time = 0.0
        teleop.active_hands = ("left", "right")
        teleop.sides = {
            "left": SideState(
                side="left",
                name="left",
                home_pos=np.array([0.4, 0.2, 0.5]),
                home_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            ),
            "right": SideState(side="right", name="right"),
        }
        calls: list[str] = []
        teleop._set_home_targets = lambda **_kwargs: calls.append("set-home")
        teleop._move_to_home = lambda **_kwargs: calls.append("saved-home")
        teleop._apply_home_sequence = lambda **_kwargs: calls.append("joint-home")

        teleop._go_home_from_button(teleop.sides["left"])

        self.assertEqual(calls, ["joint-home"])

    def test_home_sequence_ends_with_explicit_inactive_home_target(self) -> None:
        teleop = object.__new__(DualTianyiVrTeleop)
        teleop.args = types.SimpleNamespace(
            home_offset_xyz=(0.0, 0.0, 0.0),
            home_max_step_m=1.0,
            home_duration_s=0.2,
            control_hz=5.0,
            release_freeze_s=0.1,
            udp_host="127.0.0.1",
            udp_port=19011,
        )
        teleop.ros = None
        teleop.udp_sock = _FakeSocket()
        teleop._homing = False
        teleop.active_hands = ("left", "right")
        quat = np.array([1.0, 0.0, 0.0, 0.0])
        homes = {
            "left": np.array([0.40, 0.49, 0.09]),
            "right": np.array([0.40, -0.49, 0.09]),
        }
        starts = {
            "left": np.array([0.49, 0.25, 0.14]),
            "right": np.array([0.33, -0.18, 0.25]),
        }
        teleop.sides = {
            side: SideState(
                side=side,
                name=side,
                home_pos=home.copy(),
                home_quat_wxyz=quat.copy(),
                hold_pos=starts[side].copy(),
                hold_quat_wxyz=quat.copy(),
            )
            for side, home in homes.items()
        }
        teleop._current_tcp = (  # type: ignore[method-assign]
            lambda side: (starts[side].copy(), quat.copy())
        )

        teleop._move_to_home(label="test")

        packets = [json.loads(raw.decode("utf-8")) for raw in teleop.udp_sock.payloads]
        self.assertTrue(packets[0]["left"]["active"])
        for packet in packets[-3:]:
            self.assertFalse(packet["left"]["active"])
            self.assertFalse(packet["right"]["active"])
            np.testing.assert_allclose(packet["left"]["xyz"], homes["left"])
            np.testing.assert_allclose(packet["right"]["xyz"], homes["right"])

    def test_release_locks_measured_tcp_not_previous_command(self) -> None:
        ros = _FakeRos()
        guard = TeleopHoldGuard(ros, watchdog_timeout_s=0.15)
        command = np.array([0.60, 0.20, 0.60])
        quat = np.array([1.0, 0.0, 0.0, 0.0])

        guard.accept_pose("left", active=True, xyz=command, quat=quat, now=10.0)
        guard.accept_pose("left", active=False, now=10.01)

        state = guard.states["left"]
        self.assertFalse(state.active)
        np.testing.assert_allclose(state.hold_pos, ros.actual["left"][0])
        np.testing.assert_allclose(ros.published[-1][1], ros.actual["left"][0])
        self.assertFalse(np.allclose(ros.published[-1][1], command))

    def test_watchdog_locks_measured_tcp(self) -> None:
        ros = _FakeRos()
        guard = TeleopHoldGuard(ros, watchdog_timeout_s=0.15)
        guard.accept_pose(
            "right",
            active=True,
            xyz=np.array([0.6, -0.2, 0.6]),
            quat=np.array([1.0, 0.0, 0.0, 0.0]),
            now=20.0,
        )

        guard.tick(now=20.16)

        self.assertFalse(guard.states["right"].active)
        np.testing.assert_allclose(guard.states["right"].hold_pos, ros.actual["right"][0])

    def test_explicit_home_release_cannot_be_overwritten_by_watchdog_tf(self) -> None:
        ros = _FakeRos()
        guard = TeleopHoldGuard(ros, watchdog_timeout_s=0.15)
        home = np.array([0.40, 0.49, 0.09])
        quat = np.array([1.0, 0.0, 0.0, 0.0])

        guard.accept_pose("left", active=True, xyz=home, quat=quat, now=10.0)
        guard.accept_pose("left", active=False, xyz=home, quat=quat, now=10.1)
        # Simulate a much later watchdog tick while measured TF is still stale.
        guard.tick(now=20.0)

        state = guard.states["left"]
        self.assertFalse(state.active)
        np.testing.assert_allclose(state.hold_pos, home)
        self.assertFalse(np.allclose(state.hold_pos, ros.actual["left"][0]))

    def test_release_packet_is_explicitly_inactive(self) -> None:
        teleop = object.__new__(DualTianyiVrTeleop)
        teleop.args = types.SimpleNamespace(
            release_freeze_s=0.45,
            udp_host="127.0.0.1",
            udp_port=19011,
        )
        teleop.ros = None
        teleop.udp_sock = _FakeSocket()
        teleop._homing = False
        state = SideState(
            side="left",
            name="left",
            is_clutching=True,
            desired_pos=np.array([0.5, 0.2, 0.5]),
            desired_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        teleop.sides = {"left": state}
        teleop.active_hands = ("left",)

        teleop._release_clutch(state)
        teleop._publish_targets()

        payload = json.loads(teleop.udp_sock.payloads[-1].decode("utf-8"))
        self.assertFalse(payload["left"]["active"])

    def test_rotation_delta_is_applied_in_world_frame(self) -> None:
        teleop = object.__new__(DualTianyiVrTeleop)
        ref_ctrl = _axis_angle((1.0, 0.0, 0.0), math.radians(65.0))
        world_delta = _axis_angle((0.0, 0.0, 1.0), math.radians(30.0))
        cur_ctrl = quat_multiply_wxyz(world_delta, ref_ctrl)
        ref_ee = _axis_angle((0.0, 1.0, 0.0), math.radians(40.0))
        expected = quat_multiply_wxyz(
            quat_multiply_wxyz(cur_ctrl, quat_inverse_wxyz(ref_ctrl)), ref_ee
        )

        actual = teleop._apply_rot_deadzone(
            ref_ctrl,
            cur_ctrl,
            ref_ee,
            rotation_scale=1.0,
            deadzone_deg=0.0,
        )

        self.assertTrue(_same_rotation(actual, expected))

    def test_hold_a_release_keeps_current_orientation(self) -> None:
        teleop = object.__new__(DualTianyiVrTeleop)
        teleop.args = types.SimpleNamespace(
            home_cooldown_s=2.0,
            grip_release=0.35,
            grip_engage=0.55,
            position_scale=1.0,
            pos_deadzone_m=0.0,
            pos_filter_alpha=1.0,
            rotation_mode="hold-a",
            rotation_scale=1.0,
            rot_deadzone_deg=0.0,
            rot_filter_alpha=1.0,
            max_cmd_step_m=1.0,
        )
        teleop._homing = False
        identity = np.array([1.0, 0.0, 0.0, 0.0])
        state = SideState(
            side="right",
            name="right",
            is_clutching=True,
            hold_pos=np.zeros(3),
            hold_quat_wxyz=identity.copy(),
            ref_ee_pos=np.zeros(3),
            ref_ee_quat_wxyz=identity.copy(),
            ref_controller_xyz=np.zeros(3),
            ref_controller_quat_wxyz=identity.copy(),
            filt_pos=np.zeros(3),
            filt_quat_wxyz=identity.copy(),
            desired_pos=np.zeros(3),
            desired_quat_wxyz=identity.copy(),
        )

        def controller(quat: np.ndarray, a_pressed: bool) -> dict:
            w, x, y, z = quat
            buttons = [{"pressed": False} for _ in range(6)]
            buttons[4]["pressed"] = a_pressed
            return {
                "grip": 1.0,
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "qx": x,
                "qy": y,
                "qz": z,
                "qw": w,
                "buttons": buttons,
            }

        teleop._update_from_controller(state, controller(identity, True))
        moved = _axis_angle((0.0, 1.0, 0.0), math.radians(25.0))
        teleop._update_from_controller(state, controller(moved, True))
        orientation_before_release = state.desired_quat_wxyz.copy()
        teleop._update_from_controller(state, controller(moved, False))

        self.assertTrue(_same_rotation(state.desired_quat_wxyz, orientation_before_release))

    def test_never_rotation_mode_is_disabled(self) -> None:
        self.assertFalse(rotation_enabled({}, "never"))
        self.assertFalse(rotation_enabled({}, "off"))
        self.assertTrue(rotation_enabled({}, "always"))


if __name__ == "__main__":
    unittest.main()
