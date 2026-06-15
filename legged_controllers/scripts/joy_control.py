#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2: subscribe Joy, publish geometry_msgs/Twist on cmd_vel (teleop + deadman),
and ocs2_msgs/ModeSchedule on rising edges of button combos (same as legacy joy_teleop + joy_gait_publisher)."""

from __future__ import annotations

import re
import time
from typing import Any, Dict, List, Optional, Tuple

import yaml

import rclpy
from geometry_msgs.msg import Twist
from ocs2_msgs.msg import ModeSchedule
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int8

# ocs2_legged_robot MotionPhaseDefinition.h
MODE_NAME_TO_INT = {
    "FLY": 0,
    "RH": 1,
    "LH": 2,
    "LH_RH": 3,
    "RF": 4,
    "RF_RH": 5,
    "RF_LH": 6,
    "RF_LH_RH": 7,
    "LF": 8,
    "LF_RH": 9,
    "LF_LH": 10,
    "LF_LH_RH": 11,
    "LF_RF": 12,
    "LF_RF_RH": 13,
    "LF_RF_LH": 14,
    "STANCE": 15,
}


def _btn_pressed(msg: Joy, idx: int, thresh: float) -> bool:
    if idx < 0 or idx >= len(msg.buttons):
        return False
    try:
        return float(msg.buttons[idx]) > thresh
    except (TypeError, ValueError):
        return False


def _parse_list_names(content: str) -> List[str]:
    m = re.search(r"list\s*\{([^}]*)\}", content, flags=re.DOTALL)
    if not m:
        return []
    names = []
    for line in m.group(1).splitlines():
        mm = re.search(r"\]\s*(\w+)", line.strip())
        if mm:
            names.append(mm.group(1))
    return names


def _extract_braced_block(text: str, key: str) -> Optional[str]:
    i = text.find(key)
    if i < 0:
        return None
    j = text.find("{", i)
    if j < 0:
        return None
    depth = 0
    for k in range(j, len(text)):
        if text[k] == "{":
            depth += 1
        elif text[k] == "}":
            depth -= 1
            if depth == 0:
                return text[j + 1 : k]
    return None


def parse_gait_info(path: str) -> Dict[str, Tuple[List[float], List[int]]]:
    """Return dict gait_name -> (event_times, mode_sequence)."""
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        content = f.read()
    gaits: Dict[str, Tuple[List[float], List[int]]] = {}
    for name in _parse_list_names(content):
        block_m = re.search(
            r"^" + re.escape(name) + r"\s*\n\s*\{",
            content,
            flags=re.MULTILINE,
        )
        if not block_m:
            continue
        start = block_m.end() - 1
        depth = 0
        end = start
        for k in range(start, len(content)):
            if content[k] == "{":
                depth += 1
            elif content[k] == "}":
                depth -= 1
                if depth == 0:
                    end = k
                    break
        block = content[start + 1 : end]

        ms_inner = _extract_braced_block(block, "modeSequence")
        st_inner = _extract_braced_block(block, "switchingTimes")
        if ms_inner is None or st_inner is None:
            continue

        mode_seq: List[int] = []
        for line in ms_inner.splitlines():
            mm = re.search(r"\]\s*(\w+)", line.strip())
            if mm:
                tok = mm.group(1)
                if tok not in MODE_NAME_TO_INT:
                    raise ValueError(f"unknown mode token '{tok}' in gait '{name}'")
                mode_seq.append(MODE_NAME_TO_INT[tok])

        times: List[float] = []
        for line in st_inner.splitlines():
            mm = re.search(r"\]\s*([\d.+-eE]+)", line.strip())
            if mm:
                times.append(float(mm.group(1)))

        if len(times) != len(mode_seq) + 1:
            pass  # same warning as ROS1 optional
        gaits[name] = (times, mode_seq)
    return gaits



def combo_active(msg: Joy, required: List[int], thresh: float) -> bool:
    if not required:
        return False
    for b in required:
        if not _btn_pressed(msg, b, thresh):
            return False
    return True


class JoyControl(Node):
    def __init__(self) -> None:
        super().__init__("joy_control")

        self._btn_thresh = float(self.declare_parameter("button_threshold", 0.5).value)
        self._cooldown_sec = float(self.declare_parameter("cooldown_sec", 0.25).value)
        self._publish_repeats = int(self.declare_parameter("publish_repeats", 3).value)
        self._publish_repeat_dt = float(self.declare_parameter("publish_repeat_dt", 0.03).value)
        self._debug = bool(self.declare_parameter("debug", False).value)
        # 映射后线速度/角速度绝对值小于死区则置 0，避免摇杆漂移导致「自己不摸也在走」
        self._db_lin = float(self.declare_parameter("cmd_vel_deadband_linear", 0.05).value)
        self._db_ang = float(self.declare_parameter("cmd_vel_deadband_angular", 0.05).value)

        joy_topic: str = self.declare_parameter("joy_topic", "joy").value
        if not str(joy_topic).startswith("/"):
            joy_topic = "/" + str(joy_topic)

        robot_name: str = self.declare_parameter("robot_name", "legged_robot").value
        mode_topic: str = self.declare_parameter(
            "mode_schedule_topic", f"/{robot_name}_mpc_mode_schedule"
        ).value
        if not str(mode_topic).startswith("/"):
            mode_topic = "/" + str(mode_topic)

        teleop_path = self.declare_parameter("teleop_config_file", "").value
        mappings_path = self.declare_parameter("gait_mappings_file", "").value
        gait_file = self.declare_parameter("gait_command_file", "").value
        self._stand_gait_name = str(self.declare_parameter("stand_gait_name", "stance").value)
        self._lie_down_gait_name = str(self.declare_parameter("lie_down_gait_name", "lie_down").value)
        locomotion_gaits = str(self.declare_parameter("locomotion_gaits", "trot").value)
        self._locomotion_gaits = {g.strip().lower() for g in locomotion_gaits.split(",") if g.strip()}
        emergency_stop_topic = str(self.declare_parameter("emergency_stop_topic", "/emergency_stop").value)
        if not emergency_stop_topic.startswith("/"):
            emergency_stop_topic = "/" + emergency_stop_topic
        self._emergency_stop_buttons = [
            int(x) for x in self.declare_parameter("emergency_stop_buttons", [8, 9]).value
        ]

        self._teleop = self._load_teleop(str(teleop_path))
        cmd_topic: str = self.declare_parameter("cmd_vel_topic", "").value
        if not str(cmd_topic):
            cmd_topic = str(self._teleop.get("topic_name", "/cmd_vel"))
        if not str(cmd_topic).startswith("/"):
            cmd_topic = "/" + str(cmd_topic)

        self._mappings = []
        self._gaits: Dict[str, Tuple[List[float], List[int]]] = {}

        raw_mappings: List[Dict[str, Any]] = []
        if str(mappings_path):
            raw_mappings = self._load_mappings(str(mappings_path))

        emergency_stop_buttons = self._load_emergency_stop_buttons(str(mappings_path))
        if emergency_stop_buttons is not None:
            self._emergency_stop_buttons = emergency_stop_buttons

        if raw_mappings:
            if not str(gait_file):
                self.get_logger().fatal("已配置 gait_mappings_file 时必须设置 gait_command_file（gait.info 路径）")
                raise RuntimeError("gait_command_file")
            try:
                self._gaits = parse_gait_info(str(gait_file))
            except Exception as e:
                self.get_logger().fatal(f"解析步态文件失败 {gait_file}: {e}")
                raise
            self._mappings = [m for m in raw_mappings if m["gait"] in self._gaits]
            if not self._mappings:
                self.get_logger().fatal("组合映射为空或步态名均不在 gait 文件中")
                raise RuntimeError("empty mappings")

        self._prev_joy: Optional[Joy] = None
        self._last_fire = self.get_clock().now() - Duration(seconds=3600)
        self._last_gait_name: Optional[str] = None
        self._last_n_conn = 0
        self._last_cmd_zero = True
        self._emergency_stop_active = False

        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        stop_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._emergency_stop_pub = self.create_publisher(Bool, emergency_stop_topic, stop_qos)

        mode_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._mode_pub = self.create_publisher(ModeSchedule, mode_topic, mode_qos)

        posture_command_topic = str(
            self.declare_parameter("posture_command_topic", f"/{robot_name}_posture_command").value
        )
        if not posture_command_topic.startswith("/"):
            posture_command_topic = "/" + posture_command_topic
        # 固定姿态命令(Int8: 1=STAND, 2=LIE_DOWN, 0=解除)。stance/lie_down 改发此话题，不再发 mode_schedule/target。
        self._posture_pub = self.create_publisher(Int8, posture_command_topic, mode_qos)

        self.create_subscription(Joy, joy_topic, self._joy_cb, 1)
        self.create_timer(0.5, self._on_timer)

        wait_sec = float(self.declare_parameter("wait_for_subscribers_sec", 2.0).value)
        t0 = time.monotonic()
        while rclpy.ok() and (time.monotonic() - t0) < wait_sec:
            if self._mode_pub.get_subscription_count() > 0:
                break
            time.sleep(0.05)
        n = self._mode_pub.get_subscription_count()
        self._last_n_conn = n
        self.get_logger().info(f"话题 {mode_topic} 当前订阅者数 = {n}")
        if n == 0 and self._mappings:
            self.get_logger().warn(
                "尚无 ModeSchedule 订阅者；控制器就绪后本节点会补发上次步态。"
            )

        self.get_logger().info(f"订阅 Joy: {joy_topic}；发布 Twist: {cmd_topic}")
        self.get_logger().warn(
            f"急停组合 buttons={self._emergency_stop_buttons}；触发后发布 {emergency_stop_topic}=true 并锁存"
        )
        if self._mappings:
            self.get_logger().info(
                f"步态: {gait_file}；已加载 {len(self._mappings)} 条组合映射"
            )
            for i, m in enumerate(self._mappings):
                self.get_logger().info(f"  [{i}] gait={m['gait']} buttons={m['buttons']}")

    def _load_teleop(self, path: str) -> Dict[str, Any]:
        if not path:
            self.get_logger().fatal("必须设置参数 teleop_config_file（joy.yaml）")
            raise RuntimeError("teleop_config_file")
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        teleop = data.get("teleop") or {}
        if "walk" not in teleop:
            self.get_logger().fatal("teleop_config_file 中缺少 teleop.walk")
            raise RuntimeError("teleop.walk")
        walk = teleop["walk"]
        if "deadman_buttons" not in walk or "axis_mappings" not in walk:
            self.get_logger().fatal("teleop.walk 需要 deadman_buttons 与 axis_mappings")
            raise RuntimeError("teleop.walk fields")
        for m in walk["axis_mappings"]:
            axis_raw = m.get("axis")
            if isinstance(axis_raw, bool):
                self.get_logger().fatal(f"axis 必须是整数轴编号，收到 {axis_raw!r}")
                raise RuntimeError("teleop.walk axis")
            try:
                axis_value = float(axis_raw)
            except (TypeError, ValueError):
                self.get_logger().fatal(f"axis 必须是整数轴编号，收到 {axis_raw!r}")
                raise RuntimeError("teleop.walk axis")
            if not axis_value.is_integer():
                self.get_logger().fatal(f"axis 必须是整数轴编号，收到 {axis_raw!r}")
                raise RuntimeError("teleop.walk axis")
            m["axis"] = int(axis_value)
        topic_name = str(walk.get("topic_name", "/cmd_vel"))
        if not topic_name.startswith("/"):
            topic_name = "/" + topic_name
        walk["topic_name"] = topic_name
        return walk

    def _load_mappings(self, path: str) -> List[Dict[str, Any]]:
        if not path:
            return []
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        out: List[Dict[str, Any]] = []
        entries = raw.get("mappings") if isinstance(raw, dict) else raw
        if not isinstance(entries, list):
            return out
        for entry in entries:
            if not isinstance(entry, dict):
                continue
            g = str(entry.get("gait", "")).lower().strip()
            bt = entry.get("buttons", [])
            if not g or not isinstance(bt, list):
                continue
            out.append({"gait": g, "buttons": [int(x) for x in bt]})
        return out

    def _load_emergency_stop_buttons(self, path: str) -> Optional[List[int]]:
        if not path:
            return None
        with open(path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        if not isinstance(raw, dict):
            return None
        config = raw.get("emergency_stop")
        if not isinstance(config, dict):
            return None
        buttons = config.get("buttons")
        if not isinstance(buttons, list):
            return None
        return [int(x) for x in buttons]

    def _publish_emergency_stop(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self._emergency_stop_pub.publish(msg)

    def _publish_gait(self, name: str) -> None:
        self._last_gait_name = name

        if name == self._stand_gait_name:
            # 固定姿态(站立)：绕过 MPC，只发姿态命令，不发 mode_schedule。
            self._publish_posture_command(1, "stand")
            self.get_logger().info(f"已发布步态 '{name}' (固定姿态-站立)")
            return
        if name == self._lie_down_gait_name:
            # 固定姿态(趴下)：绕过 MPC，只发姿态命令，不发 mode_schedule。
            self._publish_posture_command(2, "lie_down")
            self.get_logger().info(f"已发布步态 '{name}' (固定姿态-趴下)")
            return

        # 运动步态：解除固定姿态(发 0)并发布 mode_schedule 启动 MPC。
        self._publish_posture_command(0, "none")
        event_times, mode_sequence = self._gaits[name]
        msg = ModeSchedule()
        msg.event_times = event_times
        msg.mode_sequence = [int(m) for m in mode_sequence]
        for _ in range(max(1, self._publish_repeats)):
            self._mode_pub.publish(msg)
            time.sleep(self._publish_repeat_dt)

        self.get_logger().info(f"已发布步态 '{name}'")

    def _publish_posture_command(self, posture: int, name: str) -> None:
        msg = Int8()
        msg.data = int(posture)
        self._posture_pub.publish(msg)
        self.get_logger().info(f"已发布姿态命令 {name} ({posture})")

    def _republish_gait_light(self, name: str) -> None:
        # 固定姿态(stance/lie_down)补发姿态命令；运动步态补发 mode_schedule。
        if name == self._stand_gait_name:
            self._publish_posture_command(1, "stand")
            return
        if name == self._lie_down_gait_name:
            self._publish_posture_command(2, "lie_down")
            return
        event_times, mode_sequence = self._gaits[name]
        msg = ModeSchedule()
        msg.event_times = event_times
        msg.mode_sequence = [int(m) for m in mode_sequence]
        for _ in range(5):
            self._mode_pub.publish(msg)
            time.sleep(0.02)

    def _on_timer(self) -> None:
        n = self._mode_pub.get_subscription_count()
        if n > self._last_n_conn and self._last_gait_name is not None:
            self.get_logger().info(
                f"订阅者 {self._last_n_conn} -> {n}，补发步态 '{self._last_gait_name}'"
            )
            self._republish_gait_light(self._last_gait_name)
        self._last_n_conn = n

    def _deadman_ok(self, msg: Joy) -> bool:
        for b in self._teleop["deadman_buttons"]:
            if not _btn_pressed(msg, int(b), self._btn_thresh):
                return False
        return True

    def _make_twist(self, msg: Joy) -> Twist:
        t = Twist()
        for m in self._teleop["axis_mappings"]:
            axis = int(m["axis"])
            target = str(m["target"])
            scale = float(m.get("scale", 1.0))
            if axis < 0 or axis >= len(msg.axes):
                continue
            try:
                v = float(msg.axes[axis]) * scale
            except (TypeError, ValueError):
                continue
            if target == "linear.x":
                t.linear.x = v
            elif target == "linear.y":
                t.linear.y = v
            elif target == "linear.z":
                t.linear.z = v
            elif target == "angular.z":
                t.angular.z = v
        return t

    def _apply_cmd_deadband(self, t: Twist) -> None:
        if abs(t.linear.x) < self._db_lin:
            t.linear.x = 0.0
        if abs(t.linear.y) < self._db_lin:
            t.linear.y = 0.0
        if abs(t.linear.z) < self._db_lin:
            t.linear.z = 0.0
        if abs(t.angular.z) < self._db_ang:
            t.angular.z = 0.0

    @staticmethod
    def _is_zero_twist(t: Twist) -> bool:
        return (
            t.linear.x == 0.0
            and t.linear.y == 0.0
            and t.linear.z == 0.0
            and t.angular.x == 0.0
            and t.angular.y == 0.0
            and t.angular.z == 0.0
        )

    def _joy_cb(self, msg: Joy) -> None:
        now = self.get_clock().now()
        buttons = list(msg.buttons)
        if self._prev_joy is None:
            self._prev_joy = Joy()
            self._prev_joy.buttons = [0] * len(buttons)

        if self._debug:
            self.get_logger().debug(f"buttons={buttons}")

        emergency_now = combo_active(msg, self._emergency_stop_buttons, self._btn_thresh)
        emergency_prev = combo_active(self._prev_joy, self._emergency_stop_buttons, self._btn_thresh)
        if emergency_now and not emergency_prev:
            self._emergency_stop_active = True
            self._last_gait_name = None
            self._publish_emergency_stop(True)
            if not self._last_cmd_zero:
                self._cmd_pub.publish(Twist())
                self._last_cmd_zero = True
            self.get_logger().fatal("急停已触发：已发布 emergency_stop=true，当前步态已清空")

        if self._emergency_stop_active:
            self._prev_joy = msg
            return

        gait_fired = False
        if self._mappings and self._gaits:
            for m in self._mappings:
                req = m["buttons"]
                on_now = combo_active(msg, req, self._btn_thresh)
                on_prev = combo_active(self._prev_joy, req, self._btn_thresh)
                if on_now and not on_prev:
                    dt = (now - self._last_fire).nanoseconds / 1e9
                    if dt < self._cooldown_sec:
                        break
                    self._last_fire = now
                    self._publish_gait(m["gait"])
                    gait_fired = True
                    break

        twist = Twist()
        if self._deadman_ok(msg):
            twist = self._make_twist(msg)
            self._apply_cmd_deadband(twist)

        if gait_fired or self._last_gait_name not in self._locomotion_gaits:
            twist = Twist()

        # Re-publishing zero velocity continuously makes the target generator
        # reset the target pose to the current drifting pose. Publish zero once
        # to stop, then keep the previous fixed target until a non-zero command.
        is_zero = self._is_zero_twist(twist)
        if is_zero and self._last_cmd_zero:
            self._prev_joy = msg
            return
        self._cmd_pub.publish(twist)
        self._last_cmd_zero = is_zero
        self._prev_joy = msg


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = JoyControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # SIGINT 时 rclpy 可能已由 C++ 信号处理关闭，避免二次 shutdown 报错
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
