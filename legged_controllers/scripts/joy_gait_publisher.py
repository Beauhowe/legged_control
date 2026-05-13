#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Subscribe to sensor_msgs/Joy (default ~joy_topic=/legged_robot/joystick,与 joy_teleop.launch 一致)
and publish ocs2_msgs/mode_schedule on button-combo rising edges.
Same gait topic as legged_robot_gait_command: /legged_robot_mpc_mode_schedule

Gait templates from param /gaitCommandFile (load_controller.launch sets this).
Mappings from private param ~mappings (see config/joy_gait_mappings.yaml).
"""

from __future__ import annotations

import re
import sys

import rospy
from ocs2_msgs.msg import mode_schedule
from sensor_msgs.msg import Joy

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


def _parse_list_names(content: str):
    m = re.search(r"list\s*\{([^}]*)\}", content, flags=re.DOTALL)
    if not m:
        return []
    names = []
    for line in m.group(1).splitlines():
        mm = re.search(r"\]\s*(\w+)", line.strip())
        if mm:
            names.append(mm.group(1))
    return names


def _extract_braced_block(text: str, key: str):
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


def parse_gait_info(path: str):
    """Return dict gait_name -> (event_times, mode_sequence)."""
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        content = f.read()
    gaits = {}
    for name in _parse_list_names(content):
        block_m = re.search(
            r"^" + re.escape(name) + r"\s*\n\s*\{",
            content,
            flags=re.MULTILINE,
        )
        if not block_m:
            rospy.logwarn_throttle(60, "joy_gait: gait block not found for '%s'" % name)
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
            rospy.logwarn_throttle(60, "joy_gait: missing modeSequence/switchingTimes in '%s'" % name)
            continue

        mode_seq = []
        for line in ms_inner.splitlines():
            mm = re.search(r"\]\s*(\w+)", line.strip())
            if mm:
                tok = mm.group(1)
                if tok not in MODE_NAME_TO_INT:
                    rospy.logerr("joy_gait: unknown mode token '%s' in gait '%s'" % (tok, name))
                    raise ValueError(tok)
                mode_seq.append(MODE_NAME_TO_INT[tok])

        times = []
        for line in st_inner.splitlines():
            mm = re.search(r"\]\s*([\d.+-eE]+)", line.strip())
            if mm:
                times.append(float(mm.group(1)))

        if len(times) != len(mode_seq) + 1:
            rospy.logwarn(
                "joy_gait: gait '%s' len(switchingTimes)=%d vs len(modeSequence)+1=%d"
                % (name, len(times), len(mode_seq) + 1)
            )
        gaits[name] = (times, mode_seq)
    return gaits


def combo_active(msg: Joy, required, thresh: float) -> bool:
    if not required:
        return False
    for b in required:
        if not _btn_pressed(msg, b, thresh):
            return False
    return True


class JoyGaitPublisher:
    def __init__(self):
        self._btn_thresh = float(rospy.get_param("~button_threshold", 0.5))
        self._gait_file = rospy.get_param("/gaitCommandFile", "")
        if not self._gait_file:
            rospy.logfatal(
                "joy_gait: /gaitCommandFile 未设置。请先启动 load_controller.launch，再启动本节点；"
                "或 rosparam set /gaitCommandFile $(rospack find legged_controllers)/config/a1/gait.info"
            )
            sys.exit(1)

        try:
            self._gaits = parse_gait_info(self._gait_file)
        except Exception as e:
            rospy.logfatal("joy_gait: 解析步态文件失败 %s: %s" % (self._gait_file, e))
            sys.exit(1)

        robot = rospy.get_param("~robot_name", "legged_robot")
        topic = rospy.get_param("~mode_schedule_topic", "/" + robot + "_mpc_mode_schedule")
        if not topic.startswith("/"):
            topic = "/" + topic
        self._topic = topic
        self._pub = rospy.Publisher(topic, mode_schedule, queue_size=1, latch=True)

        raw = rospy.get_param("~mappings", None)
        if raw is None:
            rospy.logfatal(
                "joy_gait: 未找到私有参数 ~mappings。请确认 launch 里在 <node joy_gait_publisher> 内加载了 joy_gait_mappings.yaml，"
                "且 roslaunch ... enable_joy_gait:=true"
            )
            sys.exit(1)

        self._mappings = []
        if isinstance(raw, dict) and "mappings" in raw:
            raw = raw["mappings"]
        for entry in raw:
            if not isinstance(entry, dict):
                continue
            g = str(entry.get("gait", "")).lower().strip()
            bt = entry.get("buttons", [])
            if not g or not isinstance(bt, list):
                rospy.logwarn("joy_gait: 跳过无效项: %s" % entry)
                continue
            if g not in self._gaits:
                rospy.logwarn("joy_gait: 步态 '%s' 不在 gait 文件中，已跳过" % g)
                continue
            self._mappings.append({"gait": g, "buttons": [int(x) for x in bt]})

        if not self._mappings:
            rospy.logfatal("joy_gait: ~mappings 为空或全部被跳过，请编辑 joy_gait_mappings.yaml")
            sys.exit(1)

        self._prev = None
        self._cooldown = rospy.Duration(float(rospy.get_param("~cooldown_sec", 0.25)))
        self._last_fire = rospy.Time(0)
        self._publish_repeats = int(rospy.get_param("~publish_repeats", 3))
        self._publish_repeat_dt = float(rospy.get_param("~publish_repeat_dt", 0.03))
        self._debug = bool(rospy.get_param("~debug", False))
        self._last_gait_name = None
        self._last_n_conn = 0

        wait_sec = float(rospy.get_param("~wait_for_subscribers_sec", 2.0))
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < wait_sec:
            n = self._pub.get_num_connections()
            if n > 0:
                break
            rospy.sleep(0.05)
        n = self._pub.get_num_connections()
        self._last_n_conn = n
        rospy.loginfo("joy_gait: 话题 %s 当前订阅者数 = %d" % (topic, n))
        if n == 0:
            rospy.logwarn(
                "joy_gait: 尚无订阅者（GaitReceiver 在 legged_controller 进程内）。"
                "请 rosservice call 或 rqt 启动 controllers/legged_controller；"
                "启动后本节点会自动重发「上一次」手柄步态（若已按过组合键）。"
            )

        joy_topic = rospy.get_param("~joy_topic", "/legged_robot/joystick")
        if not str(joy_topic).startswith("/"):
            joy_topic = "/" + str(joy_topic)
        rospy.Subscriber(joy_topic, Joy, self._cb, queue_size=1)
        rospy.Timer(rospy.Duration(0.5), self._on_timer)
        rospy.loginfo("joy_gait: 订阅 Joy 话题 %s" % joy_topic)
        rospy.loginfo("joy_gait: 已加载 %d 条组合映射；步态文件: %s" % (len(self._mappings), self._gait_file))
        for i, m in enumerate(self._mappings):
            rospy.loginfo("  [%d] gait=%s buttons=%s" % (i, m["gait"], m["buttons"]))

    def _publish_gait(self, name: str):
        self._last_gait_name = name
        event_times, mode_sequence = self._gaits[name]
        msg = mode_schedule()
        msg.eventTimes = event_times
        msg.modeSequence = [int(m) for m in mode_sequence]
        for _ in range(max(1, self._publish_repeats)):
            self._pub.publish(msg)
            rospy.sleep(self._publish_repeat_dt)
        rospy.loginfo("joy_gait: 已发布步态 '%s' -> %s" % (name, self._topic))

    def _republish_gait_light(self, name: str):
        """控制器晚于手柄启动时，补发几次（避免仅 latch 未进 MPC）。"""
        event_times, mode_sequence = self._gaits[name]
        msg = mode_schedule()
        msg.eventTimes = event_times
        msg.modeSequence = [int(m) for m in mode_sequence]
        for _ in range(5):
            self._pub.publish(msg)
            rospy.sleep(0.02)

    def _on_timer(self, _event):
        n = self._pub.get_num_connections()
        if n > self._last_n_conn and self._last_gait_name is not None:
            rospy.loginfo(
                "joy_gait: 订阅者从 %d -> %d，补发步态 '%s' 给 GaitReceiver"
                % (self._last_n_conn, n, self._last_gait_name)
            )
            self._republish_gait_light(self._last_gait_name)
        self._last_n_conn = n

    def _cb(self, msg: Joy):
        now = rospy.Time.now()
        buttons = list(msg.buttons)

        if self._prev is None:
            self._prev = Joy()
            self._prev.buttons = [0] * len(buttons)

        if self._debug:
            rospy.logdebug_throttle(1.0, "joy_gait: buttons=%s" % (buttons,))

        for m in self._mappings:
            req = m["buttons"]
            on_now = combo_active(msg, req, self._btn_thresh)
            on_prev = combo_active(self._prev, req, self._btn_thresh)
            if on_now and not on_prev:
                if now - self._last_fire < self._cooldown:
                    rospy.logdebug_throttle(0.5, "joy_gait: cooldown 忽略一次触发")
                    break
                self._last_fire = now
                self._publish_gait(m["gait"])
                break

        self._prev = msg


def main():
    rospy.init_node("joy_gait_publisher")
    JoyGaitPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
