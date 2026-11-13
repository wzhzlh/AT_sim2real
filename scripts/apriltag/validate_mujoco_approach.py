#!/usr/bin/env python3
"""Run the real MuJoCo viewer, policy and AprilTag node in a closed loop.

Run after sourcing ROS and install/setup.bash. Requires an X11 desktop;
libXtst sends real key presses to this test's own MuJoCo window.
Ground truth is used only for assertions, never as controller input.
"""
import argparse
import ctypes
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


def planar_yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class Desktop:
    def __init__(self):
        self.x = ctypes.CDLL('libX11.so.6')
        self.xt = ctypes.CDLL('libXtst.so.6')
        self.x.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x.XOpenDisplay.restype = ctypes.c_void_p
        self.display = self.x.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('No X11 display')
        self.x.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x.XKeysymToKeycode.restype = ctypes.c_uint
        self.x.XSetInputFocus.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
        self.x.XFlush.argtypes = [ctypes.c_void_p]
        self.x.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.xt.XTestFakeKeyEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]

    @staticmethod
    def windows():
        tree = subprocess.check_output(['xwininfo', '-root', '-tree'], text=True)
        return {int(value, 16) for value in re.findall(r'(0x[0-9a-f]+) "MuJoCo[^\"]*"', tree)}

    def key(self, window, char):
        self.x.XSetInputFocus(self.display, window, 1, 0)
        code = self.x.XKeysymToKeycode(self.display, ord(char))
        self.xt.XTestFakeKeyEvent(self.display, code, 1, 0)
        self.x.XFlush(self.display)
        time.sleep(0.05)
        self.xt.XTestFakeKeyEvent(self.display, code, 0, 0)
        self.x.XFlush(self.display)

    def close(self):
        self.x.XCloseDisplay(self.display)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--move-tag', action='store_true', help='Move tag after arrival and verify automatic re-approach')
    parser.add_argument('--interrupt', action='store_true', help='Suspend detector to test watchdog stand and recovery')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain', type=int, default=89)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
    desktop = Desktop()
    previous_windows = desktop.windows()
    rclpy.init()
    node = rclpy.create_node('validate_mujoco_approach')
    poses, goals, commands, stands = [], [], [], []
    latest_image = [None]
    start_time = time.monotonic()

    def stamp():
        return time.monotonic() - start_time

    def on_pose(msg):
        p, q = msg.pose.position, msg.pose.orientation
        poses.append([stamp(), p.x, p.y, p.z, planar_yaw(msg.pose),
                      math.acos(max(-1.0, min(1.0, 1 - 2*(q.x*q.x + q.y*q.y))))])

    subscriptions = [
        node.create_subscription(PoseStamped, '/mujoco/base_pose', on_pose, qos_profile_sensor_data),
        node.create_subscription(PoseStamped, '/tag_approach/goal_pose_body',
                                 lambda m: goals.append([stamp(), m.pose.position.x, m.pose.position.y, planar_yaw(m.pose)]), 10),
        node.create_subscription(Twist, '/mujoco/cmd_vel',
                                 lambda m: commands.append([stamp(), m.linear.x, m.linear.y, m.angular.z]), 10),
        node.create_subscription(Bool, '/mujoco/stand', lambda m: stands.append([stamp(), m.data]), 10),
        node.create_subscription(Image, '/mujoco/camera/image_raw',
                                 lambda m: latest_image.__setitem__(0, m), qos_profile_sensor_data),
    ]
    tag_pub = node.create_publisher(PoseStamped, '/mujoco/tag_pose', 1)
    log = (output / 'process.log').open('w')
    process = subprocess.Popen(
        ['ros2', 'launch', 'apriltag_localization', 'tag_approach.launch.py',
         'use_mujoco:=true', 'show_image:=false'], cwd=root,
        stdout=log, stderr=log, start_new_session=True)
    suspended_pid = None
    result = {'passed': False, 'interrupt': args.interrupt, 'checks': {}}

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
            if process.poll() is not None:
                raise RuntimeError('Launch exited; inspect process.log')

    def snapshot(name):
        msg = latest_image[0]
        if msg is not None:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(output / name), frame)

    def check(name, condition, evidence):
        result['checks'][name] = {'passed': bool(condition), 'evidence': evidence}
        print(name, 'PASS' if condition else 'FAIL', evidence, flush=True)
        if not condition:
            raise AssertionError(name)

    try:
        deadline = time.monotonic() + 25
        window = None
        while time.monotonic() < deadline:
            spin(0.2)
            new_windows = desktop.windows() - previous_windows
            if new_windows and poses and goals:
                window = max(new_windows)
                break
        check('startup_and_tag5_detection', window is not None and bool(goals), len(goals))
        snapshot('initial.png')
        desktop.key(window, '0')
        spin(4)
        check('stand_up', poses[-1][3] > 0.2 and abs(poses[-1][5]) < math.radians(15), poses[-1])
        snapshot('initial.png')
        # Only 0 and 1 are needed: 1 starts both the detector and RL tracking.
        check('stand_key_does_not_autowalk', all(max(abs(v) for v in c[1:]) < 1e-6 for c in commands), len(commands))
        desktop.key(window, '1')
        moving_start = stamp()
        spin(1.5)
        check('policy_moves_forward', poses[-1][1] > 0.3, poses[-1])
        if args.interrupt:
            table = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True)
            executable = str(root / 'install/apriltag_localization/lib/apriltag_localization/tag_approach')
            suspended_pid = next(int(line.split()[0]) for line in table.splitlines()
                                 if line.split(maxsplit=1)[-1].startswith(executable + ' '))
            os.kill(suspended_pid, signal.SIGSTOP)
            spin(3)
            recent = [p for p in poses if p[0] > stamp() - 1]
            drift = math.hypot(recent[-1][1] - recent[0][1], recent[-1][2] - recent[0][2])
            check('detector_watchdog_stand', drift < 0.015 and poses[-1][3] > 0.2, {'drift_m': drift})
            os.kill(suspended_pid, signal.SIGCONT)
            suspended_pid = None
        for i in range(30):
            spin(1)
            p = poses[-1]
            print(f't={stamp():.1f} x={p[1]:.3f} y={p[2]:.3f} yaw={math.degrees(p[4]):.2f}', flush=True)
            check_fall = p[3] > 0.18 and abs(p[5]) < math.radians(20)
            if not check_fall:
                raise AssertionError('Robot fell')
            if '已到达目标位姿' in (output / 'process.log').read_text():
                break
        check('arrival_detected', '已到达目标位姿' in (output / 'process.log').read_text(), stamp() - moving_start)
        spin(2)
        hold_start = len(poses)
        spin(10)
        hold = poses[hold_start:]
        target_error = math.hypot(poses[-1][1] - 1.4, poses[-1][2])
        drift = max(math.hypot(p[1] - hold[0][1], p[2] - hold[0][2]) for p in hold)
        check('final_xy_error', target_error < 0.07, target_error)
        check('final_yaw_error', abs(poses[-1][4]) < math.radians(5), math.degrees(poses[-1][4]))
        check('ten_second_hold', drift < 0.015, drift)
        check('zero_velocity_and_stand', bool(stands[-1][1]) and max(abs(v) for v in commands[-1][1:]) < 1e-6, commands[-1])
        if args.move_tag:
            target = PoseStamped()
            target.header.frame_id = 'world'
            target.pose.position.x = 2.3
            target.pose.position.z = 0.5
            target.pose.orientation.w = 0.5
            target.pose.orientation.x = 0.5
            target.pose.orientation.y = -0.5
            target.pose.orientation.z = -0.5
            previous_x = poses[-1][1]
            command_index = len(commands)
            tag_pub.publish(target)
            arrival_count = (output / 'process.log').read_text().count('已到达目标位姿')
            stable_stand = False
            for _ in range(25):
                spin(1)
                recent_stands = [row for row in stands if row[0] > stamp() - 2]
                stable_stand = (bool(recent_stands) and all(row[1] for row in recent_stands) and
                                math.hypot(poses[-1][1] - 1.7, poses[-1][2]) < 0.07 and
                                (output / 'process.log').read_text().count('已到达目标位姿') > arrival_count)
                if stable_stand:
                    break
            check('moving_tag_reaches_stand', stable_stand, poses[-1])
            check('moving_tag_resumes_rl',
                  poses[-1][1] > previous_x + 0.20 and
                  any(c[1] > 0.1 for c in commands[command_index:]), poses[-1])
            spin(3)
            hold_index = len(poses)
            spin(10)
            moved_hold = poses[hold_index:]
            error = math.hypot(poses[-1][1] - 1.7, poses[-1][2])
            drift = max(math.hypot(p[1] - moved_hold[0][1], p[2] - moved_hold[0][2]) for p in moved_hold)
            check('moving_tag_arrival_error', error < 0.07, error)
            check('moving_tag_hold', drift < 0.015, drift)
            hold = poses[-1:]
        desktop.key(window, '0')
        spin(2)
        check('manual_stop_holds', math.hypot(poses[-1][1] - hold[-1][1], poses[-1][2] - hold[-1][2]) < 0.015, poses[-1])
        if args.move_tag:
            stopped = poses[-1]
            target.pose.position.x = 2.6
            tag_pub.publish(target)
            spin(3)
            check('zero_key_disables_following',
                  math.hypot(poses[-1][1] - stopped[1], poses[-1][2] - stopped[2]) < 0.015,
                  poses[-1])
        snapshot('final.png')
        result['passed'] = True
    finally:
        if suspended_pid:
            os.kill(suspended_pid, signal.SIGCONT)
        result['trace'] = {'poses': poses, 'goals': goals, 'commands': commands, 'stand': stands}
        (output / 'result.json').write_text(json.dumps(result, indent=2))
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        desktop.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
