#!/usr/bin/env python3
"""Read-only ROS subscriptions and Qt controls for manual hand-eye calibration."""

import json
import signal
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.validate_full_topic_name import validate_full_topic_name
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from PyQt5.QtCore import QObject, QTimer
from PyQt5.QtGui import QImage
from PyQt5.QtWidgets import QApplication

from .calibration_session import CalibrationSession
from .calibration_window import CalibrationWindow


DEFAULTS = {
    'piper_topic': '/piper_right_ctrl_node/end_pose',
    'marker_topic': '/aruco_single/pose',
    'image_topic': '/aruco_single/result',
    'mode': 'eye_in_hand',
    'result_save_path': str(Path.home() / 'hand_eye/results/right_wrist'),
    'min_num': 10,
}
MAX_AGE = 0.7
MAX_PAIR_DELAY = 0.15
CAPTURE_TIMEOUT = 3.0


def pose_values(pose):
    p, q = pose.position, pose.orientation
    return [p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class CalibrationGuiNode(Node):
    def __init__(self):
        super().__init__('handeye_calibration_gui')
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        self.config = {name: self.get_parameter(name).value for name in DEFAULTS}


class CalibrationController(QObject):
    """Spin ROS on the Qt thread; capture only new, fresh messages after a click."""

    def __init__(self, node, window):
        super().__init__(window)
        self.node = node
        self.window = window
        self.bridge = CvBridge()
        self.config = dict(node.config)
        self.session = CalibrationSession(self.config['mode'], self.config['min_num'])
        self.connected = False
        self.subscriptions = []
        self.latest = {}
        self.pending = None
        self.last_image = None
        self.last_image_error = ''
        window.connect_requested.connect(self.connect_or_disconnect)
        window.capture_requested.connect(self.request_capture)
        window.undo_requested.connect(self.undo)
        window.clear_requested.connect(self.clear)
        window.solve_requested.connect(self.solve_and_save)
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.spin)
        self.spin_timer.start(20)
        self.render_timer = QTimer(self)
        self.render_timer.timeout.connect(self.refresh)
        self.render_timer.start(100)
        window.append_log('本界面不控制机械臂。采集时请保持机械臂静止，直到采集完成。')
        window.append_log('末端 Pose 无时间戳：按到达时间配对，不是硬件同步。建议采集 15–25 组多轴旋转姿态。')

    def connect_or_disconnect(self, config):
        if self.connected:
            self.disconnect()
            return
        try:
            config = dict(config)
            for key in ('piper_topic', 'marker_topic', 'image_topic'):
                config[key] = config[key].strip()
                validate_full_topic_name(config[key])
            if config['mode'] not in ('eye_in_hand', 'eye_to_hand'):
                raise ValueError('标定模式必须是 eye_in_hand 或 eye_to_hand')
            config['min_num'] = int(config['min_num'])
            if config['min_num'] < 5:
                raise ValueError('最少采样数不能少于 5，建议至少 15 组。')
            if not str(config['result_save_path']).strip():
                raise ValueError('请选择结果保存目录。')
            config['result_save_path'] = str(Path(config['result_save_path']).expanduser())
            if self.session.samples and config != self.config:
                raise ValueError('已有样本。请先清空，再修改话题、模式或保存配置，避免混用数据。')
            if not self.session.samples:
                self.session = CalibrationSession(config['mode'], config['min_num'])
            self.config = config
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.latest.clear()
            self.last_image = None
            self.last_image_error = ''
            for key, msg_type, topic in (
                ('robot', Pose, config['piper_topic']),
                ('marker', PoseStamped, config['marker_topic']),
                ('image', Image, config['image_topic']),
            ):
                self.subscriptions.append(self.node.create_subscription(
                    msg_type, topic, lambda msg, key=key: self.receive(key, msg), qos))
            self.connected = True
            self.window.set_connection_state(True)
            self.window.set_samples(self.session.samples)
            self.window.append_log('已连接：' + config['piper_topic'] + ' + ' + config['marker_topic'])
            self.window.append_log('图像：' + config['image_topic'] + '；模式：' + config['mode'])
        except Exception as exc:
            self.disconnect(log=False)
            self.window.show_error(str(exc))

    def disconnect(self, log=True):
        self.pending = None
        for sub in self.subscriptions:
            self.node.destroy_subscription(sub)
        self.subscriptions.clear()
        self.latest.clear()
        self.connected = False
        self.window.set_busy(False)
        self.window.set_connection_state(False)
        if log:
            self.window.append_log('已断开订阅；机械臂和相机驱动保持运行。')
        self.refresh()

    def receive(self, key, msg):
        self.latest[key] = (msg, time.monotonic())

    def fresh(self, key, now):
        entry = self.latest.get(key)
        if entry is None or now - entry[1] > MAX_AGE:
            return False
        if key != 'robot':
            age = self.node.get_clock().now().nanoseconds / 1e9 - stamp_seconds(entry[0].header.stamp)
            if age < -0.1 or age > MAX_AGE:
                return False
        return True

    def can_capture(self, now):
        return (self.connected and not self.last_image_error and
                all(self.fresh(key, now) for key in ('robot', 'marker', 'image')))

    def request_capture(self):
        if self.pending is not None:
            return
        now = time.monotonic()
        if not self.can_capture(now):
            self.window.show_error('图像、标记或末端位姿尚未就绪／已过期，不能采集。')
            return
        self.pending = now
        self.window.set_busy(True)
        self.window.append_log('正在采集新数据，请保持机械臂静止……')

    def try_capture(self):
        if self.pending is None:
            return
        now = time.monotonic()
        if now - self.pending > CAPTURE_TIMEOUT:
            self.pending = None
            self.window.set_busy(False)
            self.window.show_error('采集超时：未收到新鲜且相近的两路位姿。请检查标记可见性和话题。')
            return
        if not self.can_capture(now):
            return
        robot, robot_received = self.latest['robot']
        marker, marker_received = self.latest['marker']
        if min(robot_received, marker_received) <= self.pending:
            return
        if abs(robot_received - marker_received) > MAX_PAIR_DELAY:
            return
        self.pending = None
        try:
            self.session.add_sample(
                pose_values(robot), pose_values(marker.pose),
                camera_frame=marker.header.frame_id,
                captured_at=stamp_seconds(marker.header.stamp),
            )
            self.window.set_samples(self.session.samples)
            self.window.append_log('已采集第 %d 组，可以移动到下一姿态。' % len(self.session.samples))
        except Exception as exc:
            self.window.show_error(str(exc))
        finally:
            self.pending = None
            self.window.set_busy(False)

    def spin(self):
        if not rclpy.ok():
            return
        try:
            for _ in range(6):
                rclpy.spin_once(self.node, timeout_sec=0.0)
            self.try_capture()
        except Exception as exc:
            self.disconnect(log=False)
            self.window.show_error('ROS 接收异常：' + str(exc))

    def refresh(self):
        now = time.monotonic()
        statuses = {}
        for key, label in (('robot', '机械臂'), ('marker', '标记'), ('image', '图像')):
            if not self.connected:
                statuses[key] = label + '：未连接'
            elif not self.fresh(key, now):
                statuses[key] = label + '：等待数据／已过期'
            elif key == 'image':
                msg = self.latest[key][0]
                statuses[key] = '图像：%d × %d' % (msg.width, msg.height)
            else:
                msg = self.latest[key][0]
                p = msg.position if key == 'robot' else msg.pose.position
                statuses[key] = '%s：在线  XYZ %.4f / %.4f / %.4f m' % (label, p.x, p.y, p.z)
        if self.last_image_error:
            statuses['image'] = '图像转换失败：' + self.last_image_error
        self.window.set_status(statuses['robot'], statuses['marker'], statuses['image'],
                               self.can_capture(now) and self.pending is None and not self.last_image_error)
        if 'image' in self.latest:
            msg = self.latest['image'][0]
            if msg is not self.last_image:
                try:
                    rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                    image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
                    self.window.set_image(image)
                    self.last_image_error = ''
                except Exception as exc:
                    error = str(exc)
                    if error != self.last_image_error:
                        self.window.append_log('图像转换失败：' + error)
                    self.last_image_error = error
                self.last_image = msg

    def undo(self):
        if self.pending is not None:
            return
        self.session.undo()
        self.window.set_samples(self.session.samples)
        self.window.append_log('已撤销上一组采样。')

    def clear(self):
        self.pending = None
        self.window.set_busy(False)
        self.session.clear()
        self.window.set_samples(self.session.samples)
        self.window.append_log('已清空采样，可开始新的标定。')

    def solve_and_save(self):
        if self.pending is not None:
            return
        self.window.set_busy(True)
        try:
            metadata = dict(self.config)
            metadata['pairing'] = 'Fresh messages received after click; unstamped robot pose; arm must remain stationary.'
            path = self.session.save(self.config['result_save_path'], metadata)
            result = json.loads(path.read_text(encoding='utf-8'))
            self.window.show_result(result, str(path))
            self.window.append_log('标定结果与采样数据已保存：' + str(path))
        except Exception as exc:
            self.window.show_error('计算／保存失败：' + str(exc))
        finally:
            self.window.set_busy(False)

    def shutdown(self):
        self.spin_timer.stop()
        self.render_timer.stop()
        self.disconnect(log=False)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv[:1])
    app.setApplicationName('手眼标定助手')
    node = CalibrationGuiNode()
    window = CalibrationWindow(node.config)
    controller = CalibrationController(node, window)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    window.show()
    print('手眼标定 GUI 已启动；只读订阅，不控制机械臂。', flush=True)
    QTimer.singleShot(0, lambda: controller.connect_or_disconnect(node.config))
    try:
        app.exec_()
    finally:
        controller.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
