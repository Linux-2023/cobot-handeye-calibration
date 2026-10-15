import time

import pytest
import rclpy
from PyQt5.QtWidgets import QApplication
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image

from handeye_calibration_ros.calibration_gui import (
    CalibrationGuiNode, CalibrationController, DEFAULTS, MAX_AGE,
)
from handeye_calibration_ros.calibration_window import CalibrationWindow


@pytest.fixture
def controller():
    app = QApplication.instance() or QApplication([])
    rclpy.init()
    node = CalibrationGuiNode()
    window = CalibrationWindow(DEFAULTS)
    ctl = CalibrationController(node, window)
    ctl.spin_timer.stop()
    ctl.render_timer.stop()
    errors = []
    window.show_error = errors.append
    ctl.connected = True
    yield ctl, errors, app
    ctl.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    window.deleteLater()
    app.processEvents()


def feed(ctl):
    robot = Pose()
    robot.position.z = 0.2
    robot.orientation.w = 1.0
    marker = PoseStamped()
    marker.header.frame_id = 'test_optical'
    marker.header.stamp = ctl.node.get_clock().now().to_msg()
    marker.pose.position.z = 0.4
    marker.pose.orientation.w = 1.0
    image = Image()
    image.header.stamp = marker.header.stamp
    for key, value in [('robot', robot), ('marker', marker), ('image', image)]:
        ctl.receive(key, value)


def test_capture_waits_for_both_messages_after_click(controller):
    ctl, errors, _ = controller
    feed(ctl)
    ctl.request_capture()
    assert ctl.pending is not None
    ctl.try_capture()
    assert not ctl.session.samples
    feed(ctl)
    ctl.try_capture()
    assert len(ctl.session.samples) == 1
    assert ctl.pending is None
    assert not errors


def test_capture_rejects_stale_marker_receipt(controller):
    ctl, errors, _ = controller
    feed(ctl)
    msg, receipt = ctl.latest['marker']
    ctl.latest['marker'] = (msg, receipt - MAX_AGE - 0.1)
    ctl.request_capture()
    assert ctl.pending is None
    assert not ctl.session.samples
    assert errors


def test_capture_rejects_delayed_marker_timestamp(controller):
    ctl, errors, _ = controller
    feed(ctl)
    ctl.latest['marker'][0].header.stamp.sec -= 2
    ctl.request_capture()
    assert ctl.pending is None
    assert not ctl.session.samples
    assert errors


def test_capture_times_out_without_storing_stale_pose(controller):
    ctl, errors, _ = controller
    feed(ctl)
    ctl.pending = time.monotonic() - 4.0
    ctl.try_capture()
    assert ctl.pending is None
    assert not ctl.session.samples
    assert errors


def test_reconfiguration_cannot_mix_samples(controller):
    ctl, errors, _ = controller
    feed(ctl)
    ctl.request_capture()
    feed(ctl)
    ctl.try_capture()
    ctl.disconnect()
    changed = dict(DEFAULTS, piper_topic='/different_arm/end_pose')
    ctl.connect_or_disconnect(changed)
    assert not ctl.connected
    assert len(ctl.session.samples) == 1
    assert ctl.config['piper_topic'] == DEFAULTS['piper_topic']
    assert errors


def test_invalid_capture_clears_pending_before_error_dialog(controller):
    ctl, _, _ = controller
    feed(ctl)
    ctl.request_capture()
    feed(ctl)
    ctl.latest['robot'][0].orientation.w = 0.0
    observed = []

    def error_dialog(message):
        observed.append(message)
        assert ctl.pending is None
        ctl.try_capture()

    ctl.window.show_error = error_dialog
    ctl.try_capture()
    assert len(observed) == 1
    assert not ctl.session.samples


def test_configuration_remains_locked_until_samples_cleared(controller):
    ctl, _, _ = controller
    feed(ctl)
    ctl.request_capture()
    feed(ctl)
    ctl.try_capture()
    ctl.disconnect()
    assert not ctl.window.fields['piper_topic'].isEnabled()
    ctl.clear()
    assert ctl.window.fields['piper_topic'].isEnabled()
