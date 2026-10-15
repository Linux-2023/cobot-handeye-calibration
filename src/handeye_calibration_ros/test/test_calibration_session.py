import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from handeye_calibration_ros.calibration_session import CalibrationSession


def transform(rot, xyz):
    value = np.eye(4)
    value[:3, :3] = Rotation.from_rotvec(rot).as_matrix()
    value[:3, 3] = xyz
    return value


def pose_vector(value):
    return value[:3, 3].tolist() + Rotation.from_matrix(value[:3, :3]).as_quat().tolist()


def make_session(mode):
    session = CalibrationSession(mode=mode, min_samples=10)
    expected = transform([0.1, -0.15, 0.2], [0.04, -0.03, 0.08])
    fixed = transform([-0.2, 0.1, 0.05], [0.4, 0.0, 0.5])
    rng = np.random.default_rng(17)
    for i in range(18):
        robot = transform(rng.uniform(-0.8, 0.8, 3), rng.uniform(-0.2, 0.2, 3))
        if mode == 'eye_in_hand':
            marker = np.linalg.inv(expected) @ np.linalg.inv(robot) @ fixed
        else:
            marker = np.linalg.inv(expected) @ robot @ fixed
        session.add_sample(pose_vector(robot), pose_vector(marker), camera_frame='test_optical', captured_at=float(i))
    return session, expected


@pytest.mark.parametrize('mode', ['eye_in_hand', 'eye_to_hand'])
def test_recovers_known_transform_and_saves_samples(tmp_path, mode):
    session, expected = make_session(mode)
    result = session.solve()
    np.testing.assert_allclose(result['transform_matrix'], expected, atol=1e-7)
    assert len(result['rpy']) == 1
    assert result['camera_frame'] == 'test_optical'
    path = session.save(tmp_path, {'piper_topic': '/test/pose'})
    saved = json.loads(path.read_text())
    assert saved['sample_count'] == 18
    assert len(saved['samples']) == 18
    assert saved['metadata']['piper_topic'] == '/test/pose'
    assert session.save(tmp_path, {}) != path


def test_undo_clear_and_too_few_samples():
    session, _ = make_session('eye_in_hand')
    session.undo()
    assert len(session.samples) == 17
    session.clear()
    assert not session.samples
    with pytest.raises(ValueError):
        session.solve()


def test_rejects_stationary_samples():
    session = CalibrationSession()
    identity = [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
    for i in range(10):
        session.add_sample(identity, identity, camera_frame='test_optical', captured_at=float(i))
    with pytest.raises(ValueError):
        session.solve()


def test_rejects_invalid_pose_and_mixed_frames():
    session = CalibrationSession()
    identity = [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
    session.add_sample(identity, identity, camera_frame='one', captured_at=1.0)
    with pytest.raises(ValueError):
        session.add_sample(identity, identity, camera_frame='two', captured_at=2.0)
    invalid = [float('nan')] + identity[1:]
    with pytest.raises(ValueError):
        session.add_sample(invalid, identity, camera_frame='one', captured_at=2.0)
    with pytest.raises(ValueError):
        session.add_sample(identity[:3] + [0.0] * 4, identity, camera_frame='one', captured_at=2.0)
    assert len(session.samples) == 1


def test_rejects_single_axis_rotations():
    session = CalibrationSession()
    for i in range(10):
        pose = pose_vector(transform([0.0, 0.0, i * 0.1], [i * 0.01, 0.0, 0.3]))
        session.add_sample(pose, pose, camera_frame='one', captured_at=float(i))
    with pytest.raises(ValueError):
        session.solve()


def test_samples_are_independent_of_incoming_pose_lists():
    session = CalibrationSession()
    pose = [0.0, 0.0, 0.4, 0.0, 0.0, 0.0, 1.0]
    session.add_sample(pose, pose, camera_frame='one', captured_at=1.0)
    pose[0] = 100.0
    assert session.samples[0]['robot'][0] == 0.0
    assert session.samples[0]['marker'][0] == 0.0
