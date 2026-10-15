"""Manual calibration samples, shared solver and reproducible JSON results."""

import datetime
import json
from pathlib import Path
import uuid

import numpy as np
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation

from .handeye_calibration import PoseMartix, solve_handeye


def checked_pose(values):
    value = np.asarray(values, dtype=float)
    if value.shape != (7,) or not np.isfinite(value).all():
        raise ValueError('位姿必须包含 7 个有限数值：XYZ 和四元数 XYZW。')
    norm = np.linalg.norm(value[3:])
    if norm < 1e-9:
        raise ValueError('位姿四元数为零，不能采集。')
    result = value.copy()
    result[3:] /= norm
    return result.tolist()


def to_pose(values):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = values[:3]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = values[3:]
    return pose


class CalibrationSession:
    def __init__(self, mode='eye_in_hand', min_samples=10):
        if mode not in ('eye_in_hand', 'eye_to_hand'):
            raise ValueError('不支持的标定模式。')
        if int(min_samples) != min_samples or min_samples < 5:
            raise ValueError('最少采样数必须是大于等于 5 的整数。')
        self.mode = mode
        self.min_samples = int(min_samples)
        self.samples = []

    def add_sample(self, robot, marker, *, camera_frame, captured_at):
        robot = checked_pose(robot)
        marker = checked_pose(marker)
        if not isinstance(camera_frame, str) or not camera_frame.strip():
            raise ValueError('标记位姿缺少相机参考坐标系，不能采集。')
        if self.samples and self.samples[0]['camera_frame'] != camera_frame:
            raise ValueError('相机参考坐标系发生变化，请清空后重新采集。')
        if not np.isfinite(captured_at):
            raise ValueError('无效的采样时间。')
        sample = dict(robot=robot, marker=marker, camera_frame=camera_frame,
                      captured_at=float(captured_at))
        self.samples.append(sample)
        return sample

    def undo(self):
        return self.samples.pop() if self.samples else None

    def clear(self):
        self.samples.clear()

    def solve(self):
        if len(self.samples) < self.min_samples:
            raise ValueError('样本不足：至少需要 %d 组，当前 %d 组。' %
                             (self.min_samples, len(self.samples)))
        rotations = Rotation.from_quat([sample['robot'][3:] for sample in self.samples])
        relative_vectors = (rotations[0].inv() * rotations[1:]).as_rotvec()
        singular_values = np.linalg.svd(relative_vectors, compute_uv=False)
        if (np.max(np.linalg.norm(relative_vectors, axis=1)) < np.deg2rad(3.0) or
                singular_values[1] < np.deg2rad(1.0)):
            raise ValueError('旋转姿态不足：请绕至少两个不平行轴采集明显不同的姿态，不能只平移或重复采样。')
        piper_poses = [PoseMartix(to_pose(sample['robot']), True, self.mode)
                       for sample in self.samples]
        marker_poses = [PoseMartix(to_pose(sample['marker']), False, self.mode)
                        for sample in self.samples]
        result = solve_handeye(piper_poses, marker_poses)
        matrix = np.eye(4)
        matrix[:3, :3] = result.rotation_matrix
        matrix[:3, 3] = result.translation_matrix
        if not np.isfinite(matrix).all():
            raise ValueError('解算结果包含无效数值，请增加多轴旋转姿态并重新采样。')
        destination = 'end_pose 所定义的末端坐标系' if self.mode == 'eye_in_hand' else 'end_pose 所使用的机械臂基坐标系'
        return dict(
            position=result.position_list, orientation=result.orientation_list,
            rpy=[list(result.rpy_list[0])], transform_matrix=matrix.tolist(),
            sample_count=len(self.samples), mode=self.mode,
            camera_frame=self.samples[0]['camera_frame'],
            transform_direction='将相机坐标变换到' + destination,
            units={'position': 'm', 'rpy': 'rad', 'quaternion_order': 'xyzw'},
        )

    def save(self, directory, metadata):
        result = self.solve()
        result['samples'] = self.samples
        result['metadata'] = dict(metadata)
        result['created_at'] = datetime.datetime.now().astimezone().isoformat()
        text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        name = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S_%f')
        path = directory / (name + '_' + uuid.uuid4().hex[:8] + '_calibration.json')
        with path.open('x', encoding='utf-8') as stream:
            stream.write(text)
            stream.write('\n')
        return path
