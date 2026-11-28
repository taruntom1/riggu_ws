# Copyright 2026 Tarun Tom
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
URDF checks: xacro expands per mode, check_urdf passes, the parts fit together.

The geometry checks guard later edits of config/robot.yaml: a corrected value
that makes a wheel float or the lidar sit inside the chassis fails here.
"""

import math
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

PKG = Path(__file__).resolve().parents[1]
XACRO_FILE = PKG / 'urdf' / 'riggu.urdf.xacro'
CONFIG = PKG / 'config' / 'robot.yaml'
MODES = ('real', 'sim', 'mock')
SOURCES = {'estimate', 'old_config', 'datasheet', 'user', 'measured', 'calibrated'}
IMU_INTERFACES = {f'{group}.{axis}'
                  for group, axes in (('orientation', 'xyzw'),
                                      ('angular_velocity', 'xyz'),
                                      ('linear_acceleration', 'xyz'))
                  for axis in axes}
TOL = 1e-3  # m


def expand(mode):
    """Return the URDF XML string for a mode."""
    doc = xacro.process_file(str(XACRO_FILE),
                             mappings={'mode': mode, 'robot_config': str(CONFIG)})
    return doc.toxml()


@pytest.fixture(scope='module')
def cfg():
    """Raw values from robot.yaml as {section: {key: value}}."""
    with CONFIG.open(encoding='utf-8') as stream:
        raw = yaml.safe_load(stream)
    return {section: {key: leaf['value'] for key, leaf in leaves.items()}
            for section, leaves in raw.items()}


@pytest.fixture(scope='module')
def model():
    """Forward kinematics over the mock-mode URDF."""
    return Model(ET.fromstring(expand('mock')))


# --- tiny forward kinematics ----------------------------------------------------

def _rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _origin(element):
    origin = element.find('origin') if element is not None else None
    xyz = [0.0] * 3
    rpy = [0.0] * 3
    if origin is not None:
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    return _rpy(*rpy), xyz


def _compose(parent, child):
    (r1, t1), (r2, t2) = parent, child
    rot = [[sum(r1[i][k] * r2[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    trans = [sum(r1[i][k] * t2[k] for k in range(3)) + t1[i] for i in range(3)]
    return rot, trans


class Model:
    """Link poses in base_footprint."""

    def __init__(self, root):
        """Index links and joints."""
        self.links = {link.get('name'): link for link in root.findall('link')}
        self.parent = {j.find('child').get('link'): j for j in root.findall('joint')}

    def pose(self, link):
        """Return (R, t) of a link frame in the root frame."""
        joint = self.parent.get(link)
        if joint is None:
            return _rpy(0, 0, 0), [0.0, 0.0, 0.0]
        return _compose(self.pose(joint.find('parent').get('link')), _origin(joint))

    def geometry(self, link, kind='collision'):
        """Return ((R, t), geometry element) of a link's first visual/collision."""
        element = self.links[link].find(kind)
        return _compose(self.pose(link), _origin(element)), element.find('geometry')[0]


# --- expansion and structure --------------------------------------------------

@pytest.mark.parametrize('mode', MODES)
def test_check_urdf(mode, tmp_path):
    """Each mode expands and passes check_urdf."""
    exe = shutil.which('check_urdf')
    assert exe, 'check_urdf not found (liburdfdom-tools)'
    urdf = tmp_path / f'riggu_{mode}.urdf'
    urdf.write_text(expand(mode), encoding='utf-8')
    result = subprocess.run([exe, str(urdf)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_bad_mode_fails():
    """An unknown mode is an error, not a model without hardware."""
    with pytest.raises(Exception, match='mode_must_be_real_sim_or_mock'):
        expand('simulation')


@pytest.mark.parametrize('mode', MODES)
def test_ros2_control_interfaces(mode):
    """Every mode exposes the same joints, interfaces and IMU sensor."""
    systems = ET.fromstring(expand(mode)).findall('ros2_control')
    assert len(systems) == 1
    system = systems[0]
    plugin = system.find('hardware/plugin').text.strip()
    expected = {'sim': 'gz_ros2_control/GazeboSimSystem',
                'mock': 'mock_components/GenericSystem'}
    if mode in expected:
        assert plugin == expected[mode]
    else:  # DriveLink owns its plugin name; just make sure it is not a stand-in
        assert plugin not in expected.values()

    joints = {j.get('name'): j for j in system.findall('joint')}
    assert set(joints) == {'left_wheel_joint', 'right_wheel_joint'}
    for joint in joints.values():
        assert [c.get('name') for c in joint.findall('command_interface')] == ['velocity']
        states = {s.get('name') for s in joint.findall('state_interface')}
        assert states == {'position', 'velocity'}

    sensors = system.findall('sensor')
    assert [s.get('name') for s in sensors] == ['imu']
    assert {s.get('name') for s in sensors[0].findall('state_interface')} == IMU_INTERFACES


def test_frames(model):
    """REP-105 tree with the frame names the rest of the stack uses."""
    assert set(model.links) == {'base_footprint', 'base_link', 'left_wheel', 'right_wheel',
                                'caster', 'laser', 'imu_link'}
    for link in model.links:
        if link != 'base_footprint':
            expected = 'base_footprint' if link == 'base_link' else 'base_link'
            assert model.parent[link].find('parent').get('link') == expected


# --- robot.yaml ---------------------------------------------------------------

def test_config_schema():
    """Every value in robot.yaml is a number with a known source."""
    with CONFIG.open(encoding='utf-8') as stream:
        raw = yaml.safe_load(stream)
    for section, leaves in raw.items():
        for key, leaf in leaves.items():
            name = f'{section}.{key}'
            assert set(leaf) <= {'value', 'source', 'note'}, name
            assert isinstance(leaf['value'], (int, float)), name
            assert leaf['source'] in SOURCES, name


# --- geometry: the parts fit together -----------------------------------------

def test_base_link_on_axle(model, cfg):
    """base_link sits on the axle midpoint, wheel_radius above the floor."""
    _, t = model.pose('base_link')
    assert t == pytest.approx([0, 0, cfg['wheel']['radius']], abs=1e-9)


@pytest.mark.parametrize('side,sign', [('left', 1), ('right', -1)])
def test_wheels(model, cfg, side, sign):
    """Wheels touch the floor, spin about +y, and clear the chassis."""
    (rot, centre), cyl = model.geometry(f'{side}_wheel')
    axis = [rot[i][2] for i in range(3)]
    assert axis == pytest.approx([0, 1, 0], abs=1e-9)
    joint_axis = model.parent[f'{side}_wheel'].find('axis').get('xyz')
    assert joint_axis == '0 0 1'
    assert centre[2] - float(cyl.get('radius')) == pytest.approx(0, abs=TOL)
    assert sign * centre[1] == pytest.approx(cfg['wheel']['separation'] / 2, abs=1e-9)
    inner_face = abs(centre[1]) - float(cyl.get('length')) / 2
    assert inner_face > cfg['chassis']['width'] / 2, 'wheel overlaps the chassis'


def test_caster(model, cfg):
    """Caster touches the floor, under the chassis, behind the axle."""
    (_, centre), sphere = model.geometry('caster')
    assert centre[2] - float(sphere.get('radius')) == pytest.approx(0, abs=TOL)
    x_min, x_max, _, _ = _chassis_box(model)
    assert x_min < centre[0] < x_max
    assert centre[0] < 0


def _chassis_box(model):
    (_, centre), box = model.geometry('base_link')
    sx, sy, sz = (float(v) for v in box.get('size').split())
    return centre[0] - sx / 2, centre[0] + sx / 2, centre[2] - sz / 2, centre[2] + sz / 2


def test_chassis(model, cfg):
    """Chassis clears the floor and spans the axle."""
    x_min, x_max, z_min, z_max = _chassis_box(model)
    assert z_min > TOL, 'chassis touches the floor'
    assert z_max == pytest.approx(cfg['chassis']['top_above_floor'], abs=1e-9)
    assert x_min < 0 < x_max


def test_lidar_sees_over_robot(model):
    """The scan plane is above the chassis top and the wheel tops."""
    _, laser = model.pose('laser')
    _, _, _, chassis_top = _chassis_box(model)
    (_, wheel), cyl = model.geometry('left_wheel')
    assert laser[2] > chassis_top
    assert laser[2] > wheel[2] + float(cyl.get('radius')), 'scan plane hits the wheels'


def test_imu_inside_chassis(model):
    """imu_link lies within the chassis volume."""
    _, imu = model.pose('imu_link')
    x_min, x_max, z_min, z_max = _chassis_box(model)
    assert x_min <= imu[0] <= x_max
    assert z_min <= imu[2] <= z_max


def test_total_mass(model):
    """Sum of link masses is plausible for a Pi 5 robot (0.5 to 10 kg)."""
    total = sum(float(link.find('inertial/mass').get('value'))
                for link in model.links.values() if link.find('inertial') is not None)
    assert 0.5 < total < 10
