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

"""Static checks: slam_toolbox / AMCL config, launch lidar range, sim reference map."""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
ROBOT_YAML = Path(get_package_share_directory('riggu_description')) / 'config' / 'robot.yaml'


def load(path):
    """Read a yaml file."""
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def launch_module(name):
    """Import a launch file from this package's source tree."""
    spec = importlib.util.spec_from_file_location(name, PKG / 'launch' / f'{name}.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def robot():
    """riggu_description robot.yaml."""
    return load(ROBOT_YAML)


def test_slam_toolbox_params():
    """REP-105 frames, /scan, 5 cm cells; laser range left to the launch."""
    p = load(PKG / 'config' / 'slam_toolbox.yaml')['slam_toolbox']['ros__parameters']
    assert (p['map_frame'], p['odom_frame'], p['base_frame']) == ('map', 'odom',
                                                                  'base_footprint')
    assert p['scan_topic'] == '/scan'
    assert p['mode'] == 'mapping'
    assert p['resolution'] == pytest.approx(0.05)
    assert 'min_laser_range' not in p and 'max_laser_range' not in p


def test_amcl_params():
    """Diff-drive motion model, REP-105 frames; laser range and map left to the launch."""
    params = load(PKG / 'config' / 'nav2_params.yaml')
    p = params['amcl']['ros__parameters']
    assert p['robot_model_type'] == 'nav2_amcl::DifferentialMotionModel'
    assert (p['global_frame_id'], p['odom_frame_id'], p['base_frame_id']) == (
        'map', 'odom', 'base_footprint')
    assert p['tf_broadcast'] is True
    assert 'laser_min_range' not in p and 'laser_max_range' not in p
    assert params['map_server']['ros__parameters']['yaml_filename'] == ''


@pytest.mark.parametrize('name', ['slam', 'localization'])
def test_launch_lidar_range(name, robot):
    """Both launches take the laser range from robot.yaml (C1: 0.05-12 m)."""
    context = LaunchContext()
    context.launch_configurations['robot_config'] = str(ROBOT_YAML)
    assert launch_module(name).lidar_range(context) == (
        robot['lidar']['range_min']['value'], robot['lidar']['range_max']['value'])


def test_sim_reference_map():
    """maps/house.yaml: 5 cm, trinary, image next to it, covers the 10 x 8 m house."""
    meta = load(PKG / 'maps' / 'house.yaml')
    assert meta['resolution'] == pytest.approx(0.05)
    image = PKG / 'maps' / meta['image']
    assert image.is_file()
    with open(image, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = (int(v) for v in line.split())
    assert width * meta['resolution'] >= 10.0
    assert height * meta['resolution'] >= 8.0
