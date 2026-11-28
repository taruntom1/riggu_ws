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

"""Yaw unwrapping and closed-path residuals in scripts/odom_check.py."""

import importlib.util
import math
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'odom_check.py'
spec = importlib.util.spec_from_file_location('odom_check', SCRIPT)
check = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check)


class Q:
    def __init__(self, yaw):
        self.x, self.y = 0.0, 0.0
        self.z, self.w = math.sin(yaw / 2), math.cos(yaw / 2)


@pytest.mark.parametrize('yaw', [0.0, 1.0, -2.5, math.pi - 1e-6])
def test_yaw_of(yaw):
    assert check.yaw_of(Q(yaw)) == pytest.approx(yaw)


def test_track_unwraps_full_spin():
    track = check.Track()
    for i in range(0, 361 + 360, 10):  # two turns, counter-clockwise, through +-pi
        track.update(check.wrap(math.radians(i)))
    assert track.yaw == pytest.approx(math.radians(720))


def test_turn_residual():
    assert check.turn_residual(math.radians(362.0)) == pytest.approx(math.radians(2.0))
    assert check.turn_residual(math.radians(-718.5)) == pytest.approx(math.radians(1.5))
    assert check.turn_residual(math.radians(-3.0)) == pytest.approx(math.radians(-3.0))


def test_report_distance_and_missing():
    start = {'/odom': (0.0, (0.0, 0.0)), '/imu': (0.0, None)}
    end = {'/odom': (2 * math.pi + 0.01, (0.3, 0.4)), '/imu': (2 * math.pi, None)}
    odom, filtered, imu = check.report(start, end)
    assert 'distance from start 0.500 m' in odom and 'residual  +0.57 deg' in odom
    assert 'no data' in filtered
    assert 'distance' not in imu and '+360.00 deg' in imu
