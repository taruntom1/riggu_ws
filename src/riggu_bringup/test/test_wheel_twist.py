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

"""Diff-drive kinematics in scripts/wheel_twist.py."""

import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'wheel_twist.py'
spec = importlib.util.spec_from_file_location('wheel_twist', SCRIPT)
wheel_twist = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wheel_twist)

R, B = 0.06374, 0.3853


def test_straight():
    assert wheel_twist.twist_from_wheels(2.0, 2.0, R, B) == pytest.approx((2.0 * R, 0.0))


def test_turn_in_place_left_is_positive_yaw():
    vx, vyaw = wheel_twist.twist_from_wheels(-1.0, 1.0, R, B)
    assert vx == pytest.approx(0.0)
    assert vyaw == pytest.approx(2.0 * R / B) and vyaw > 0


def test_matches_diff_drive_convention():
    # one wheel only: half the speed, yaw rate = wheel speed / separation
    vx, vyaw = wheel_twist.twist_from_wheels(0.0, 3.0, R, B)
    assert vx == pytest.approx(1.5 * R)
    assert vyaw == pytest.approx(3.0 * R / B)
