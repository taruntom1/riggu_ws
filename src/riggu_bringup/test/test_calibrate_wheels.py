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

"""Calibration maths in scripts/calibrate_wheels.py."""

import importlib.util
import math
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'calibrate_wheels.py'
spec = importlib.util.spec_from_file_location('calibrate_wheels', SCRIPT)
cal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cal)

R, B = 0.06675, 0.390


def test_straight_round_trip():
    # true radius 2% larger than configured: 2 m needs fewer wheel radians
    true_r = R * 1.02
    angle = 2.0 / true_r
    assert cal.radius_from_straight(angle, angle, 2.0) == pytest.approx(true_r)
    assert cal.straight_error(angle, angle, R, 2.0) == pytest.approx(-0.02 / 1.02)
    assert cal.straight_error(-angle, -angle, true_r, -2.0) == pytest.approx(0.0)


def test_turns_round_trip():
    true_b = B * 1.05
    turns = 10
    # in place, CCW: right wheel forward, left back; each wheel covers turns*pi*b
    arc = turns * math.pi * true_b / R
    assert cal.separation_from_turns(-arc, arc, R, turns) == pytest.approx(true_b)
    assert cal.separation_from_turns(arc, -arc, R, -turns) == pytest.approx(true_b)
    assert cal.turns_error(-arc, arc, R, true_b, turns) == pytest.approx(0.0)
    assert cal.turns_error(-arc, arc, R, B, turns) == pytest.approx(0.05)


def test_load_wheel():
    config = Path(__file__).resolve().parents[2] / 'riggu_description' / 'config' / 'robot.yaml'
    radius, separation = cal.load_wheel(config)
    assert 0.02 < radius < 0.2
    assert 0.1 < separation < 1.0
