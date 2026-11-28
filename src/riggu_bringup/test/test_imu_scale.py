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

"""Gyro scale correction in scripts/imu_scale.py."""

import importlib.util
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'imu_scale.py'
spec = importlib.util.spec_from_file_location('imu_scale', SCRIPT)
imu_scale = importlib.util.module_from_spec(spec)
spec.loader.exec_module(imu_scale)


def test_fast_gyro_is_slowed():
    # a gyro reading 3.2 % fast: 371.7 deg integrated over a true 360 deg spin
    assert imu_scale.corrected_rate(1.032, 1.032) == pytest.approx(1.0)
    assert imu_scale.corrected_rate(-0.5, 1.0) == -0.5
