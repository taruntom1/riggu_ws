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

"""Button logic of stop_latch (EdgeToggle) and arm_switch (ArmCombo)."""

import importlib.util
from pathlib import Path

PKG = Path(__file__).resolve().parents[1] / 'riggu_teleop'


def load(name):
    """Import riggu_teleop/<name>.py from the source tree."""
    spec = importlib.util.spec_from_file_location(name, PKG / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


EdgeToggle = load('stop_latch').EdgeToggle
ArmCombo = load('arm_switch').ArmCombo


def test_latch_toggles_on_press_only():
    latch = EdgeToggle()
    assert not latch.state
    assert latch.update(True) and latch.state        # press: latched
    assert not latch.update(True) and latch.state    # held: no change
    assert not latch.update(False) and latch.state   # release: still latched
    assert latch.update(True) and not latch.state    # press again: released
    assert not latch.update(False) and not latch.state


def buttons(*pressed, n=13):
    """Button array with the given indices pressed."""
    return [1 if i in pressed else 0 for i in range(n)]


def test_arm_needs_full_hold():
    combo = ArmCombo([4, 9], 1.0, 8)
    assert combo.buttons_used() == 10
    assert combo.update(buttons(4, 9), 0.0) is None
    assert combo.update(buttons(4, 9), 0.5) is None
    assert combo.update(buttons(4, 9), 1.0) == 'arm'
    assert combo.update(buttons(4, 9), 1.5) is None   # fires once per hold
    assert combo.update(buttons(), 2.0) is None
    assert combo.update(buttons(4, 9), 3.0) is None   # new hold restarts the timer
    assert combo.update(buttons(4, 9), 4.0) == 'arm'


def test_arm_resets_on_partial_release():
    combo = ArmCombo([4, 9], 1.0, 8)
    combo.update(buttons(4, 9), 0.0)
    combo.update(buttons(9), 0.6)                     # deadman let go
    assert combo.update(buttons(4, 9), 1.2) is None
    assert combo.update(buttons(4, 9), 2.2) == 'arm'


def test_disarm_on_press_and_blocks_arm():
    combo = ArmCombo([4, 9], 1.0, 8)
    assert combo.update(buttons(8), 0.0) == 'disarm'
    assert combo.update(buttons(8), 0.1) is None      # held: once
    assert combo.update(buttons(4, 8, 9), 0.2) is None
    assert combo.update(buttons(4, 8, 9), 2.0) is None  # disarm held: never arms
    assert combo.update(buttons(), 2.1) is None
    assert combo.update(buttons(4, 9, 8), 2.2) == 'disarm'
