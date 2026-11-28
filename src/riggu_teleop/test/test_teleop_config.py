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

"""Safety semantics in config/ (riggu_teleop/CLAUDE.md): don't change without the user."""

from pathlib import Path

import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config'


def params(name, node):
    """ros__parameters of `node` in config/<name>."""
    with open(CONFIG / name, encoding='utf-8') as f:
        return yaml.safe_load(f)[node]['ros__parameters']


def test_mux_priorities_and_lock():
    mux = params('twist_mux.yaml', 'twist_mux')
    topics, locks = mux['topics'], mux['locks']
    assert topics['joy']['topic'] == 'cmd_vel_joy' and topics['joy']['priority'] == 100
    assert topics['nav']['topic'] == 'cmd_vel_nav' and topics['nav']['priority'] == 10
    assert locks['estop']['topic'] == 'estop' and locks['estop']['priority'] == 255
    # heartbeat lock: a dead stop_latch engages it
    stop_rate = params('joy_generic.yaml', 'stop_latch')['rate']
    assert locks['estop']['timeout'] > 2.0 / stop_rate
    # joy input times out before the controller's cmd_vel_timeout (0.5 s)
    assert 0.0 < topics['joy']['timeout'] < 0.5


def test_deadman_required():
    joy = params('joy_generic.yaml', 'teleop_twist_joy_node')
    assert joy['require_enable_button'] is True
    assert joy['publish_stamped_twist'] is True
    assert joy['frame'] == 'base_footprint'


def test_arm_combo_includes_deadman():
    deadman = params('joy_generic.yaml', 'teleop_twist_joy_node')['enable_button']
    arm = params('joy_generic.yaml', 'arm_switch')
    assert deadman in arm['arm_buttons'] and len(arm['arm_buttons']) >= 2
    assert arm['disarm_button'] not in arm['arm_buttons']
    assert arm['arm_hold'] >= 0.5
    stop = params('joy_generic.yaml', 'stop_latch')['button']
    assert stop not in arm['arm_buttons'] + [arm['disarm_button'], deadman]
