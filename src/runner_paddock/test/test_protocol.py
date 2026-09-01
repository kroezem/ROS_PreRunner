# Copyright 2026 matti
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

"""Tests for strict versioned JSON frames."""

import json
import math

import pytest

from runner_paddock.protocol import encode_message


def test_frame_has_protocol_version_and_type():
    frame = json.loads(encode_message('state', pose=None))
    assert frame == {
        'protocol_version': 1,
        'type': 'state',
        'pose': None,
    }


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_non_finite_values_are_rejected(value):
    with pytest.raises(ValueError, match='non-finite'):
        encode_message('state', nested={'value': value})


def test_non_json_values_and_keys_are_rejected():
    with pytest.raises(TypeError, match='unsupported JSON value'):
        encode_message('state', value=(1, 2))
    with pytest.raises(TypeError, match='non-string object key'):
        encode_message('state', value={1: 'not allowed'})
