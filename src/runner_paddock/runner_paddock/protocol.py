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

"""Versioned, strict JSON encoding for the read-only Paddock stream."""

import json
import math
from typing import Any


PROTOCOL_VERSION = 1


class ValidatedGridData(list):
    """JSON-ready grid cells validated once at the ROS ingress boundary."""

    def __init__(self, values: Any) -> None:
        super().__init__()
        for value in values:
            if type(value) is not int:
                raise TypeError('OccupancyGrid data contains a non-integer')
            if value < -1 or value > 100:
                raise ValueError('OccupancyGrid data is outside [-1, 100]')
            list.append(self, value)

    def _immutable(self, *_args: Any, **_kwargs: Any) -> None:
        raise TypeError('validated OccupancyGrid data is immutable')

    __delitem__ = _immutable
    __iadd__ = _immutable
    __imul__ = _immutable
    __setitem__ = _immutable
    append = _immutable
    clear = _immutable
    extend = _immutable
    insert = _immutable
    pop = _immutable
    remove = _immutable
    reverse = _immutable
    sort = _immutable


def _validate_json(value: Any, path: str = '$') -> None:
    if value is None or isinstance(value, (bool, str, int)):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f'non-finite number at {path}')
        return
    if isinstance(value, ValidatedGridData):
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _validate_json(item, f'{path}[{index}]')
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise TypeError(f'non-string object key at {path}')
            _validate_json(item, f'{path}.{key}')
        return
    raise TypeError(f'unsupported JSON value at {path}: {type(value).__name__}')


def encode_message(message_type: str, **fields: Any) -> str:
    """Encode one protocol frame, rejecting lossy or non-finite values."""
    message = {
        'protocol_version': PROTOCOL_VERSION,
        'type': message_type,
        **fields,
    }
    _validate_json(message)
    return json.dumps(
        message,
        allow_nan=False,
        check_circular=True,
        ensure_ascii=False,
        separators=(',', ':'),
    )
