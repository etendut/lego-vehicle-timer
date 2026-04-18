"""Smoke tests for the compiled ``docs/pybricks/lego_vehicle_timer_*.py`` files.

Each compiled file is loaded directly from disk (not via the import system) and
its top-level ``Run*Motors`` class is checked for the public methods the base
runtime calls. This catches splice bugs that only surface after concatenation.
"""
import importlib.util
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).parent.parent.resolve()
COMPILED_DIR = PROJECT_ROOT / 'docs' / 'pybricks'

REQUIRED_METHODS = (
    'stop_motors',
    'handle_remote_press',
    'reset_homing',
    'handle_flip',
)

VEHICLES = [
    ('odv',        'RunODVMotors'),
    ('servo',      'RunServoSteerMotors'),
    ('skid_steer', 'RunSkidSteerMotors'),
    ('train',      'RunTrainMotor'),
]


def _load_compiled(vehicle: str):
    path = COMPILED_DIR / f'lego_vehicle_timer_{vehicle}.py'
    spec = importlib.util.spec_from_file_location(f'_compiled_{vehicle}', path)
    assert spec and spec.loader, f'cannot build import spec for {path}'
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('vehicle,class_name', VEHICLES)
def test_compiled_file_exposes_run_motors_class(vehicle, class_name):
    module = _load_compiled(vehicle)
    cls = getattr(module, class_name, None)
    assert cls is not None, f'{class_name} missing from compiled {vehicle} file'
    for method in REQUIRED_METHODS:
        assert callable(getattr(cls, method, None)), (
            f'{class_name} is missing required method {method}'
        )


@pytest.mark.parametrize('vehicle,_class_name', VEHICLES)
def test_compiled_file_defines_main(vehicle, _class_name):
    module = _load_compiled(vehicle)
    assert callable(getattr(module, 'main', None)), (
        f'compiled {vehicle} file is missing top-level main()'
    )
