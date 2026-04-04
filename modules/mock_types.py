from typing import Protocol
from pybricks.parameters import Side, Color


class MockLight(Protocol):
    def blink(self, col: list, seq: list) -> None: ...
    def on(self, color: Color) -> None: ...


class MockIMU(Protocol):
    def up(self) -> Side: ...


class MockBattery(Protocol):
    def voltage(self) -> int: ...


class MockHub(Protocol):
    battery: MockBattery
    imu: MockIMU
    light: MockLight


class MockRemoteButtons(Protocol):
    def pressed(self) -> list: ...


class MockRemote(Protocol):
    buttons: MockRemoteButtons
    light: MockLight
