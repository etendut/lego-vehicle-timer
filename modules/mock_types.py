from pybricks.parameters import Side


class MockLight:
    """For typing only, this will be replaced by CityHub or TechnicHub"""

    def blink(self, col, seq):
        pass

    def on(self, color):
        pass


class MockIMU:
    """For typing only, this will be replaced by TechnicHub"""

    def up(self) -> Side:
        pass


class MockHub:
    """For typing only, this will be replaced by CityHub or TechnicHub"""

    def __init__(self):
        self.imu = MockIMU()
        self.light = MockLight()

class MockRemoteButtons:
    """For typing only, this will be replaced by RemoteButtons"""
    def pressed(self)->list:
        print(self.__class__.__name__)
        return []

class MockRemote:
    """For typing only, this will be replaced by RemoteButtons"""
    def __init__(self):
        self.buttons = MockRemoteButtons()
        self.light = MockLight()