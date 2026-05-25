import micropython
import uerrno
from pybricks.pupdevices import Motor

# uerrno is a stub package that doesn't include errno constants — add the ones we need
uerrno.ENODEV = 19

# Motor.run_until_stalled returns None without hardware, which breaks calibrate_steering math
Motor.run_until_stalled = lambda self, *args, **kwargs: 0

# micropython.const() returns None in the pybricks stub — make it a pass-through so
# compiled files (which lack the source-level mock_const fallback) can be imported.
micropython.const = lambda value: value
