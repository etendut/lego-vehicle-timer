# Timed Train, Servo Steer, Skid Steer, and ODV vehicle program for interactive displays
# Copyright Etendut
# https://etendut.github.io/lego-vehicle-timer/
# licence MIT
from micropython import const, mem_info
from pybricks.parameters import Color, Button
from pybricks.pupdevices import Remote
from pybricks.tools import wait, StopWatch

try:
    from typing import TYPE_CHECKING
except ImportError:
    TYPE_CHECKING = False

if TYPE_CHECKING:
    # noinspection PyUnusedImports
    from pybricks.hubs import CityHub, TechnicHub
    from pybricks.pupdevices import Remote

from pybricks.parameters import Port, Side, Direction
from pybricks.pupdevices import DCMotor
from uerrno import ENODEV



__BUILD__ = 'f8046b6'  # replaced at compile time with git hash + timestamp
print('Version 3.0.0 build', __BUILD__)
##################################################################################
#  Settings
##################################################################################

# ── user configuration ────────────────────────────────────────────────────────
COUNTDOWN_LIMIT_MINUTES: int = const(3)  # run for (x) minutes, min 1 minute, max up to you. the default of 3 minutes is play tested :).
COUNTDOWN_RESET_CODE         = 'c,c,c'  # c = center button, + = + button, - = - button

# ── battery / voltage ─────────────────────────────────────────────────────────
# fresh battery = 1.6V; 
# hub programming often fails below 9.3v (~1.5Vx6);
# hub fails when batteries reach ~1.36V so critical level = 1.4V * 6 cells = 8400mV
MILLIVOLT_CRITICAL_LEVEL = const(8400)

_REMOTE_DISABLED = False # ODV overrides this from DRIVE_MODE; all other vehicles (servo/train/skid_steer) leave it False
_AUTO_UNLOAD_ON_TIMER_END = False # ODV opts in via AUTO_UNLOAD_ON_TIMER_END; non-homing vehicles leave it False

# ── user configuration ────────────────────────────────────────────────────────
SKID_STEER_SPEED: int                  = const(80)  # set between 50 and 100
SKID_STEER_SWAP_MOTOR_SIDES: bool      = const(False)  # set to True if Left/Right remote buttons are backwards
SKID_STEER_REVERSE_LEFT_MOTOR: bool    = const(False)  # set to True if remote + button cause motor to run backwards
SKID_STEER_REVERSE_RIGHT_MOTOR: bool   = const(False)  # set to True if remote + button cause motor to run backwards



##################################################################################
# ---------Main program below, editing should not be needed -------------

class ErrorFlashCodes:
    def __init__(self):
        self.flash_count = 1  # Other errors

    def set_error_no_motor_on_a(self):
        print('ERROR: NO MOTOR ON A')
        self.flash_count = 2

    def set_error_no_motor_on_b(self):
        print('ERROR: NO MOTOR ON B')
        self.flash_count = 3

    def set_error_no_remote(self):
        print('ERROR: NO REMOTE')
        self.flash_count = 4

    def set_error_low_battery(self):
        print('ERROR: LOW BATTERY')
        self.flash_count = 5

    def flash_error_code(self):
        """
            this flashes the remote led
        """

        global hub
        global remote

        for f in range(self.flash_count):
            hub.light.on(Color.RED)
            if not _REMOTE_DISABLED:
                remote.light.on(Color.RED)
            wait(350)
            hub.light.on(Color.NONE)
            if not _REMOTE_DISABLED:
                remote.light.on(Color.NONE)
            wait(350)
        if self.flash_count > 1:
            wait(2000)


class MotorHelper:

    def __init__(self, supports_flip: bool, supports_homing: bool):
        self.mh_supports_flip = supports_flip
        self.mh_supports_homing = supports_homing
        self.mh__remote_disabled = False
        self.mh_auto_drive = False
        self.mh_is_homed = False

    def handle_flip(self):
        """Tracked racer only"""
        pass

    def home_and_unload(self):
        """ODV only"""
        pass

    def park_at_unload(self):
        """ODV only — timer-end auto-park at the unload tile."""
        pass

    def reset_homing(self):
        """ODV only"""
        pass

    def auto_unload(self):
        """ODV only"""
        pass

    def auto_load(self):
        """ODV only"""
        pass

    def enable_auto_drive(self):
        """enable mh_auto_drive, ODV only"""
        if self.mh_auto_drive:
            return
        print("Enable Auto-Drive")
        self.mh_auto_drive = True

    def disable_auto_drive(self):
        """Disables mh_auto_drive, ODV only"""
        if not self.mh_auto_drive:
            return
        print("Disable Auto-Drive")
        self.mh_auto_drive = False

    def set_is_homed(self):
        """Set mh_is_homed, ODV only"""
        if self.mh_is_homed:
            return
        print("Set IsHomed")
        self.mh_is_homed = True

    def reset_is_homed(self):
        """Reset mh_is_homed, ODV only"""
        if not self.mh_is_homed:
            return
        print("Reset IsHomed")
        self.mh_is_homed = False

    def handle_remote_press(self):
        """All vehicles"""
        pass

    def stop_motors(self):
        """All vehicles"""
        pass

    def idle_timed_out(self) -> bool:
        """ODV only — returns True when auto-drive should engage."""
        return False

    def reset_idle_timeout(self):
        """ODV only — call after any activity that should delay auto-drive."""
        pass


##################################################################################
# Countdown helper
##################################################################################

def wait_for_no_pressed_buttons():
    if _REMOTE_DISABLED:
        return
    remote_buttons_pressed = remote.buttons.pressed()
    while remote_buttons_pressed:
        wait(100)
        remote_buttons_pressed = remote.buttons.pressed()


def convert_millis_hours_minutes_seconds(millis: int):
    """
        utility for making milliseconds hours/minutes/seconds
    :param millis:
    :return hours, minutes, seconds:
    """
    hours = int((millis / (1000 * 60 * 60)) % 24)
    minutes = int((millis / (1000 * 60)) % 60)
    seconds = int((millis / 1000) % 60)

    return hours, minutes, seconds


_READY: int = const(0)
_ACTIVE: int = const(10)
_FINAL_MINUTE: int = const(20)
_FINAL_20_SECS: int = const(30)
_ENDED: int = const(40)
_UNKNOWN: int = const(99)


class CountdownTimer:
    """
    This allows the model to run for a set time
    """

    def __init__(self):
        # assign external objects to properties of the class
        self.last_hub_remote_color = None
        self.last_countdown_message: str = ''
        self.countdown_status: int = _UNKNOWN
        self._debug_last_logged_status: int = -1  # sentinel so first show_status logs

        # Start a timer.
        self.stopwatch = StopWatch()
        self.led_flash_sw_time = 0
        self.last_flash_color = None
        self.end_time = 0
        # battery check throttle
        self._battery_check_time = 0

    def has_time_remaining(self):
        """
            Checks if countdown has time remaining
        :return:
        """
        if self.countdown_status == _ENDED or self.countdown_status == _READY:
            return False

        # calculate remaining_time time
        remaining_time = self.end_time - self.stopwatch.time()

        # print a friendly console message
        con_hour, con_min, con_sec = convert_millis_hours_minutes_seconds(int(remaining_time))

        if con_sec % 10 == 0 and con_min < 1:
            countdown_message = 'countdown ending in: {}:{:02}'.format(con_min, con_sec)
            if self.last_countdown_message != countdown_message:
                self.last_countdown_message = countdown_message
                print(self.last_countdown_message)  # when time has run out end countdown
        if remaining_time <= 0:
            self.countdown_status = _ENDED
            self.show_status()
            return False
        # in last 20s fast flash a warning
        if remaining_time < (1000 * 20):
            self.countdown_status = _FINAL_20_SECS
            self.show_status()
        # in last minute slow flash a warning
        elif remaining_time < (1000 * 60):
            self.countdown_status = _FINAL_MINUTE
            self.show_status()

        return True

    def __start_countdown__(self):
        """
            start the countdown sequence by resetting timers and status
        """
        print('start countdown')
        self.countdown_status = _ACTIVE
        self.show_status()
        self.end_time = self.stopwatch.time() + (COUNTDOWN_LIMIT_MINUTES * 60 * 1000)

    def reset(self):
        if _REMOTE_DISABLED:
            print('countdown time reset')
        else:
            print('countdown time reset, press Remote CENTER to restart countdown')
        self.countdown_status = _READY

    def check_remote_buttons(self) -> bool:
        """
        Check countdown time buttons.
        Returns True if the reset code was pressed (caller should reset motor state).
        """
        global remote
        if _REMOTE_DISABLED:
            return False

        remote_buttons_pressed = remote.buttons.pressed()
        if len(remote_buttons_pressed) == 0:
            return False

        if self.countdown_status == _READY and Button.CENTER in remote_buttons_pressed:
            self.__start_countdown__()
            wait_for_no_pressed_buttons()

        # if reset sequence pressed reset the countdown timer
        if all(i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_PRESSED) and not any(
                i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_NOT_PRESSED):
            self.reset()
            wait_for_no_pressed_buttons()
            return True

        return False

    def should_check_battery(self) -> bool:
        if self.stopwatch.time() > self._battery_check_time:
            self._battery_check_time = self.stopwatch.time() + 5000
            return True
        return False

    def show_status(self):
        if self.countdown_status != self._debug_last_logged_status:
            print('SHOW_STATUS status=', self.countdown_status,
                  'last_color=', self.last_hub_remote_color)
            self._debug_last_logged_status = self.countdown_status
        if self.countdown_status == _READY:
            self.flash_hub_and_remote_light(Color.GREEN, 500, Color.NONE, 500, True)
        elif self.countdown_status == _ACTIVE:
            self.set_hub_and_remote_light(Color.GREEN, True)
        elif self.countdown_status == _FINAL_20_SECS:
            self.flash_hub_and_remote_light(Color.ORANGE, 200, Color.NONE, 100, True)
        elif self.countdown_status == _FINAL_MINUTE:
            self.flash_hub_and_remote_light(Color.ORANGE, 500, Color.NONE, 250, True)
        elif self.countdown_status == _ENDED:
            self.set_hub_and_remote_light(Color.ORANGE, True)

    def set_hub_and_remote_light(self, on_color:Color, include_remote:bool):
        """
        Set remote and hub light color if changed
        :param include_remote:
        :param on_color:
        :return:
        """
        global hub
        global remote

        # only set color if it's changed
        if on_color == self.last_hub_remote_color:
            return
        self.last_hub_remote_color = on_color

        hub.light.on(on_color)

        if include_remote and not _REMOTE_DISABLED:
            remote.light.on(on_color)

    def flash_hub_and_remote_light(self, on_color:Color, on_msec: int, off_color, off_msec: int, include_remote:bool):
        """
            this flashes the hub (and optionally remote) led
        :param include_remote:
        :param on_color:
        :param on_msec:
        :param off_color:
        :param off_msec:
        """
        # we use a timer to make it a non-blocking call
        now = self.stopwatch.time()
        if now > (on_msec + off_msec + self.led_flash_sw_time):
            self.led_flash_sw_time = now
            self.set_hub_and_remote_light(off_color, include_remote)
        elif now > (off_msec + self.led_flash_sw_time):
            self.set_hub_and_remote_light(on_color, include_remote)


##################################################################################
# Code helper
##################################################################################

def code_to_button_press_hash(button_code):
    """
    Returns the button needed to match a given code
    :param button_code:
    :return buttons_that_are_pressed[], buttons_that_should_not_pressed[]:
    """
    code_items = button_code.split(',')
    buttons_that_should_not_pressed = [Button.LEFT_PLUS, Button.LEFT_MINUS, Button.LEFT, Button.CENTER,
                                       Button.RIGHT_PLUS, Button.RIGHT_MINUS, Button.RIGHT]
    buttons_that_are_pressed = []
    # left code
    if '+' in code_items[0]:
        buttons_that_are_pressed.append(Button.LEFT_PLUS)
        buttons_that_should_not_pressed.remove(Button.LEFT_PLUS)
    if '-' in code_items[0]:
        buttons_that_are_pressed.append(Button.LEFT_MINUS)
        buttons_that_should_not_pressed.remove(Button.LEFT_MINUS)
    if 'c' in code_items[0]:
        buttons_that_are_pressed.append(Button.LEFT)
        buttons_that_should_not_pressed.remove(Button.LEFT)

    # middle code
    if 'c' in code_items[1]:
        buttons_that_are_pressed.append(Button.CENTER)
        buttons_that_should_not_pressed.remove(Button.CENTER)

    # right code
    if '+' in code_items[2]:
        buttons_that_are_pressed.append(Button.RIGHT_PLUS)
        buttons_that_should_not_pressed.remove(Button.RIGHT_PLUS)
    if '-' in code_items[2]:
        buttons_that_are_pressed.append(Button.RIGHT_MINUS)
        buttons_that_should_not_pressed.remove(Button.RIGHT_MINUS)
    if 'c' in code_items[2]:
        buttons_that_are_pressed.append(Button.RIGHT)
        buttons_that_should_not_pressed.remove(Button.RIGHT)

    return buttons_that_are_pressed, buttons_that_should_not_pressed


PROGRAM_RESET_CODE_PRESSED, PROGRAM_RESET_CODE_NOT_PRESSED = code_to_button_press_hash(COUNTDOWN_RESET_CODE)

##################################################################################
# Main program
##################################################################################


hub: "CityHub | TechnicHub"
remote: "Remote" = None  # type: ignore  # bound by setup_remote(); stays None in headless modes


def setup_hub():
    global hub

    try:
        # this import will fail if the city hub is not connected.
        from pybricks.hubs import CityHub
        hub = CityHub()
        print('Lego City Hub found')
        return False
    except ImportError as ex1:
        print(ex1)
        try:
            from pybricks.hubs import TechnicHub
            hub = TechnicHub()
            print('Lego Technic Hub found')
            return True

        except ImportError as ex2:
            print(ex2)
            raise Exception('This program only support Lego City hub and Lego Technic hub')

def hub_battery_ok()->bool:
    global hub
    mv_voltage = hub.battery.voltage()
    return mv_voltage > MILLIVOLT_CRITICAL_LEVEL

LED_FLASHING_SEQUENCE = [75] * 5 + [1000]


def setup_remote(error_flash_code_helper, retry=5):
    global hub
    global remote

    # Flashing led while waiting connection
    hub.light.blink(Color.WHITE, LED_FLASHING_SEQUENCE)

    # try to connect to remote multiple times
    remote_retry_count = 1

    while True:
        # noinspection PyBroadException
        try:
            print("--looking for remote try " + str(remote_retry_count))
            # Connect to the remote
            remote = Remote()
            print("--remote connected.")
            break
        except:
            if remote_retry_count >= retry:
                error_flash_code_helper.set_error_no_remote()
                raise  # ignore first 20 errors
        remote_retry_count += 1
        wait(50)


##################################################################################
# Skid steer helper
##################################################################################

class RunSkidSteerMotors(MotorHelper):
    """
        Handles driving a skid steer model and reverses control when it flips over
    """

    def __init__(self, error_flash_code_helper: ErrorFlashCodes, drive_speed: int, swap_motor_sides: bool,
                 reverse_left_motor: bool, reverse_right_motor: bool):

        super().__init__(True, False)

        self.error_flash_code = error_flash_code_helper
        if swap_motor_sides:
            self.left_motor_port = Port.B
            self.right_motor_port = Port.A
        else:
            self.left_motor_port = Port.A
            self.right_motor_port = Port.B

        if reverse_left_motor:
            self.left_motor_direction = Direction.CLOCKWISE
        else:
            self.left_motor_direction = Direction.COUNTERCLOCKWISE

        if reverse_right_motor:
            self.right_motor_direction = Direction.COUNTERCLOCKWISE
        else:
            self.right_motor_direction = Direction.CLOCKWISE

        self.drive_speed = drive_speed
        self.last_side = None
        try:
            self.left_motor = DCMotor(self.left_motor_port, positive_direction=self.left_motor_direction)
        except OSError as ex:
            if ex.errno == ENODEV:
                print('Motor needs to be connected to ' + str(self.left_motor_port))
                self.error_flash_code.set_error_no_motor_on_a()
            raise
        try:
            self.right_motor = DCMotor(self.right_motor_port, positive_direction=self.right_motor_direction)
        except OSError as ex:
            if ex.errno == ENODEV:
                print('Motor needs to be connected to ' + str(self.right_motor_port))
                self.error_flash_code.set_error_no_motor_on_b()
            raise

        self.stop_motors()

    def handle_flip(self):
        """
            swap the motors so that the left and right controls are the same when it flips
        :return:
        """
        global hub
        # Check which side of the hub is up.
        assert hub is not None
        up_side = hub.imu.up()

        # if the hub hasn't flipped ignore the rest of the logic
        if self.last_side == up_side:
            return

        self.last_side = up_side
        # normal side up
        if up_side == Side.TOP:
            print('--Top Up')
            self.right_motor = DCMotor(self.right_motor_port, positive_direction=self.right_motor_direction)
            self.left_motor = DCMotor(self.left_motor_port, positive_direction=self.left_motor_direction)
        # upside down
        if up_side == Side.BOTTOM:
            print('--Bottom Up')
            self.right_motor = DCMotor(self.left_motor_port, positive_direction=self.right_motor_direction)
            self.left_motor = DCMotor(self.right_motor_port, positive_direction=self.left_motor_direction)

    def handle_remote_press(self):
        """
            handle remote button clicks
        """
        if self.mh__remote_disabled:
            return
        # Check which remote_buttons are pressed.
        assert remote is not None
        remote_buttons_pressed = remote.buttons.pressed()
        if len(remote_buttons_pressed) == 0 or Button.RIGHT in remote_buttons_pressed or Button.LEFT in remote_buttons_pressed:
            self.stop_motors()
            return
        # stop motors as this is bang-bang mode where a button
        #  needs to be held down for racer to run
        self.stop_motors()

        #  handle button press
        if Button.LEFT_PLUS in remote_buttons_pressed:
            self.left_motor.dc(self.drive_speed)

        if Button.LEFT_MINUS in remote_buttons_pressed:
            self.left_motor.dc(-self.drive_speed)

        if Button.RIGHT_PLUS in remote_buttons_pressed:
            self.right_motor.dc(self.drive_speed)

        if Button.RIGHT_MINUS in remote_buttons_pressed:
            self.right_motor.dc(-self.drive_speed)

    # stop all motors
    def stop_motors(self):
        self.left_motor.dc(0)
        self.right_motor.dc(0)




def main():
    error_flash_code = ErrorFlashCodes()
    print('SETUP')
    print('--setup hub')
    setup_hub()
    print('voltage',hub.battery.voltage())
    try:
        print("--setup countdown")
        countdown_timer = CountdownTimer()
        print("--setup motors")
        drive_motors = RunSkidSteerMotors(error_flash_code, SKID_STEER_SPEED, SKID_STEER_SWAP_MOTOR_SIDES,
                                  SKID_STEER_REVERSE_LEFT_MOTOR, SKID_STEER_REVERSE_RIGHT_MOTOR)

        drive_motors.mh__remote_disabled = _REMOTE_DISABLED

        if _REMOTE_DISABLED:
            print('--no remote')
        else:
            print('--setup remote')
            setup_remote(error_flash_code)

        # give everything a chance to warm up
        wait(500)

        print('SETUP complete')

        countdown_timer.reset()
        mem_info()
        # one-shot guards so debug prints fire on each transition, not every tick
        _main_gate_closed_logged = [False]
        while True:
            if countdown_timer.should_check_battery() and not hub_battery_ok():
                error_flash_code.set_error_low_battery()
                raise Exception('low battery')

            if not _REMOTE_DISABLED:
                if countdown_timer.check_remote_buttons():
                    drive_motors.stop_motors()
                    if drive_motors.mh_supports_homing:
                        drive_motors.reset_homing()

            if drive_motors.mh_supports_homing:
                if not drive_motors.mh_auto_drive:
                    # Full auto: enable immediately once homed, no remote needed
                    if _REMOTE_DISABLED and drive_motors.mh_is_homed:
                        drive_motors.enable_auto_drive()
                    # Hybrid: enable after idle timeout
                    elif drive_motors.idle_timed_out():
                        drive_motors.enable_auto_drive()

                if drive_motors.mh_auto_drive and drive_motors.mh_is_homed:
                    drive_motors.auto_unload()
                    drive_motors.auto_load()
                    # Hybrid: if a button press interrupted auto, reset the idle timer so auto
                    # doesn't re-enable immediately on the next loop iteration
                    if not drive_motors.mh_auto_drive:
                        countdown_timer.__start_countdown__()
                        drive_motors.reset_idle_timeout()

            # if there is no remote, then there is no point in a countdown
            if countdown_timer.has_time_remaining() or _REMOTE_DISABLED or drive_motors.mh_auto_drive:
                _main_gate_closed_logged[0] = False  # arm the print for the next close
                if drive_motors.mh_supports_homing and not drive_motors.mh_is_homed:
                    drive_motors.home_and_unload()
                if drive_motors.mh_supports_flip:
                    drive_motors.handle_flip()
                if not _REMOTE_DISABLED:
                    drive_motors.handle_remote_press()
            else:
                if not _main_gate_closed_logged[0]:
                    print('TIMER ENDED -- gate closed, handle_remote_press NOT called',
                          'status=', countdown_timer.countdown_status,
                          'auto_drive=', drive_motors.mh_auto_drive)
                    _main_gate_closed_logged[0] = True
                    # opt-in: when the countdown ran out naturally (not a user
                    # reset), park the cart at U so the rig is ready to resume
                    # on the next countdown start. ODV's park_at_unload uses
                    # the planner when the encoder is calibrated (respects
                    # one-way barriers) and falls back to home_and_unload's
                    # stall routine otherwise.
                    if (_AUTO_UNLOAD_ON_TIMER_END
                            and drive_motors.mh_supports_homing
                            and countdown_timer.countdown_status == _ENDED):
                        drive_motors.park_at_unload()
                drive_motors.stop_motors()
                if drive_motors.mh_supports_homing:
                    drive_motors.reset_homing()

            countdown_timer.show_status()
            # add a small delay to keep the loop stable and allow for events to occur
            wait(10)

            if _REMOTE_DISABLED and not drive_motors.mh_supports_homing:
                print("No remote exiting")
                raise SystemExit

    except Exception as e:
        print(e)
        while True:
            error_flash_code.flash_error_code()


if __name__ == "__main__":
    main()
