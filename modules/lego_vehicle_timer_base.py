# Timed train and vehicle program for interactive displays
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

# IMPORT_SECTION

print('Version 2.3.0')
##################################################################################
#  Settings
##################################################################################

# countdown time settings
COUNTDOWN_LIMIT_MINUTES: int = const(
    3)  # run for (x) minutes, min 1 minute, max up to you. the default of 3 minutes is play tested :).
# c = center button, + = + button, - = - button
COUNTDOWN_RESET_CODE = 'c,c,c'  # left center button, center button, right center button

# How many seconds to wait before doing a load/unload automatically. 0 = disabled
ODV_AUTO_DRIVE_TIMEOUT_SECS: int = const(0)

# for debugging or ODV full auto
REMOTE_DISABLED = False

# low voltage protection in millivolts e.g. 1.2 * 6 * 1000 = 7200mV

# battery notes
# - fresh battery = 1.6V
# - hub programming often fails below 1.5v
# - TODO critical level may be too low
MILLIVOLT_CRITICAL_LEVEL = const(7200) 

# VARS_SECTION

##################################################################################
# ---------Main program below, editing should not be needed -------------

# {Insert drive module here}

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
            if not REMOTE_DISABLED:
                remote.light.on(Color.RED)
            wait(350)
            hub.light.on(Color.NONE)
            if not REMOTE_DISABLED:
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


##################################################################################
# Countdown helper
##################################################################################

def wait_for_no_pressed_buttons():
    if REMOTE_DISABLED:
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

        # Start a timer.
        self.stopwatch = StopWatch()
        self.led_flash_sw_time = 0
        self.last_flash_color = None
        self.end_time = 0
        # remote timing
        self.remote_buttons_time_out_ms = 0
        self.reset_time_since_last_remote_press()
        # battery check throttle
        self._battery_check_time = 0

    def reset_time_since_last_remote_press(self):
        self.remote_buttons_time_out_ms = self.stopwatch.time() + (ODV_AUTO_DRIVE_TIMEOUT_SECS * 1000)

    def remote_button_press_timed_out(self) -> bool:
        return self.stopwatch.time() > self.remote_buttons_time_out_ms

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
        # in last 25s slow flash a warning
        if remaining_time < (1000 * 20):
            self.countdown_status = _FINAL_20_SECS
            self.show_status()

            # in last minute slow flash a warning
        if remaining_time < (1000 * 60):
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
        if REMOTE_DISABLED:
            print('countdown time reset')
        else:
            print('countdown time reset, press Remote CENTER to restart countdown')
        self.countdown_status = _READY
        self.reset_time_since_last_remote_press()

    def check_remote_buttons(self):
        """
            check countdown time buttons
        """
        if REMOTE_DISABLED:
            return

        remote_buttons_pressed = remote.buttons.pressed()
        if len(remote_buttons_pressed) == 0:
            return

        self.reset_time_since_last_remote_press()

        if self.countdown_status == _READY and Button.CENTER in remote_buttons_pressed:
            self.__start_countdown__()
            wait_for_no_pressed_buttons()

        # if reset sequence pressed reset the countdown timer
        if all(i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_PRESSED) and not any(
                i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_NOT_PRESSED):
            self.reset()
            wait_for_no_pressed_buttons()

    def should_check_battery(self) -> bool:
        if self.stopwatch.time() > self._battery_check_time:
            self._battery_check_time = self.stopwatch.time() + 5000
            return True
        return False

    def show_status(self):
        if self.countdown_status == _READY:
            self.flash_hub_and_remote_light(Color.GREEN, 500, Color.NONE, 500, False)
        elif self.countdown_status == _ACTIVE:
            self.set_hub_and_remote_light(Color.GREEN, True)
        elif self.countdown_status == _FINAL_20_SECS:
            self.flash_hub_and_remote_light(Color.ORANGE, 200, Color.NONE, 100, False)
        elif self.countdown_status == _FINAL_MINUTE:
            self.flash_hub_and_remote_light(Color.ORANGE, 500, Color.NONE, 250, False)
        elif self.countdown_status == _ENDED:
            self.set_hub_and_remote_light(Color.ORANGE, False)

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

        if include_remote and not REMOTE_DISABLED:
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
remote: "Remote"


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


# VEHICLE_SECTION

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
        drive_motors = MotorHelper(False, False)
        drive_motors.mh__remote_disabled = REMOTE_DISABLED

        if REMOTE_DISABLED:
            print('--no remote')
        else:
            print('--setup remote')
            setup_remote(error_flash_code)

        # give everything a chance to warm up
        wait(500)

        print('SETUP complete')

        countdown_timer.reset()
        mem_info()
        while True:
            if countdown_timer.should_check_battery() and not hub_battery_ok():
                error_flash_code.set_error_low_battery()
                break

            if not REMOTE_DISABLED:
                countdown_timer.check_remote_buttons()

            if drive_motors.mh_supports_homing:
                if not drive_motors.mh_auto_drive:
                    # Full auto: enable immediately once homed, no remote needed
                    if REMOTE_DISABLED and drive_motors.mh_is_homed:
                        drive_motors.enable_auto_drive()
                    # Hybrid: enable after timeout with no remote activity
                    elif ODV_AUTO_DRIVE_TIMEOUT_SECS > 0 and countdown_timer.remote_button_press_timed_out():
                        drive_motors.enable_auto_drive()

                if drive_motors.mh_auto_drive and drive_motors.mh_is_homed:
                    drive_motors.auto_unload()
                    drive_motors.auto_load()
                    # Hybrid: if a button press interrupted auto, reset the idle timer so auto
                    # doesn't re-enable immediately on the next loop iteration
                    if not drive_motors.mh_auto_drive:
                        countdown_timer.reset_time_since_last_remote_press()

            # if there is no remote, then there is no point in a countdown
            if countdown_timer.has_time_remaining() or REMOTE_DISABLED:
                if drive_motors.mh_supports_homing and not drive_motors.mh_is_homed:
                    drive_motors.home_and_unload()
                if drive_motors.mh_supports_flip:
                    drive_motors.handle_flip()
                if not REMOTE_DISABLED:
                    drive_motors.handle_remote_press()
            else:
                drive_motors.stop_motors()
                if drive_motors.mh_supports_homing:
                #     drive_motors.auto_unload()
                    drive_motors.reset_homing()

            countdown_timer.show_status()
            # add a small delay to keep the loop stable and allow for events to occur
            wait(10)

            if REMOTE_DISABLED and not drive_motors.mh_supports_homing:
                print("No remote exiting")
                raise SystemExit

    except Exception as e:
        print(e)
        while True:
            error_flash_code.flash_error_code()


if __name__ == "__main__":
    main()
