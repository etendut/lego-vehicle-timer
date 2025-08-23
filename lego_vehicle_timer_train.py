# Timed train and vehicle program for interactive displays
# Copyright Etendut
# licence MIT

import sys
from micropython import const
from pybricks.parameters import Color, Side, Button
from pybricks.pupdevices import Remote
from pybricks.tools import wait, StopWatch

from pybricks.parameters import Port, Direction
from pybricks.pupdevices import DCMotor, Light



print('Version 2.1.0')
##################################################################################
#  Settings
##################################################################################

# countdown time settings
COUNTDOWN_LIMIT_MINUTES: int = const(
    3)  # run for (x) minutes, min 1 minute, max up to you. the default of 3 minutes is play tested :).
# c = center button, + = + button, - = - button
COUNTDOWN_RESET_CODE = 'c,c,c'  # left center button, center button, right center button

#How many seconds to wait before doing a load/unload automatically. 0 = disabled
ODV_AUTO_DRIVE_TIMEOUT_SECS: int = const(0)

# for debugging or ODV full auto
REMOTE_DISABLED=False

# Train mode settings
TRAIN_MOTOR_SPEED_STEP: int = const(10)  # the amount each button press changes the train speed
TRAIN_MOTOR_MIN_SPEED: int = const(30)  # lowest speed the train will go set between 30 and 100
TRAIN_MOTOR_MAX_SPEED: int = const(80)  # set between 80 and 100
TRAIN_REVERSE_MOTOR_1: bool = False  # set to True if remote + button cause motor to run backwards
TRAIN_REVERSE_MOTOR_2: bool = True  # only used if a second train motor is on Port B




##################################################################################
# ---------Main program below, editing should not be needed -------------

# {Insert drive module here}

class ErrorFlashCodes:
    def __init__(self):
        self.flash_count = 1  # Other errors

    def set_error_no_motor_on_a(self):
        self.flash_count = 2

    def set_error_no_motor_on_b(self):
        self.flash_count = 3

    def set_error_no_remote(self):
        self.flash_count = 4

    def flash_error_code(self):
        """
            this flashes the remote led
        """

        global hub

        for f in range(self.flash_count):
            hub.light.on(Color.RED)
            wait(350)
            hub.light.on(Color.NONE)
            wait(350)
        if self.flash_count > 1:
            wait(2000)


class MotorHelper:

    def __init__(self, supports_flip: bool, supports_homing: bool):
        self.mh_supports_flip = supports_flip
        self.mh_supports_homing = supports_homing
        self.mh_auto_drive = False

    def handle_flip(self):
        """Tracked racer only"""
        pass

    def do_homing(self):
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

    def auto_home(self):
        """ODV only"""
        pass

    def enable_auto_drive(self):
        """ODV only"""
        print("Enable Auto-Drive")
        self.mh_auto_drive = True

    def disable_auto_drive(self):
        """ODV only"""
        print("Disable Auto-Drive")
        self.mh_auto_drive = False

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
    remote_buttons_pressed = remote.buttons.pressed()
    while remote_buttons_pressed:
        remote_buttons_pressed = remote.buttons.pressed()
        wait(100)


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


READY: int = const(0)
ACTIVE: int = const(10)
FINAL_MINUTE: int = const(20)
FINAL_20_SECS: int = const(30)
ENDED: int = const(40)
UNKNOWN: int = const(99)


class CountdownTimer:
    """
    This allows the model to run for a set time
    """

    def __init__(self):
        # assign external objects to properties of the class
        self.last_countdown_message:str = ''
        self.countdown_status:int = UNKNOWN

        # Start a timer.
        self.countdown_stopwatch = StopWatch()
        self.led_flash_stopwatch = StopWatch()
        self.remote_buttons_pressed_stopwatch = StopWatch()
        self.end_time = 0

    def has_time_remaining(self):
        """
            Checks if countdown has time remaining
        :return:
        """
        if self.countdown_status == ENDED or self.countdown_status == READY:
            return False

        # calculate remaining_time time
        remaining_time = self.end_time - self.countdown_stopwatch.time()

        # print a friendly console message
        con_hour, con_min, con_sec = convert_millis_hours_minutes_seconds(int(remaining_time))

        if con_sec % 10 == 0 and con_min < 1:
            countdown_message = 'countdown ending in: {}:{:02}'.format(con_min, con_sec)
            if self.last_countdown_message != countdown_message:
                self.last_countdown_message = countdown_message
                print(self.last_countdown_message)  # when time has run out end countdown
        if remaining_time <= 0:
            self.countdown_status = ENDED
            self.show_status()
            return False
        # in last 25s slow flash a warning
        if remaining_time < (1000 * 20):
            self.countdown_status = FINAL_20_SECS
            self.show_status()

            # in last minute slow flash a warning
        if remaining_time < (1000 * 60):
            self.countdown_status = FINAL_MINUTE
            self.show_status()

        return True

    def __start_countdown__(self):
        """
            start the countdown sequence by resetting timers and status
        """
        print('start countdown')
        self.countdown_status = ACTIVE
        self.show_status()
        self.countdown_stopwatch.reset()
        self.end_time = self.countdown_stopwatch.time() + (COUNTDOWN_LIMIT_MINUTES * 60 * 1000)

    def reset(self):
        if REMOTE_DISABLED:
            print('countdown time reset')
        else:
            print('countdown time reset, press Remote CENTER to restart countdown')
        self.countdown_status = READY

    def check_remote_buttons(self):
        """
            check countdown time buttons
        """
        remote_buttons_pressed = remote.buttons.pressed()
        if len(remote_buttons_pressed) == 0:
            return

        self.remote_buttons_pressed_stopwatch.reset()

        if self.countdown_status == READY and Button.CENTER in remote_buttons_pressed:
            self.__start_countdown__()
            wait_for_no_pressed_buttons()

        # if reset sequence pressed reset the countdown timer
        if all(i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_PRESSED) and not any(
                i in remote_buttons_pressed for i in PROGRAM_RESET_CODE_NOT_PRESSED):
            self.reset()
            wait_for_no_pressed_buttons()

    def show_status(self):
        global hub
        global remote
        if self.countdown_status == READY:
            self.__flash_remote_and_hub_light__(Color.GREEN, 500, Color.NONE, 500)
        elif self.countdown_status == ACTIVE:
            hub.light.on(Color.GREEN)
            remote.light.on(Color.GREEN)
        elif self.countdown_status == FINAL_20_SECS:
            self.__flash_remote_and_hub_light__(Color.ORANGE, 200, Color.NONE, 100)
        elif self.countdown_status == FINAL_MINUTE:
            self.__flash_remote_and_hub_light__(Color.ORANGE, 500, Color.NONE, 250)
        elif self.countdown_status == ENDED:
            hub.light.on(Color.ORANGE)
            remote.light.on(Color.ORANGE)


    def __flash_remote_and_hub_light__(self, on_color, on_msec: int, off_color, off_msec: int):
        """
            this flashes the remote led
        :param on_color:
        :param on_msec:
        :param off_color:
        :param off_msec:
        """
        global hub
        # we use a timer to make it a non-blocking call
        if self.led_flash_stopwatch.time() > (on_msec + off_msec):
            self.led_flash_stopwatch.reset()
            remote.light.on(off_color)
            hub.light.on(off_color)
        elif self.led_flash_stopwatch.time() > off_msec:
            remote.light.on(on_color)
            hub.light.on(on_color)


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


hub: MockHub = MockHub()
remote: MockRemote = MockRemote()

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
# Train motor helper
##################################################################################
class RunTrainMotor(MotorHelper):
    """
        Handles driving a train ensuring it doesn't run too fast
        Expects a train motor on Port A, and an optional train motor on Port B
    """

    def __init__(self, error_flash_code_helper: ErrorFlashCodes, min_speed: int, max_speed: int, speed_step: int,
                 reverse_motor: bool, reverse_motor_2: bool):

        super().__init__(False, False)
        self.error_flash_code = error_flash_code_helper
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.speed_step = speed_step
        self.current_motor_speed = 0
        motor_found = False
        self.train_motor_port_a = None
        self.train_motor_port_b = None
        # noinspection PyBroadException
        try:
            if reverse_motor:
                self.train_motor_port_a = DCMotor(Port.A, Direction.COUNTERCLOCKWISE)
            else:
                self.train_motor_port_a = DCMotor(Port.A, Direction.CLOCKWISE)
            print('Found train motor on ' + str(Port.A))
            motor_found = True
        except:
            pass
            self.train_motor_port_a = None  # ignore not found first motor

        if not motor_found:
            # noinspection PyBroadException
            try:
                if reverse_motor_2:
                    self.train_motor_port_b = DCMotor(Port.B, Direction.COUNTERCLOCKWISE)
                else:
                    self.train_motor_port_b = DCMotor(Port.B, Direction.CLOCKWISE)
                print('Found train motor on ' + str(Port.B))
                motor_found = True
            except:
                pass
                # ignore not found
                self.train_motor_port_b = None

        if not motor_found:
            self.error_flash_code.set_error_no_motor_on_a()
            raise Exception('Train motor needs to be connected to ' + str(Port.A) + ' or ' + str(Port.B))

        self.lights = None
        if self.train_motor_port_a is None:
            # noinspection PyBroadException
            try:
                self.lights = Light(Port.A)
                print('Found lights on ' + str(Port.B))
            except:
                pass
        if self.train_motor_port_b is None:
            # noinspection PyBroadException
            try:
                self.lights = Light(Port.B)
                print('Found lights on ' + str(Port.B))
            except:
                pass

    def handle_remote_press(self):
        """
            handle remote button clicks
        """
        # Check which remote_buttons are pressed.
        remote_buttons_pressed = remote.buttons.pressed()
        if len(remote_buttons_pressed) == 0 or Button.RIGHT in remote_buttons_pressed or Button.LEFT in remote_buttons_pressed:
            self.stop_motors()
            return
        # left remote_buttons
        # noinspection DuplicatedCode
        if Button.LEFT in remote_buttons_pressed or Button.RIGHT in remote_buttons_pressed:
            self.current_motor_speed = 0

        elif Button.LEFT_PLUS in remote_buttons_pressed or Button.RIGHT_PLUS in remote_buttons_pressed:

            if self.current_motor_speed == 0:  # if stopped go forward
                self.current_motor_speed = self.min_speed
            else:
                self.current_motor_speed += self.speed_step

            # if between 0 and TRAIN_MIN_SPEED snap to stopped
            if 0 > self.current_motor_speed > -self.min_speed:
                self.current_motor_speed = 0

            if self.current_motor_speed > self.max_speed:
                self.current_motor_speed = self.max_speed

        elif Button.LEFT_MINUS in remote_buttons_pressed or Button.RIGHT_MINUS in remote_buttons_pressed:

            if self.current_motor_speed == 0:  # if stopped go in reverse
                self.current_motor_speed = -self.min_speed
            else:
                self.current_motor_speed -= self.speed_step

            # if between 0 and TRAIN_MIN_SPEED snap to stopped
            if self.min_speed > self.current_motor_speed > 0:
                self.current_motor_speed = 0

            if self.current_motor_speed < -self.max_speed:  # max reverse
                self.current_motor_speed = -self.max_speed

        self.train_motor_port_a.dc(self.current_motor_speed)
        if self.train_motor_port_b:
            self.train_motor_port_b.dc(self.current_motor_speed)
        # turn the lights on
        if self.current_motor_speed != 0:
            if self.lights is not None:
                self.lights.on(100)
        else:
            if self.lights is not None:
                self.lights.off()

    # stop all motors
    def stop_motors(self):
        self.train_motor_port_a.dc(0)
        if self.train_motor_port_b:
            self.train_motor_port_b.dc(0)
        if self.lights is not None:
            self.lights.off()




def main():
    error_flash_code = ErrorFlashCodes()
    print('SETUP')
    print('--setup hub')
    try:
        print("--setup countdown")
        countdown_timer = CountdownTimer()
        print("--setup motors")
        drive_motors = RunTrainMotor(error_flash_code, TRAIN_MOTOR_MIN_SPEED, TRAIN_MOTOR_MAX_SPEED, TRAIN_MOTOR_SPEED_STEP,
                             TRAIN_REVERSE_MOTOR_1, TRAIN_REVERSE_MOTOR_2)  # DRIVE_SETUP_END



        if REMOTE_DISABLED:
            print('--no remote')
        else:
            print('--setup remote')
            setup_remote(error_flash_code)


        # give everything a chance to warm up
        wait(500)

        print('SETUP complete')

        countdown_timer.reset()
        while True:
            if not REMOTE_DISABLED:
                countdown_timer.check_remote_buttons()

            if ODV_AUTO_DRIVE_TIMEOUT_SECS > 0:
                if ODV_AUTO_DRIVE_TIMEOUT_SECS > (countdown_timer.remote_buttons_pressed_stopwatch.time()/1000):
                    drive_motors.enable_auto_drive()

            if drive_motors.mh_auto_drive:
                drive_motors.auto_unload()
                drive_motors.auto_load()
            # if there is no remote, then there is no point in a countdown
            elif countdown_timer.has_time_remaining() or REMOTE_DISABLED:
                if drive_motors.mh_supports_homing:
                    drive_motors.do_homing()
                if drive_motors.mh_supports_flip:
                    drive_motors.handle_flip()
                if not REMOTE_DISABLED:
                    drive_motors.handle_remote_press()
            else:
                drive_motors.stop_motors()
                if drive_motors.mh_supports_homing:
                    drive_motors.auto_unload()
                    drive_motors.auto_home()
                    drive_motors.reset_homing()

            countdown_timer.show_status()
            # add a small delay to keep the loop stable and allow for events to occur
            wait(10)

            if REMOTE_DISABLED and ODV_AUTO_DRIVE_TIMEOUT_SECS == 0:
                print("No remote or auto drive exiting")
                sys.exit(0)

    except Exception as e:
        print(e)
        while True:
            error_flash_code.flash_error_code()


if __name__ == "__main__":
    main()
