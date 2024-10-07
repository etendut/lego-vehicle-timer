# Timed train and vehicle program for interactive displays
# Copyright Etendut
# licence MIT
try:
    from typing import Union
except ImportError:
    def Union():
        pass
from micropython import const
from pybricks.parameters import Color, Side, Button, Port, Direction
from pybricks.pupdevices import DCMotor, Motor, Remote
from pybricks.tools import wait, StopWatch
from uerrno import ENODEV

print('Version 1.3.1')
##################################################################################
#  Settings
##################################################################################

# vehicle type
# 'train' - Expects a train motor on Port A, and an optional train motor on Port B
# 'skid_steer' - Expects a DC motor on Port and Port B
# 'servo_steer'  - Expects a DC motor on Port A and a servo type motor on Port B
#
VEHICLE_TYPE = 'train'  # must be one of 'skid_steer', 'servo_steer' or 'train'

# countdown time settings
COUNTDOWN_LIMIT_MINUTES: int = const(
    3)  # run for (x) minutes, min 1 minute, max up to you. the default of 3 minutes is play tested :).
# c = center button, + = + button, - = - button
COUNTDOWN_RESET_CODE = 'c,c,c'  # left center button, center button, right center button

# Train mode settings
TRAIN_MOTOR_SPEED_STEP: int = const(10)  # the amount each button press changes the train speed
TRAIN_MOTOR_MIN_SPEED: int = const(30)  # lowest speed the train will go set between 30 and 100
TRAIN_MOTOR_MAX_SPEED: int = const(80)  # set between 80 and 100
TRAIN_REVERSE_MOTOR_1: bool = False  # set to True if remote + button cause motor to run backwards
TRAIN_REVERSE_MOTOR_2: bool = True  # only used if a second train motor is on Port B

# skid steer dual motor settings
SKID_STEER_SPEED: int = const(80)  # set between 50 and 100
SKID_STEER_SWAP_MOTOR_SIDES: bool = False  # set to True if Left/Right remote buttons are backwards
SKID_STEER_REVERSE_LEFT_MOTOR: bool = False  # set to True if remote + button cause motor to run backwards
SKID_STEER_REVERSE_RIGHT_MOTOR: bool = False  # set to True if remote + button cause motor to run backwards

# servo steer settings
SERVO_STEER_SPEED: int = const(80)  # set between 50 and 100
SERVO_STEER_TURN_ANGLE: int = const(45)  # angle to turn wheels
SERVO_STEER_REVERSE_DRIVE_MOTOR: bool = False  # set to True if remote + button cause motor to run backwards
SERVO_STEER_REVERSE_TURN_MOTOR: bool = False  # set to True if remote + button cause motor to turn wrong way


##################################################################################
# ---------Main program below, editing should not be needed -------------


class ErrorFlashCodes:
    def __init__(self):
        self.flash_count = 1 #Other errors

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



##################################################################################
# Skid steer helper
##################################################################################

class RunSkidSteerMotors:
    """
        Handles driving a skid steer model and reverses control when it flips over
    """

    def __init__(self, error_flash_code_helper: ErrorFlashCodes, drive_speed: int, swap_motor_sides: bool,
                 reverse_left_motor: bool, reverse_right_motor: bool):

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
        # Check which remote_buttons are pressed.
        remote_buttons = remote.buttons.pressed()

        # stop motors as this is bang-bang mode where a button
        #  needs to be held down for racer to run
        self.stop_motors()

        #  handle button press
        if Button.LEFT_PLUS in remote_buttons:
            self.left_motor.dc(self.drive_speed)

        if Button.LEFT_MINUS in remote_buttons:
            self.left_motor.dc(-self.drive_speed)

        if Button.RIGHT_PLUS in remote_buttons:
            self.right_motor.dc(self.drive_speed)

        if Button.RIGHT_MINUS in remote_buttons:
            self.right_motor.dc(-self.drive_speed)

    # stop all motors
    def stop_motors(self):
        self.left_motor.dc(0)
        self.right_motor.dc(0)


##################################################################################
# Servo steer helper
##################################################################################

class RunServoSteerMotors:
    """
        Handles driving a servo steer model
        Expects a DC motor on Port A and a self centering Servo motor on Port B
    """

    def __init__(self, error_flash_code_helper: ErrorFlashCodes, drive_speed: int, turn_angle: int,
                 reverse_drive_motor: bool, reverse_steering_motor: bool):
        self.error_flash_code = error_flash_code_helper
        self.drive_speed = drive_speed
        self.turn_angle = turn_angle
        try:
            if reverse_drive_motor:
                self.drive_motor = DCMotor(Port.A, positive_direction=Direction.CLOCKWISE)
            else:
                self.drive_motor = DCMotor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
            print('Found drive motor on ' + str(Port.A))
        except OSError as ex:
            if ex.errno == ENODEV:
                print('Drive motor needs to be connected to ' + str(Port.A))
                self.error_flash_code.set_error_no_motor_on_a()
            raise
        try:
            if reverse_steering_motor:
                self.steering_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
            else:
                self.steering_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)
            print('Found steering motor on ' + str(Port.A))
        except OSError as ex:
            if ex.errno == ENODEV:
                print('Steering motor needs to be connected to ' + str(Port.B))
                self.error_flash_code.set_error_no_motor_on_b()
            raise

        self.calibrate_steering()
        self.stop_motors()

    def calibrate_steering(self):
        print('--setting steering limits')
        left_end = self.steering_motor.run_until_stalled(-200, duty_limit=60)
        right_end = self.steering_motor.run_until_stalled(200, duty_limit=60)
        self.steering_motor.reset_angle((right_end - left_end) / 2)
        print('--centering')
        self.steering_motor.run_target(200, 0)

    def handle_flip(self):
        """
            flipping is bad news, added to keep same interface
        :return:
        """
        pass

    def handle_remote_press(self):
        """
            handle remote button clicks
        """
        # Check which remote_buttons are pressed.
        remote_buttons = remote.buttons.pressed()

        # stop motors as this is bang-bang mode where a button
        #  needs to be held down for racer to run

        #  handle button press
        if Button.LEFT_PLUS in remote_buttons:
            self.drive_motor.dc(self.drive_speed)
        elif Button.LEFT_MINUS in remote_buttons:
            self.drive_motor.dc(-self.drive_speed)
        else:
            self.drive_motor.dc(0)

        if Button.RIGHT_PLUS in remote_buttons:
            self.steering_motor.run_target(200, self.turn_angle, wait=False)
        elif Button.RIGHT_MINUS in remote_buttons:
            self.steering_motor.run_target(200, -self.turn_angle, wait=False)
        else:
            self.steering_motor.run_target(200, 0, wait=False)

    # stop all motors
    def stop_motors(self):
        self.drive_motor.dc(0)
        self.steering_motor.run_target(200, 0)


##################################################################################
# Train motor helper
##################################################################################
class RunTrainMotor:
    """
        Handles driving a train ensuring it doesn't run too fast
        Expects a train motor on Port A, and an optional train motor on Port B
    """

    def __init__(self, error_flash_code_helper: ErrorFlashCodes, min_speed: int, max_speed: int, speed_step: int,
                 reverse_motor: bool, reverse_motor_2: bool):
        self.error_flash_code = error_flash_code_helper
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.speed_step = speed_step
        self.current_motor_speed = 0
        motor_found = False
        self.train_motor_1 = None
        self.train_motor_2 = None
        # noinspection PyBroadException
        try:
            if reverse_motor:
                self.train_motor_1 = DCMotor(Port.A, Direction.COUNTERCLOCKWISE)
            else:
                self.train_motor_1 = DCMotor(Port.A, Direction.CLOCKWISE)
            print('Found train motor on ' + str(Port.A))
            motor_found = True
        except:
            pass
            self.train_motor_1 = None  # ignore not found first motor

        if not motor_found:
            # noinspection PyBroadException
            try:
                if reverse_motor_2:
                    self.train_motor_2 = DCMotor(Port.B, Direction.COUNTERCLOCKWISE)
                else:
                    self.train_motor_2 = DCMotor(Port.B, Direction.CLOCKWISE)
                print('Found train motor on ' + str(Port.B))
                motor_found = True
            except:
                pass
                # ignore not found
                self.train_motor_2 = None

        if not motor_found:
            self.error_flash_code.set_error_no_motor_on_a()
            raise Exception('Train motor needs to be connected to ' + str(Port.A) + ' or ' + str(Port.B))

    def handle_remote_press(self):
        """
            handle remote button clicks
        """
        # Check which remote_buttons are pressed.
        remote_buttons = remote.buttons.pressed()

        # left remote_buttons
        # noinspection DuplicatedCode
        if Button.LEFT in remote_buttons or Button.RIGHT in remote_buttons:
            self.current_motor_speed = 0

        elif Button.LEFT_PLUS in remote_buttons or Button.RIGHT_PLUS in remote_buttons:

            if self.current_motor_speed == 0:  # if stopped go forward
                self.current_motor_speed = self.min_speed
            else:
                self.current_motor_speed += self.speed_step

            # if between 0 and TRAIN_MIN_SPEED snap to stopped
            if 0 > self.current_motor_speed > -self.min_speed:
                self.current_motor_speed = 0

            if self.current_motor_speed > self.max_speed:
                self.current_motor_speed = self.max_speed

        elif Button.LEFT_MINUS in remote_buttons or Button.RIGHT_MINUS in remote_buttons:

            if self.current_motor_speed == 0:  # if stopped go in reverse
                self.current_motor_speed = -self.min_speed
            else:
                self.current_motor_speed -= self.speed_step

            # if between 0 and TRAIN_MIN_SPEED snap to stopped
            if self.min_speed > self.current_motor_speed > 0:
                self.current_motor_speed = 0

            if self.current_motor_speed < -self.max_speed:  # max reverse
                self.current_motor_speed = -self.max_speed

        self.train_motor_1.dc(self.current_motor_speed)
        if self.train_motor_2:
            self.train_motor_2.dc(self.current_motor_speed)

    def handle_flip(self):
        """
            flipping is bad news, added to keep same interface
        :return:
        """
        pass

    # stop all motors
    def stop_motors(self):
        self.train_motor_1.dc(0)
        if self.train_motor_2:
            self.train_motor_2.dc(0)


##################################################################################
# Countdown helper
##################################################################################

def wait_for_no_pressed_buttons():
    buttons_pressed = remote.buttons.pressed()
    while buttons_pressed:
        buttons_pressed = remote.buttons.pressed()
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


class CountdownTimer:
    """
    This allows the model to run for a set time
    """

    def __init__(self):
        # assign external objects to properties of the class
        self.countdown_status = None

        # Start a timer.
        self.countdown_stopwatch = StopWatch()
        self.led_flash_stopwatch = StopWatch()
        self.last_console_msg = None
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
        msg = 'countdown ending in: {}:{:02}'.format(con_min, con_sec)
        if self.last_console_msg != msg:
            self.last_console_msg = msg
            print(msg)  # when time has run out end countdown
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
        print('countdown time reset, press Remote CENTER to restart countdown')
        self.countdown_status = READY

    def check_remote_buttons(self):
        """
            check countdown time buttons
        """
        pressed = remote.buttons.pressed()

        if self.countdown_status == READY and Button.CENTER in pressed:
            self.__start_countdown__()
            wait_for_no_pressed_buttons()

        # if reset sequence pressed reset the countdown timer
        if all(i in pressed for i in PROGRAM_RESET_CODE_PRESSED) and not any(
                i in pressed for i in PROGRAM_RESET_CODE_NOT_PRESSED):
            self.reset()
            wait_for_no_pressed_buttons()

    def check_reset_code_pressed(self):
        """
           Check if reset code was pressed
        """
        pressed = remote.buttons.pressed()
        # if reset sequence pressed at other times, end countdown
        if self.countdown_status != READY and all(i in pressed for i in PROGRAM_RESET_CODE_PRESSED) and not any(
                i in pressed for i in PROGRAM_RESET_CODE_NOT_PRESSED):
            print('reset code pressed')
            self.reset()
            wait_for_no_pressed_buttons()

    def show_status(self):
        global hub
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


hub: MockHub = MockHub()


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


remote: Union[Remote, None] = None
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
            # Connect to the remote.
            remote = Remote()
            print("--remote connected.")
            break
        except:
            if remote_retry_count >= retry:
                error_flash_code_helper.set_error_no_remote()
                raise  # ignore first 20 errors
        remote_retry_count += 1
        wait(50)


def main():
    error_flash_code = ErrorFlashCodes()
    print('SETUP')
    print('--setup hub')
    hub_supports_flip = setup_hub()
    try:
        print("--setup countdown")
        countdown_timer = CountdownTimer()
        print("--setup motors")
        print(VEHICLE_TYPE)
        if VEHICLE_TYPE == 'skid_steer':
            drive_motors = RunSkidSteerMotors(error_flash_code, SKID_STEER_SPEED, SKID_STEER_SWAP_MOTOR_SIDES,
                                              SKID_STEER_REVERSE_LEFT_MOTOR, SKID_STEER_REVERSE_RIGHT_MOTOR)

        elif VEHICLE_TYPE == 'servo_steer':
            drive_motors = RunServoSteerMotors(error_flash_code, SERVO_STEER_SPEED, SERVO_STEER_TURN_ANGLE,
                                               SERVO_STEER_REVERSE_DRIVE_MOTOR, SERVO_STEER_REVERSE_TURN_MOTOR)

        elif VEHICLE_TYPE == 'train':
            drive_motors = RunTrainMotor(error_flash_code, TRAIN_MOTOR_MIN_SPEED, TRAIN_MOTOR_MAX_SPEED,
                                         TRAIN_MOTOR_SPEED_STEP, TRAIN_REVERSE_MOTOR_1, TRAIN_REVERSE_MOTOR_2)
        else:
            raise Exception('VEHICLE_TYPE must be one of [skid_steer,servo_steer,train]')

        print('--setup remote')
        setup_remote(error_flash_code)

        # give everything a chance to warm up
        wait(500)
        print('SETUP complete')
        countdown_timer.reset()
        while True:
            countdown_timer.check_remote_buttons()
            if countdown_timer.has_time_remaining():
                if hub_supports_flip:
                    drive_motors.handle_flip()
                drive_motors.handle_remote_press()
            else:
                drive_motors.stop_motors()

            countdown_timer.show_status()
            # add a small delay to keep the loop stable and allow for events to occur
            wait(100)
    except Exception as e:
        print(e)
        while True:
            error_flash_code.flash_error_code()


if __name__ == "__main__":
    main()
