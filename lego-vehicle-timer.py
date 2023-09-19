# Timed train and vehicle program for interactive displays
# licence MIT

from pybricks.parameters import Color, Side, Button, Port, Direction
from pybricks.pupdevices import DCMotor, Remote
from pybricks.tools import wait, StopWatch

print('Version 1.0.0')
##################################################################################
#  Settings
##################################################################################

# countdown time settings
COUNTDOWN_LIMIT_MINUTES = const(
    3)  # run for (x) minutes, min 1 minute, max up to you. the default of 3 minutes is play tested :).
# c = center button, + = + button, - = - button
COUNTDOWN_RESET_CODE = 'c,c,c'  # left center button, center button, right center button
# motor setup
# 'train' - Expects a train motor on Port A, and an optional train motor on Port B
# 'skid_steer' - Expects a DC motor on Port and Port B
# 'servo_steer'  - Expects a DC motor on Port A and a self centering Servo motor on Port B
#
VEHICLE_TYPE = 'skid_steer'  # must be one of 'skid_steer', 'servo_steer' or 'train'

# Train mode settings
TRAIN_MOTOR_SPEED_STEP = const(10)  # the amount each button press changes the train speed
TRAIN_MOTOR_MIN_SPEED = const(30)  # lowest speed the train will go set between 30 and 100
TRAIN_MOTOR_MAX_SPEED = const(80)  # set between 80 and 100
TRAIN_REVERSE_MOTOR_1 = False  # set to True if remote + button cause motor to run backwards
TRAIN_REVERSE_MOTOR_2 = True  # only used if a second train motor is on Port B

# skid steer dual motor settings
SKID_STEER_SPEED = const(80)  # set between 50 and 100
SKID_STEER_SWAP_MOTOR_SIDES = False  # set to True if Left/Right remote buttons are backwards
SKID_STEER_REVERSE_LEFT_MOTOR = False  # set to True if remote + button cause motor to run backwards
SKID_STEER_REVERSE_RIGHT_MOTOR = False  # set to True if remote + button cause motor to run backwards

# servo steer settings
SERVO_STEER_SPEED = const(80)  # set between 50 and 100
SERVO_STEER_TURN_ANGLE = const(45)
SERVO_STEER_REVERSE_DRIVE_MOTOR = False
SERVO_STEER_REVERSE_TURN_MOTOR = False


##################################################################################
# ---------Main program below, editing should not be needed -------------


##################################################################################
# Skid steer helper
##################################################################################

class RunSkidSteerMotors:
    """
        Handles driving a skid steer model and reverses control when it flips over
    """

    def __init__(self, drive_speed, swap_motor_sides, reverse_left_motor, reverse_right_motor):

        if swap_motor_sides:
            self.left_motor_port = Port.A
            self.right_motor_port = Port.B
        else:
            self.left_motor_port = Port.B
            self.right_motor_port = Port.A

        if reverse_left_motor:
            self.left_motor_direction = Direction.COUNTERCLOCKWISE
        else:
            self.left_motor_direction = Direction.CLOCKWISE

        if reverse_right_motor:
            self.right_motor_direction = Direction.CLOCKWISE
        else:
            self.right_motor_direction = Direction.COUNTERCLOCKWISE

        self.drive_speed = drive_speed
        self.last_side = None

        self.right_motor = DCMotor(right_motor_port, positive_direction=right_motor_direction)
        self.left_motor = DCMotor(left_motor_port, positive_direction=left_motor_direction)

        self.stop_motors()

    def handle_flip(self):
        """
            swap the motors so that the left and right controls are the same when it flips
        :return:
        """
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
        self.stop()

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

    def __init__(self, drive_speed, turn_angle, reverse_drive_motor, reverse_turn_motor):

        self.drive_speed = drive_speed
        self.turn_angle = turn_angle

        if reverse_drive_motor:
            self.drive_motor = DCMotor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
        else:
            self.drive_motor = DCMotor(Port.A, positive_direction=Direction.CLOCKWISE)

        if reverse_turn_motor:
            self.turn_motor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
        else:
            self.turn_motor = Motor(Port.B, positive_direction=Direction.CLOCKWISE)

        self.left_end = self.turn_motor.run_until_stalled(-200, duty_limit=30)
        self.right_end = self.turn_motor.run_until_stalled(200, duty_limit=30)

        self.stop_motors()

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
        self.stop()

        #  handle button press
        if Button.LEFT_PLUS in remote_buttons:
            self.turn_motor.dc(self.drive_speed)

        if Button.LEFT_MINUS in remote_buttons:
            self.turn_motor.dc(-self.drive_speed)

        if Button.RIGHT_PLUS in remote_buttons:
            self.turn_motor.run_target(200, self.turn_angle)
        elif Button.RIGHT_MINUS in remote_buttons:
            self.turn_motor.run_target(200, -self.turn_angle)
        else:
            self.turn_motor.run_target(200, 0)

    # stop all motors
    def stop_motors(self):
        self.turn_motor.dc(0)


##################################################################################
# Train motor helper
##################################################################################
class RunTrainMotor:
    """
        Handles driving a train ensuring it doesn't run too fast
        Expects a train motor on Port A, and an optional train motor on Port B
    """

    def __init__(self, min_speed, max_speed, speed_step, reverse_motor, reverse_motor_2):

        self.min_speed = min_speed
        self.max_speed = max_speed
        self.speed_step = speed_step
        self.current_motor_speed = 0

        if reverse_motor:
            self.train_motor_1 = DCMotor(Port.A, Direction.COUNTERCLOCKWISE)
        else:
            self.train_motor_1 = DCMotor(Port.A, Direction.CLOCKWISE)

        try:
            if reverse_motor_2:
                self.train_motor_2 = DCMotor(Port.B, Direction.COUNTERCLOCKWISE)
            else:
                self.train_motor_2 = DCMotor(Port.B, Direction.CLOCKWISE)
        except Exception as e2:
            print(e2)
            self.train_motor_2 = None

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

            if self.current_motor_speed > max_speed:
                self.current_motor_speed = max_speed

        elif Button.LEFT_MINUS in remote_buttons or Button.RIGHT_MINUS in remote_buttons:

            if self.current_motor_speed == 0:  # if stopped go in reverse
                self.current_motor_speed = -self.min_speed
            else:
                self.current_motor_speed -= self.speed_step

            # if between 0 and TRAIN_MIN_SPEED snap to stopped
            if self.min_speed > self.current_motor_speed > 0:
                self.current_motor_speed = 0

            if self.current_motor_speed < -max_speed:  # max reverse
                self.current_motor_speed = -max_speed

        self.train_motor_1.dc(self.current_motor_speed)
        if self.train_motor_2:
            self.self.train_motor_2.dc(self.current_motor_speed)

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
    buttons_pressed = remote.buttons.pressed
    while buttons_pressed:
        buttons_pressed = remote.buttons.pressed
        wait(100)


def convert_millis_hours_minutes_seconds(millis):
    """
        utility for making milliseconds hours/minutes/seconds
    :param millis:
    :return hours, minutes, seconds:
    """
    hours = int((millis / (1000 * 60 * 60)) % 24)
    minutes = int((millis / (1000 * 60)) % 60)
    seconds = int((millis / 1000) % 60)

    return hours, minutes, seconds


class CountdownTimer:
    """
    This allows the model to run for a set time
    """
    READY = const(0)
    ACTIVE = const(10)
    FINAL_MINUTE = const(20)
    FINAL_20_SECS = const(30)
    ENDED = const(40)

    ORANGE_HSV = Color(h=5, s=100, v=100)

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
        if remaining_time < (1000 * const(20)):
            self.countdown_status = FINAL_20_SECS
            self.show_status()
            return True

            # in last minute slow flash a warning
        if remaining_time < (1000 * const(60)):
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
        if self.countdown_status == ENDED and all(i in pressed for i in PROGRAM_RESET_CODE_PRESSED) and not any(
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
        if self.countdown_status == READY:
            self.__flash_remote_and_hub_light__(Color.GREEN, 500, Color.RED, 500)
        elif self.countdown_status == ACTIVE:
            hub.light.on(Color.GREEN)
            remote.light.on(Color.GREEN)
        elif self.countdown_status == FINAL_20_SECS:
            self.__flash_remote_and_hub_light__(ORANGE_HSV, 200, Color.NONE, 100)
        elif self.countdown_status == FINAL_MINUTE:
            self.__flash_remote_and_hub_light__(ORANGE_HSV, 500, Color.NONE, 250)
        elif self.countdown_status == ENDED:
            hub.light.on(Color.RED)
            remote.light.on(Color.RED)

    def __flash_remote_and_hub_light__(self, on_color, on_msec: int, off_color, off_msec: int):
        """
            this flashes the remote led
        :param on_color:
        :param on_msec:
        :param off_color:
        :param off_msec:
        """
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

hub = None


def setup_hub():
    global hub

    try:
        # this import will fail if the city hub is not connected.
        from pybricks.hubs import CityHub

        hub = CityHub()
        print('Lego City Hub found')
        return false
    except ImportError:
        try:
            from pybricks.hubs import TechnicHub

            hub = TechnicHub()
            print('Lego Technic Hub found')
            return True

        except ImportError as ex2:
            print(ex2)
            raise Exception('This program only support Lego City hub and Lego Technic hub')


remote = None
LED_FLASHING_SEQUENCE = [75] * 5 + [1000]


def setup_remote():
    global remote

    # Flashing led while waiting connection
    hub.light.blink(Color.WHITE, LED_FLASHING_SEQUENCE)

    # try to connect to remote multiple times
    remote_retry_count = 0

    while true:
        try:
            print("--looking for remote")
            # Connect to the remote.
            remote = Remote()
            print("--remote connected.")
            break
        except Exception as e:
            if remote_retry_count > 20:
                raise
            print(e)
        remote_retry_count += 1
        wait(50)


if __name__ == "__main__":
    print('--setup hub')
    hub_supports_flip = setup_hub()
    print('--setup remote')
    setup_remote()
    print("--setup countdown")
    countdown_timer = CountdownTimer()
    countdown_timer.reset()
    print("--setup motors")
    if VEHICLE_TYPE == 'skid_steer':
        drive_motors = RunSkidSteerMotors(SKID_STEER_SPEED, SKID_STEER_SWAP_MOTOR_SIDES, SKID_STEER_REVERSE_LEFT_MOTOR,
                                          SKID_STEER_REVERSE_RIGHT_MOTOR)

    elif VEHICLE_TYPE == 'servo_steer':
        drive_motors = RunServoSteerMotors(SERVO_STEER_SPEED, SERVO_STEER_TURN_ANGLE, SERVO_STEER_REVERSE_DRIVE_MOTOR,
                                           SERVO_STEER_REVERSE_TURN_MOTOR)

    elif VEHICLE_TYPE == 'train':
        drive_motors = RunTrainMotor(TRAIN_MOTOR_MIN_SPEED, TRAIN_MOTOR_MAX_SPEED, TRAIN_MOTOR_SPEED_STEP,
                                     TRAIN_REVERSE_MOTOR_1, TRAIN_REVERSE_MOTOR_2)
    else:
        raise Exception('VEHICLE_TYPE must be one of [skid_steer,servo_steer,train]')

    # give everything a chance to warm up
    wait(500)

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
