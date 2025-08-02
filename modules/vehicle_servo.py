from .lego_vehicle_timer_base import MotorHelper, ErrorFlashCodes

error_flash_code = ErrorFlashCodes()
from micropython import const
from pybricks.parameters import Button
from pybricks.pupdevices import Remote

remote: Remote | None = None

# IMPORTS_START
from pybricks.parameters import Port, Direction
from pybricks.pupdevices import DCMotor, Motor
from uerrno import ENODEV

# IMPORTS_END
# VARS_START
# servo steer settings
SERVO_STEER_SPEED: int = const(80)  # set between 50 and 100
SERVO_STEER_TURN_ANGLE: int = const(45)  # angle to turn wheels
SERVO_STEER_REVERSE_DRIVE_MOTOR: bool = False  # set to True if remote + button cause motor to run backwards
SERVO_STEER_REVERSE_TURN_MOTOR: bool = False  # set to True if remote + button cause motor to turn wrong way


# VARS_END
# MODULE_START
##################################################################################
# Servo steer helper
##################################################################################

class RunServoSteerMotors(MotorHelper):
    """
        Handles driving a servo steer model
        Expects a DC motor on Port A and a self centering Servo motor on Port B
    """

    def __init__(self, error_flash_code_helper: ErrorFlashCodes, drive_speed: int, turn_angle: int,
                 reverse_drive_motor: bool, reverse_steering_motor: bool):
        super().__init__(False, False)

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

    def handle_remote_press(self):
        """
            handle remote button clicks
        """
        # Check which remote_buttons are pressed.
        remote_buttons_pressed = remote.buttons.pressed()
        if len(remote_buttons_pressed) == 0 or Button.RIGHT in remote_buttons_pressed or Button.LEFT in remote_buttons_pressed:
            self.stop_motors()
            return
        # stop motors as this is bang-bang mode where a button
        #  needs to be held down for racer to run

        #  handle button press
        if Button.LEFT_PLUS in remote_buttons_pressed:
            self.drive_motor.dc(self.drive_speed)
        elif Button.LEFT_MINUS in remote_buttons_pressed:
            self.drive_motor.dc(-self.drive_speed)
        else:
            self.drive_motor.dc(0)

        if Button.RIGHT_PLUS in remote_buttons_pressed:
            self.steering_motor.run_target(200, self.turn_angle, wait=False)
        elif Button.RIGHT_MINUS in remote_buttons_pressed:
            self.steering_motor.run_target(200, -self.turn_angle, wait=False)
        else:
            self.steering_motor.run_target(200, 0, wait=False)

    # stop all motors
    def stop_motors(self):
        self.drive_motor.dc(0)
        self.steering_motor.run_target(200, 0)


# MODULE_END
# DRIVE_SETUP_START
drive_motors = RunServoSteerMotors(error_flash_code, SERVO_STEER_SPEED, SERVO_STEER_TURN_ANGLE,
                                   SERVO_STEER_REVERSE_DRIVE_MOTOR, SERVO_STEER_REVERSE_TURN_MOTOR)  # DRIVE_SETUP_END
