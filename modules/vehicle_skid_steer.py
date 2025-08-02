from .lego_vehicle_timer_base import MotorHelper, ErrorFlashCodes

error_flash_code = ErrorFlashCodes()
from micropython import const
from pybricks.parameters import Side, Button
from pybricks.pupdevices import Remote
from pybricks.hubs import TechnicHub

hub: TechnicHub | None = None
remote: Remote | None = None

# IMPORTS_START
from pybricks.parameters import Port, Direction
from pybricks.pupdevices import DCMotor
from uerrno import ENODEV

# IMPORTS_END
# VARS_START
# skid steer dual motor settings
SKID_STEER_SPEED: int = const(80)  # set between 50 and 100
SKID_STEER_SWAP_MOTOR_SIDES: bool = False  # set to True if Left/Right remote buttons are backwards
SKID_STEER_REVERSE_LEFT_MOTOR: bool = False  # set to True if remote + button cause motor to run backwards
SKID_STEER_REVERSE_RIGHT_MOTOR: bool = False  # set to True if remote + button cause motor to run backwards


# VARS_END
# MODULE_START
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


# MODULE_END
# DRIVE_SETUP_START
drive_motors = RunSkidSteerMotors(error_flash_code, SKID_STEER_SPEED, SKID_STEER_SWAP_MOTOR_SIDES,
                                  SKID_STEER_REVERSE_LEFT_MOTOR, SKID_STEER_REVERSE_RIGHT_MOTOR)  # DRIVE_SETUP_END
