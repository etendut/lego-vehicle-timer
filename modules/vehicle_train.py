from pybricks.parameters import Button
from pybricks.pupdevices import Remote
from micropython import const
from .lego_vehicle_timer_base import MotorHelper, ErrorFlashCodes

error_flash_code = ErrorFlashCodes()
remote: Remote | None = None

# IMPORTS_START
from pybricks.parameters import Port, Direction
from pybricks.pupdevices import DCMotor, Light

# IMPORTS_END
# VARS_START
# Train mode settings
TRAIN_MOTOR_SPEED_STEP: int = const(10)  # the amount each button press changes the train speed
TRAIN_MOTOR_MIN_SPEED: int = const(30)  # lowest speed the train will go set between 30 and 100
TRAIN_MOTOR_MAX_SPEED: int = const(80)  # set between 80 and 100
TRAIN_REVERSE_MOTOR_1: bool = False  # set to True if remote + button cause motor to run backwards
TRAIN_REVERSE_MOTOR_2: bool = True  # only used if a second train motor is on Port B


# VARS_END
# MODULE_START
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


# MODULE_END
# DRIVE_SETUP_START
drive_motors = RunTrainMotor(error_flash_code, TRAIN_MOTOR_MIN_SPEED, TRAIN_MOTOR_MAX_SPEED, TRAIN_MOTOR_SPEED_STEP,
                             TRAIN_REVERSE_MOTOR_1, TRAIN_REVERSE_MOTOR_2)  # DRIVE_SETUP_END
