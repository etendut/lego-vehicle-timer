# Train, Servo Steer, and Skid Steer timed vehicle Program

A python program to enable a time limited running of a Train, Servo Steer, or Skid Steer Lego vehicle.

## Requires

- Lego City Hub or Lego Technic Hub using PyBricks firmware
- Train
  - 1 or 2 motors
- Skid Steer
  - 2 motors, one for each side
- Servo Steer
  - 1 motor for drive
  - 1 motor with rotation sensor for steering
- Lego Remote

## How do I get set up?

- Use [PyBricks](https://code.pybricks.com/) to configure your hub
- load [lego_vehicle_timer](lego_vehicle_timer.py) into PyBricks
- configure the settings as per below [Configuration](#configuration)
- flash to hub using PyBricks

## Using the Remote

### Train

Either button sets control forward and reverse
<img src="TrainRemote.jpg" alt="Train Remote Instructions" style="max-height:200px;" />

### Skid Steer

Left buttons left motor, Right buttons right motor
<img src="SkidSteerRemote.jpg" alt="Skid Steer Remote Instructions" style="max-height:200px;" />

### Servo Steer

Left buttons for drive, Right buttons for steering
<img src="ServoSteerRemote.jpg" alt="Servo Steer Remote Instructions" style="max-height:200px;" />

## Configuration

All configuration should be done in [lego_vehicle_timer](lego_vehicle_timer.py) before installing with [PyBricks](https://code.pybricks.com/)

### Vehicle type

train - Expects a train motor on Port A, and an optional train motor on Port B
skid_steer - Expects a DC motor on Port and Port B
servo_steer - Expects a DC motor on Port A and a motor with a rotation sensor on Port B

VEHICLE_TYPE = 'skid_steer' # must be one of 'skid_steer', 'servo_steer' or 'train'

### Countdown time settings

COUNTDOWN_LIMIT_MINUTES = const(3) # run for (x) minutes, min 1 minute, max up to you. the default of 3 minutes is play
tested :).
c = center button, + = + button, - = - button
COUNTDOWN_RESET_CODE = 'c,c,c' # left center button, center button, right center button

### Train mode settings

TRAIN_MOTOR_SPEED_STEP = const(10) # the amount each button press changes the train speed
TRAIN_MOTOR_MIN_SPEED = const(30) # lowest speed the train will go set between 0 and 100
TRAIN_MOTOR_MAX_SPEED = const(80) # set between 0 and 100
TRAIN_REVERSE_MOTOR = False # set to True if remote + button cause motor to run backwards

### skid steer dual motor settings

SKID_STEER_SPEED = const(80) # set between 0 and 100
SKID_STEER_SWAP_MOTOR_SIDES = False # set to True if Left/Right remote buttons are backwards
SKID_STEER_REVERSE_LEFT_MOTOR = False # set to True if remote + button cause motor to run backwards
SKID_STEER_REVERSE_RIGHT_MOTOR = False # set to True if remote + button cause motor to run backwards

### servo steer settings

SERVO_STEER_SPEED = const(80) # set between 50 and 100
SERVO_STEER_TURN_ANGLE = const(45) # angle to turn wheels
SERVO_STEER_REVERSE_DRIVE_MOTOR = False # set to True if remote + button cause motor to run backwards
SERVO_STEER_REVERSE_TURN_MOTOR = False # set to True if remote + button cause motor to turn wrong way

## Releases

### Version 1.1.0 (current)

Initial release

## Licence

Copyright © 2023, [Etendut](https://github.com/etendut).
Released under the [MIT License](LICENSE).
