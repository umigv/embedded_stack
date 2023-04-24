# embedded_stack

## ODrive Prototyping

### Connecting to ODrive

* First make sure to install a version of Python, either in the terminal or through the Anaconda Navigator (recommended).
* Then type `pip install --upgrade odrive` in the Python terminal
* Connect your device to the ODrive and type `odrivetool` in the terminal.

You should now see that your device has successfully connected to an ODrive. Remember, the ODrives require at least 12V and must be powered separately through a battery.

#### Troubleshooting

If you're on Windows, you may run into an issue where the USB device doesn't open. Open the [Zadig Utility](https://zadig.akeo.ie/) and download the latest version. Make sure to hit 'X' on the advertisement. Then run the `.exe` file and click "Allow." Connect to your ODrive if you have not already, and ensure that you see a `WinUSB` in one box and `ODrive Native Interface` in another box (these are not exact messages). Hit "Install." After a few minutes, the driver will be installed, and typing `odrivetool` in the Python terminal will allow you to successfully interface with the ODrives.

### Arduino Flashing

`odrive/arduino/` contains a test file for an Arduino Mega to control the ODrive (`odrive/arduino/src/main.ino`). Connect the Arduino Mega to the ODrive using jumper cables as follows:

* Arduino gnd -> ODrive gnd
* Arduino pin 18 -> ODrive gpio 2
* Arduino pin 19 -> ODrive gpio 1

The software depends on the [ODrive Arduino library](https://github.com/odriverobotics/ODrive/tree/master/Arduino/ODriveArduino). Follow [these](https://github.com/odriverobotics/ODrive/blob/master/Arduino/ODriveArduino/README.md) instructions to install this library in your Arduino IDE.

After flashing and opening the serial monitor at 115200 baud, it should say:

```
ODriveArduino
Setting parameters...
Ready!
Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)
Send the character 's' to exectue test move
Send the character 'b' to read bus voltage
Send the character 'p' to read motor positions in a 10s loop
```

When no motor is connected only the `b` command works; it should return roughly 24.

### ODrive Motor Testing

To work with the motors, follow the encoder and motor setup [here](https://docs.odriverobotics.com/v/latest/getting-started.html#motor-configuration).
Afterwards, calibrate the motors with `odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE`. Within two seconds, a beep will occur, followed by the calibration. The calibration consists of three different phases. If at any point the calibration fails, the motor will stop working, and the ODrive will update all the error flags.

After a successful calibration sequence, enter `odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL`. Then type `odrv0.axis0.controller.config.control_mode` to check what mode the ODrive is in. The integer value corresponds to one of four modes, which can be verified [here](https://betadocs.odriverobotics.com/api/odrive.controller.controlmode). For now, set it to `CONTROL_MODE_VELOCITY_CONTROL` by typing `odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL`.

To set an input velocity, type `odrv0.axis0.controller.input_vel = x` where x is the value in revolutions per second. The current tested range is (-MAX, 2) ∪ [0, 0] ∪ (2, MAX) where MAX is the maximum revolutions per second allowed. This value can be changed by the user. If at any point the motor stops spinning, it means that the ODrive has errors and that the error flags have been set.

If there are errors at any point, type `dump_errors(odrv0)` to view all errors, and type `odrv0.clear_errors()` to reset all the error flags. Then start over with the calibration sequence.

### ODrive Configuration

The ODrive's configuration can be saved to a JSON file using the command `odrivetool backup-config my_config.json` and saved to an ODrive from the configuration file using `odrivetool restore-config my_config.json`. The current configuration is stored under `odrive/config/backup.json`.

### Current Working ODrive Parameters
The main ROS script *should* have these be the default values that are set upon start-up. If not, please configure them to these values or update the README accordingly.  

* `pos_gain`: 6.0 for both axes
* `vel_gain`: 0.07 for axis0; 0.08 for axis1  
* `vel_integrator_gain`: 0.01 for both axes  

## Working Code

Currently, 'ODriveTwoMotors' is the working code to run in the Arduino Mega. It is able to control the odrives and communicate with ROS. 'TuneMotors' is also a working script, but it's meant for debugging. See below for how to use the script. If it gets updated, please update the README accordingly.

### `TuneMotors.ino` Tutorial

This is a very handy script to interface with the Arduino Mega and debug the robot. Make sure to open the Arduino Serial Monitor and have the "Newline" option enabled. Here's all the things a user can do:
* `c`: Sets it to calibration mode. Send `0` for calibrating axis0 and `1` for calibrating axis1. Do *not* calibrate both motors simultaneously.
* `p`: Sets it to position mode. Send a number to set a position for both motors simultaneously. This position is based on the encoder values.
* `v`: Sets it to velocity mode. Send a number to set a velocity for both motors simultaneously. Send `l` directly followed by a number to set just the left motor to a velocity. Send `r` directly followed by a number to set just the right motor to a velocity.
* `t`: Toggles the ability to tune. This is *not* a separate mode; it's just an extra step to tune values on the fly.
* `pg`, `lpg`, and `rpg`: If tuning is enabled, this will prompt the user to enter a value. Otherwise press `q` to cancel. The value entered will be used to set `pos_gain` for both motors, the left motor, or the right motor, respectively.
* `vg`, `lvg`, and `rvg`: If tuning is enabled, this will prompt the user to enter a value. Otherwise press `q` to cancel. The value entered will be used to set `vel_gain` for both motors, the left motor, or the right motor, respectively.
* `vig`, `lvig`, and `rvig`: If tuning is enabled, this will prompt the user to enter a value. Otherwise press `q` to cancel. The value entered will be used to set `vel_integrator_gain` for both motors, the left motor, or the right motor, respectively.
* `q`: Quits the application. Because this is an Arduino script, it's equivalent to a reset and restart.

## Pins Used

* E-Stop -> Arduino Digital Pin 32
* ODrive 1 -> Serial 1
* ODrive 2 -> Serial 2
