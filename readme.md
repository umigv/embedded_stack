# embedded_stack

## ODrive Prototyping

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

## Working Code

Currently, 'ODriveTwoMotors' is the working code to run in the Arduino Mega. It is able to control the odrives and communicate with ROS. 
