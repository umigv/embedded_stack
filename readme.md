# embedded_stack

## ODrive Prototyping

### Arduino Flashing

`odrive/arduino/` contains a test file for an Arduino Mega to control the ODrive (`odrive/arduino/src/main.ino`). Connect the Arduino Mega to the ODrive using jumper cables as follows:
* Arduino gnd -> ODrive gnd
* Arduino pin 18 -> ODrive gpio 2
* Arduino pin 19 -> ODrive gpio 1

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

### ODrive Configuration

The ODrive's configuration can be saved to a JSON file using the command `odrivetool backup-config my_config.json` and saved to an ODrive from the configuration file using `odrivetool restore-config my_config.json`. The current configuration is stored under `odrive/config/backup.json`.