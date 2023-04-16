// file pulled from https://github.com/odriverobotics/ODrive/tree/master/Arduino/ODriveArduino
// this file is flashed to the Arduino Mega to control the ODrive via serial/uart
// after flashing set the PC's baud rate to 115200
// for more information read this repo's readme
// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
// Printing with stream operator helper functions
template <class T>
inline Print &operator<<(Print &obj, T arg)
{
  obj.print(arg);
  return obj;
}
template <>
inline Print &operator<<(Print &obj, float arg)
{
  obj.print(arg, 4);
  return obj;
}


void setOdriveVelocity(float vel);
void setOdrivePosition(float vel);

HardwareSerial &odrive_serial = Serial1;

// ODrive object
ODriveArduino odrive(odrive_serial);

int AXIS_0_POLARITY = -1;
int AXIS_1_POLARITY = 1;

bool positionMode = false;
bool velocityMode = false;
bool calibrationMode = false;

void setupODrive(ODriveArduino &odrive);
void setupODriveParams(ODriveArduino &odrive);
void closedLoopControl(ODriveArduino &odrive);

void setup()
{
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial)
    ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  setupODriveParams(odrive);
  int requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  odrive.run_state(1, requested_state, true);
  delay(10000);
  odrive.run_state(0, requested_state, true);

  // wait for calibration before setting velocity
  delay(10000);

  Serial.println("Finished calibration!");
  closedLoopControl(odrive);

  Serial.println("Ready!");
}

void loop()
{
  // Counter for prints (alternative for delays that doesn't stall)
  static uint32_t counter = 1;

  // Stop variable --> Reads wireless e-stop
  uint8_t stop = digitalRead(6);

  // Tuning toggling variable
  static uint8_t tuning = 0;

  // Variables for reading input
  String input = "";
  char c = '\0';

  // Read input
  while (Serial.available())
  {
    // gets one byte from serial buffer
    c = Serial.read();

    // makes the String input
    input += c;

    // slow looping to allow buffer to fill with next character
    delay(2);
  }

  // Quit
  if (input == "q\n")
  {
    return;
  }

  // Remote E-Stop
  else if (stop)
  {
    setOdriveVelocity(0);
  }

  // Toggle Tuning Mode
  else if (input == "t\n")
  { // tuning
    tuning = !tuning;
    if (tuning) Serial.println("Tuning is on!");
    else Serial.println("Tuning is off.");
  }

  // Set velocity control mode
  else if (input == "v\n") {
    positionMode = false;
    velocityMode = true;
    calibrationMode = false;
    Serial.println("Velocity Mode Activated!\nPosition Mode Deactivated!\nCalibration Mode Deactivated!");
    odrive_serial << "w axis0.controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << "\n";
    odrive_serial << "w axis1.controller.config.control_mode " << CONTROL_MODE_VELOCITY_CONTROL << "\n";
  }

  // Set position control mode
  else if (input == "p\n") {
    velocityMode = false;
    positionMode = true;
    calibrationMode = false;
    Serial.println("Position Mode Activated!\nVelocity Mode Deactivated!\nCalibration Mode Deactivated!");
    odrive_serial << "w axis0.controller.config.control_mode " << CONTROL_MODE_POSITION_CONTROL << "\n";
    odrive_serial << "w axis1.controller.config.control_mode " << CONTROL_MODE_POSITION_CONTROL << "\n";
  }

  // Set calibration mode
  else if (input == "c\n") {
    velocityMode = false;
    positionMode = false;
    calibrationMode = true;
    Serial.println("Calibration Mode Activated!\nVelocity Mode Deactivated!\nPosition Mode Deactivated!");
  }
  
  else if (input.length() > 0 || stop)
  {
    if (!stop) {
      float n = digitalRead(6) ? 0 : input.toFloat(); // convert input into a number
      if (positionMode) {
        Serial.println("Input position: " + input);   // so you can see the captured String
        setOdrivePosition(n);
      }
      if (velocityMode) { 
        Serial.println("Input velocity: " + input);   // so you can see the captured String
        setOdriveVelocity(n);
      }
      if (calibrationMode) {
        if (n == 2) {
          Serial.println("Calibrating both axes simultaneously");   // so you can see the captured String
          odrive.run_state(0, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true);
          odrive.run_state(1, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true);
        }
        else if (n == 0 || n == 1) {
          Serial.println("Calibrating Axis " + input);
          odrive.run_state(n, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, true);
        }
        
      }
    }
    input = ""; // reset string
  }

  else if (input.length() > 0)
  {
    float n = input.toFloat(); // convert input into a number
      if (positionMode) {
        Serial.println("Input position: " + input);   // so you can see the captured String
        setOdrivePosition(n);
      }
      if (velocityMode) { 
        Serial.println("Input velocity: " + input);   // so you can see the captured String
        setOdriveVelocity(n);
      }
  }
  delay(1);
  input = ""; // reset
  c = '\0';   // reset
}

void closedLoopControl(ODriveArduino &odrive)
{
  int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.run_state(0, requested_state, false);
  odrive.run_state(1, requested_state, false);

  setOdriveVelocity(0);
}

void setOdriveVelocity(float vel)
{
  Serial.print("Set odrive velocity to: ");
  Serial.println(vel);
  odrive.SetVelocity(0, vel * AXIS_0_POLARITY);
  odrive.SetVelocity(1, vel * AXIS_1_POLARITY);
}

void setOdrivePosition(float pos)
{
  Serial.print("Set odrive position to: ");
  Serial.println(pos);
  odrive.SetPosition(0, pos * AXIS_0_POLARITY);
  odrive.SetPosition(1, pos * AXIS_1_POLARITY);
}

void setVelGain(int motor, HardwareSerial &serial, float val)
{
  serial << "w axis" << motor << ".controller.config.vel_gain = " << val << "\n";
}

// function doesn't currently return
float readVelGain(int motor, HardwareSerial &serial)
{
  serial << "w axis" << motor << ".controller.config.vel_gain";
  return 0;
}

void setupODriveParams(ODriveArduino& odrive) {
  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  //
  // AXIS0 Plastic
  // pos_gain = 6.1
  // vel_gain = 0.011
  // vel_integrator_gain = 0
  odrive_serial << "w axis0.motor.config.current_lim " << 60.0f << '\n';
  odrive_serial << "w axis0.encoder.config.cpr " << 42.0f << '\n';
  odrive_serial << "w axis0.controller.config.pos_gain " << 6.1f << '\n';
  odrive_serial << "w axis0.controller.config.vel_gain " << 0.03 << '\n';
  odrive_serial << "w axis0.controller.config.vel_integrator_gain " << 0.06f << '\n';
  // AXIS1 Plastic
  // pos_gain = 3.6
  // vel_gain = 0.0143
  // vel_integrator_gain = 0
  odrive_serial << "w axis1.motor.config.current_lim " << 60.0f << '\n';
  odrive_serial << "w axis1.encoder.config.cpr " << 42.0f << '\n';
  odrive_serial << "w axis1.controller.config.pos_gain " << 6.1f << '\n';
  odrive_serial << "w axis1.controller.config.vel_gain " << 0.03 << '\n';
  odrive_serial << "w axis1.controller.config.vel_integrator_gain " << 0.06f << '\n';
  
}
