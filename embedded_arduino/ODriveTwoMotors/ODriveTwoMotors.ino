// THIS IS WORKING  WITH ROS

// WARNING: Do not use Serial to print (messes with Ros)

// file pulled from https://github.com/odriverobotics/ODrive/tree/master/Arduino/ODriveArduino
// this file is flashed to the Arduino Mega to control the ODrive via serial/uart
// after flashing set the PC's baud rate to 115200
// for more information read this repo's readme

// includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////

// Below are some sample configurations.
// You can comment out the default Teensy one and uncomment the one you wish to use.
// You can of course use something different if you like
// Don't forget to also connect ODrive GND to Arduino GND.

// Teensy 3 and 4 (all versions) - Serial1
// pin 0: RX - connect to ODrive TX
// pin 1: TX - connect to ODrive RX
// See https://www.pjrc.com/teensy/td_uart.html for other options on Teensy
//HardwareSerial& odrive_serial = Serial1;

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
HardwareSerial& odrive_serial = Serial1;
HardwareSerial& odrive_serial2 = Serial2;

// Arduino without spare serial ports (such as Arduino UNO) have to use software serial.
// Note that this is implemented poorly and can lead to wrong data sent or read.
// pin 8: RX - connect to ODrive TX
// pin 9: TX - connect to ODrive RX
// SoftwareSerial odrive_serial(8, 9);


// ODrive object
ODriveArduino odrive(odrive_serial);
ODriveArduino odrive2(odrive_serial2);

void setupODrive(ODriveArduino& odrive);
void setupODriveParams(ODriveArduino& odrive);
void closedLoopControl(ODriveArduino& odrive);

float left_vel = 0;
float right_vel = 0;
unsigned long lastData = 0;
const float WHEEL_BASE = 0.62;
const float WHEEL_DIAMETER = 0.3;
const long CONTROL_TIMEOUT = 1000;
const bool REVERSE0 = true;
const bool REVERSE1 = false;
float VEL_TO_RPS = 1.0 / (WHEEL_DIAMETER * PI) * 98.0/3.0; 
const float  VEL_LIMIT = 2.235 * VEL_TO_RPS; // 5 mph (2.2 m/s) limit

ros::NodeHandle nh;
void velCallback(const geometry_msgs::Twist& twist_msg) {
  lastData = millis();

  left_vel = twist_msg.linear.x - WHEEL_BASE * twist_msg.angular.z / 2.0;
  right_vel = twist_msg.linear.x + WHEEL_BASE * twist_msg.angular.z / 2.0;

  left_vel = REVERSE0 ? -left_vel : left_vel;
  right_vel = REVERSE1 ? -right_vel : right_vel;

  //TODO: CHECK WHICH ODRIVE WHICH IS
  odrive.SetVelocity(0, int(left_vel * VEL_TO_RPS));
  odrive.SetVelocity(1, int(left_vel * VEL_TO_RPS));
  odrive2.SetVelocity(0, int(right_vel * VEL_TO_RPS));
  odrive2.SetVelocity(1, int(right_vel * VEL_TO_RPS));
}
ros::Subscriber<geometry_msgs::Twist> sub("/teleop/cmd_vel", velCallback);

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);
  odrive_serial2.begin(115200);

  // Serial to PC
  while (!Serial1) ; // wait for Arduino Serial Monitor to open
  while (!Serial2); // wait for Arduino Serial Monitor to open

  setupODriveParams(odrive);
  setupODriveParams(odrive2);
  int requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  odrive.run_state(0, requested_state, false);
  odrive2.run_state(0, requested_state, false);
  delay(19000);
  odrive.run_state(1, requested_state, false);
  odrive2.run_state(1, requested_state, false);
  delay(19000);
  //Serial.println("Finished calibration!");
  closedLoopControl(odrive);
  closedLoopControl(odrive2);
  
  nh.initNode();
  nh.subscribe(sub);
  //nh.getHardware()->setBaud(115200);
}

void loop() {
  nh.spinOnce();

  /*
  if(millis() - lastData >= CONTROL_TIMEOUT) {
    odrive.SetVelocity(0, 0);
    odrive.SetVelocity(1, 0);
    odrive2.SetVelocity(0, 0);
    odrive2.SetVelocity(1, 0);
  }
  */
  
  uint8_t stop = digitalRead(32);
  if (stop) {
    odrive.SetVelocity(0, 0);
    odrive.SetVelocity(1, 0);
    odrive2.SetVelocity(0, 0);
    odrive2.SetVelocity(1, 0);
  }
}

void setupODriveParams(ODriveArduino& odrive) {
  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << VEL_LIMIT << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 60.0f << '\n';
    odrive_serial << "w axis" << axis << ".encoder.config.cpr " << 42.0f << '\n';
    odrive_serial << "w axis" << axis << ".controller.config.pos_gain " << 45.0f << '\n';
    odrive_serial << "w axis" << axis << ".controller.config.vel_gain " << 0.0132556f << '\n';
    odrive_serial << "w axis" << axis << ".controller.config.vel_integrator_gain " << 0.00662781f << '\n';
  }
}

void closedLoopControl(ODriveArduino& odrive) {
  int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.run_state(0, requested_state, false);
  odrive.run_state(1, requested_state, false);
  
  odrive.SetVelocity(0, 0);
  odrive.SetVelocity(1, 0);
}
