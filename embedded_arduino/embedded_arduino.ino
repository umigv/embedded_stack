#include <ArduinoHardware.h>
#include <ArduinoTcpHardware.h>
#include <ros.h>

#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

#include <geometry_msgs/Twist.h>


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
//ODriveArduino odrive(odrive_serial);
ODriveArduino odrive2(odrive_serial2);

void setupODrive(ODriveArduino& odrive);
void setupODriveParams(ODriveArduino& odrive);
void closedLoopControl(ODriveArduino& odrive);

template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

float WHEEL_BASE = 0.62;
const long CONTROL_TIMEOUT = 1000;
const bool REVERSE0 = false;
const bool REVERSE1 = true;

float left_vel = 0;
float right_vel = 0;
unsigned long lastData = 0;


ros::NodeHandle nh;
void velCallback(const geometry_msgs::Twist& twist_msg) {
  lastData = millis();

  left_vel = twist_msg.linear.x - WHEEL_BASE * twist_msg.angular.z / 2.0;
  right_vel = twist_msg.linear.x + WHEEL_BASE * twist_msg.angular.z / 2.0;

  left_vel = REVERSE0 ? -left_vel : left_vel;
  right_vel = REVERSE1 ? -right_vel : right_vel;

  //TODO: CHECK WHICH ODRIVE WHICH IS
  odrive.SetVelocity(0, int(right_vel / (PI * 0.3)) );
  odrive.SetVelocity(1, int(left_vel / (PI * 0.3)) );
}
ros::Subscriber<geometry_msgs::Twist> sub("/teleop/cmd_vel", velCallback);

unsigned long pub_period = 100;//ms between publishing
unsigned long prev_time;

void setup() {
  // put your setup code here, to run once:
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 30.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 60.0f << '\n';
  }

  int requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  odrive.run_state(0, requested_state, false);
  delay(19000);
  odrive.run_state(1, requested_state, false);
  delay(19000);

  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;

  odrive.run_state(0, requested_state, false);
  odrive.run_state(1, requested_state, false);
  
  odrive.SetVelocity(0, 0);
  odrive.SetVelocity(1, 0);

  nh.initNode();
  nh.subscribe(sub);

  prev_time = 0;
}

void loop() {
  nh.spinOnce();

  if(millis() - lastData >= CONTROL_TIMEOUT) {
    odrive.SetVelocity(0, 0);
    odrive.SetVelocity(1, 0);
  }

  //TODO publish encoder data
  if (prev_time + pub_period <= millis()) {
    //publish stuff eventually
    prev_time = millis();
  }
}

// we call this if we are screwed
void setupODriveParams(ODriveArduino& odrive) {
  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 30.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 60.0f << '\n';
  }
}

void closedLoopControl(ODriveArduino& odrive) {
  int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.run_state(0, requested_state, false);
  odrive.run_state(1, requested_state, false);
  
  odrive.SetVelocity(0, 0);
  odrive.SetVelocity(1, 0);
}
