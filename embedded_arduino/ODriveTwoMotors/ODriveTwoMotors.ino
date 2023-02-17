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
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <Adafruit_NeoPixel.h>

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

// multiplier to make sure velocity cannot change after e stop
int eStopMultiplier = 1;

bool is_autonomous = false;
bool mode_change = true;
float left_vel = 0;
float right_vel = 0;
uint8_t wireless_stop = 0;
unsigned long lastData = 0;
const float WHEEL_BASE = 0.62;
const float WHEEL_DIAMETER = 0.3;
const long CONTROL_TIMEOUT = 1000;
const int8_t LEFT_POLARITY = -1;
const int8_t RIGHT_POLARITY = 1;
float VEL_TO_RPS = 1.0 / (WHEEL_DIAMETER * PI) * 98.0/3.0; 
const float RPS_LIMIT = 20;
const float  VEL_LIMIT = RPS_LIMIT / VEL_TO_RPS; // 1.2 mph (~0.57 m/s) limit

//------ ROS STUFF --------------
ros::NodeHandle nh;
//subscriber and callback to recieve velocity messages and move the motors accordingly
void velCallback(const geometry_msgs::Twist& twist_msg) {
  lastData = millis();

  left_vel = LEFT_POLARITY * (twist_msg.linear.x - WHEEL_BASE * twist_msg.angular.z / 2.0);
  right_vel = RIGHT_POLARITY * (twist_msg.linear.x + WHEEL_BASE * twist_msg.angular.z / 2.0);

  //TODO: CHECK WHICH ODRIVE WHICH IS
  if (!wireless_stop)
  {
    odrive.SetVelocity(0, left_vel * VEL_TO_RPS * eStopMultiplier);
    odrive.SetVelocity(1, left_vel * VEL_TO_RPS * eStopMultiplier);
    odrive2.SetVelocity(0, right_vel * VEL_TO_RPS * eStopMultiplier);
    odrive2.SetVelocity(1, right_vel * VEL_TO_RPS * eStopMultiplier);
  }
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", velCallback);

//subscriber and callback to recieve velocity command source info for status light
void cmdVelSourceCallback(const std_msgs::String& msg) {

  if (msg.data[0] == 'a') {
    is_autonomous = true;
  }
  else {
    is_autonomous = false;
  }

  mode_change = true;
  
}
ros::Subscriber<std_msgs::String> cmd_vel_source_sub("/cmd_vel_source", cmdVelSourceCallback);

//publisher to publish current motor velocity and position (for odometry in the sensor stack)
geometry_msgs::TwistWithCovarianceStamped encoder_vel_msg;
ros::Publisher encoder_vel_pub("/encoders/twist", &encoder_vel_msg);
/*
sensor_msgs::JointState encoder_vel_msg;
ros::Publisher encoder_vel_pub("/encoders/state", &encoder_vel_msg);
char left_joint_name[12] = "left_wheel";
char right_joint_name[12] = "right_wheel";
char* joint_names[2];
float joint_vels[2] = {0.0, 0.0};
float joint_pos[2] = {0.0, 0.0};
*/


unsigned long pub_period = 100; //ms between publish
unsigned long prev_time;

//----- ADAFRUIT LIGHT ------
#define PIXEL_PIN 9
#define PIXEL_COUNT 12
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
const long blink_interval = 500;
unsigned long prev_blink_time = 0;
bool light_on = false;

unsigned long current_time = 0;

void setup() {

  //set up interrupt
  attachInterrupt(digitalPinToInterrupt(2),interruptEStop,CHANGE);

  //set up light
  strip.begin();
  strip.show();
  
  //set up ROS stuff
  nh.initNode();
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(cmd_vel_source_sub);
  nh.advertise(encoder_vel_pub);
  for (int i = 0; i < 36; ++i) {
    encoder_vel_msg.twist.covariance[i] = 0;
  }
  prev_time = 0;
  /*
  joint_names[0] = left_joint_name;
  joint_names[1] = right_joint_name;
  */
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);
  odrive_serial2.begin(115200);

  // Serial to PC
  while (!Serial1) ; // wait for Arduino Serial Monitor to open
  while (!Serial2); // wait for Arduino Serial Monitor to open

  setupODriveParams(odrive);
  setupODriveParams2(odrive2);
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
  Serial.begin(115200);
}

void loop() {
  odrive.SetVelocity(1, 1 * VEL_TO_RPS * eStopMultiplier);
  //Serial.println(eStopMultiplier);
  current_time = millis();

  if (prev_time + pub_period <= current_time)
  {
    /*
    joint_vels[0] = LEFT_POLARITY * odrive.GetVelocity(0) / VEL_TO_RPS;
    joint_vels[1] = RIGHT_POLARITY * odrive2.GetVelocity(0) / VEL_TO_RPS;
    joint_pos[0] = LEFT_POLARITY * odrive.GetPosition(0) / VEL_TO_RPS;
    joint_pos[1] = RIGHT_POLARITY * odrive2.GetPosition(0) / VEL_TO_RPS;

    encoder_vel_msg.name = joint_names;
    encoder_vel_msg.name_length = 2;
    encoder_vel_msg.velocity = joint_vels;
    encoder_vel_msg.velocity_length = 2;
    encoder_vel_msg.position = joint_pos;
    encoder_vel_msg.position_length = 2;

    encoder_vel_pub.publish(&encoder_vel_msg);
    */
    encoder_vel_msg.header.stamp = nh.now();
    encoder_vel_msg.header.frame_id = "encoders";

    float left_vel_ = LEFT_POLARITY * odrive.GetVelocity(0) / VEL_TO_RPS;
    float right_vel_ = RIGHT_POLARITY * odrive2.GetVelocity(0) / VEL_TO_RPS;
    float linear = (left_vel_ + right_vel_) / 2.0;
    float angular = (right_vel_ - left_vel_) / 2.0;
    encoder_vel_msg.twist.twist.linear.x = linear;
    encoder_vel_msg.twist.twist.angular.z = angular;

    encoder_vel_pub.publish(&encoder_vel_msg);
    
    prev_time = current_time;
    
  }

  //blink green if autonomous, solid blue if teleop

  if (!wireless_stop)
  {
    if (is_autonomous)
    {
      if (current_time - prev_blink_time >= blink_interval) 
      {
        prev_blink_time = current_time;

        if (light_on)
        {
          strip.fill(strip.Color(0, 255, 0), 0, strip.numPixels());
          strip.show();
        }
        else
        {
          strip.clear();
          strip.show();
        }
        light_on = !light_on;
      }
      mode_change = false;
    }
    else
    {
      //teleop - solid blue
      if (mode_change) {
      strip.fill(strip.Color(0, 0, 255), 0, strip.numPixels());
        strip.show();
      }
      mode_change = false;
    }
  }
  else
  {
    strip.fill(strip.Color(255, 0, 0), 0, strip.numPixels());
    strip.show();
    mode_change = true;
  }

  //check wireless e stop
  /*
  wireless_stop = digitalRead(32);
  if (wireless_stop) {
    odrive.SetVelocity(0, 0);
    odrive.SetVelocity(1, 0);
    odrive2.SetVelocity(0, 0);
    odrive2.SetVelocity(1, 0);
  }
  */
  nh.spinOnce();
  delay(1);
}
unsigned long lastTimeStamp = millis();
//interrupt e stop funcition
void interruptEStop(){
  if((millis() - lastTimeStamp) >= 1000){
    odrive.SetVelocity(0, 0);
    odrive.SetVelocity(1, 0);
    odrive2.SetVelocity(0, 0);
    odrive2.SetVelocity(1, 0);
    if(eStopMultiplier==0){
      eStopMultiplier = 1;
    }
    else if(eStopMultiplier==1){
      eStopMultiplier = 0;
    }
    wireless_stop = !wireless_stop;
    lastTimeStamp = millis();
  }
}

void setupODriveParams(ODriveArduino& odrive) {
  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << RPS_LIMIT << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 60.0f << '\n';
    odrive_serial << "w axis" << axis << ".encoder.config.cpr " << 42.0f << '\n';
    odrive_serial << "w axis" << axis << ".controller.config.pos_gain " << 45.0f << '\n';
    odrive_serial << "w axis" << axis << ".controller.config.vel_gain " << 0.004 << '\n';
    odrive_serial << "w axis" << axis << ".controller.config.vel_integrator_gain " << 0.00662781f << '\n';
  }
}

void setupODriveParams2(ODriveArduino& odrive) {
  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << RPS_LIMIT << '\n';
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

void colorWipe(uint32_t c) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
  }
}
