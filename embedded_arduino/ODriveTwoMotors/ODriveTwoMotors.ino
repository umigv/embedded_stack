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

// Arduino Mega or Due - Serial1
// pin 19: RX - connect to ODrive TX
// pin 18: TX - connect to ODrive RX
// See https://www.arduino.cc/reference/en/language/functions/communication/serial/ for other options
HardwareSerial& odrive_serial = Serial1;

// ODrive object
ODriveArduino odrive(odrive_serial);

void setupODrive(ODriveArduino& odrive);
void setupODriveParams(ODriveArduino& odrive);
void closedLoopControl(ODriveArduino& odrive);

// multiplier to make sure velocity cannot change after e stop
int eStopMultiplier = 1;

bool is_autonomous = false;
bool mode_change = true;
float left_vel = 0;
float right_vel = 0;
bool wireless_stop = false;
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

  odrive.SetVelocity(0, left_vel * VEL_TO_RPS * eStopMultiplier);
  odrive.SetVelocity(1, right_vel * VEL_TO_RPS * eStopMultiplier);
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
  attachInterrupt(digitalPinToInterrupt(2), interruptEStop, CHANGE);

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
  odrive_serial.begin(115200);

  // TODO: Should this even be here?
  // Serial to PC
  while (!Serial1); // wait for Arduino Serial Monitor to open

  // Calibrate the ODrive
  setupODriveParams(odrive);
  int requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  //odrive.run_state(0, requested_state, true);
  //odrive.run_state(1, requested_state, true);
  odrive.run_state(0, requested_state, false);
  delay(19000);
  odrive.run_state(1, requested_state, false);
  delay(19000);

  closedLoopControl(odrive);
  Serial.begin(115200);
}

void loop() {
  current_time = millis();

  if (prev_time + pub_period <= current_time)
  {
    encoder_vel_msg.header.stamp = nh.now();
    encoder_vel_msg.header.frame_id = "encoders";

    float left_vel_ = LEFT_POLARITY * odrive.GetVelocity(0) / VEL_TO_RPS;
    float right_vel_ = RIGHT_POLARITY * odrive.GetVelocity(1) / VEL_TO_RPS;
    float linear = (left_vel_ + right_vel_) / 2.0;
    float angular = (right_vel_ - left_vel_) / 2.0;
    encoder_vel_msg.twist.twist.linear.x = linear;
    encoder_vel_msg.twist.twist.angular.z = angular;

    encoder_vel_pub.publish(&encoder_vel_msg);
    
    prev_time = current_time;
    
  }
  // Logic for setting the light
  if (!wireless_stop) {
    if (is_autonomous) {
      if (current_time - prev_blink_time >= blink_interval) {
        prev_blink_time = current_time;

        if (light_on) {
          strip.fill(strip.Color(0, 255, 0), 0, strip.numPixels());
          strip.show();
        }
        else {
          strip.clear();
          strip.show();
        }
        light_on = !light_on;
      }
      mode_change = false;
    }
    else {
      //teleop - solid blue
      if (mode_change) {
      strip.fill(strip.Color(0, 0, 255), 0, strip.numPixels());
        strip.show();
      }
      mode_change = false;
    }
  }
  else {
    strip.fill(strip.Color(255, 0, 0), 0, strip.numPixels());
    strip.show();
    mode_change = false;
  }

  nh.spinOnce();
  delay(1);
}

unsigned long lastTimeStamp = millis();

// Interrupt e-stop function
void interruptEStop(){
  odrive.SetVelocity(0, 0);
  odrive.SetVelocity(1, 0);
  if (digitalRead(2) == HIGH) {
    eStopMultiplier = 0;
    odrive_serial << "w axis0.controller.config.vel_gain " << 0.01 << '\n';
    odrive_serial << "w axis1.controller.config.vel_gain " << 0.01 << '\n';
    wireless_stop = true;
  }
  else if (digitalRead(2) == LOW) {
    eStopMultiplier = 1;
    odrive_serial << "w axis0.controller.config.vel_gain " << 0.07 << '\n';
    odrive_serial << "w axis1.controller.config.vel_gain " << 0.07 << '\n';
    wireless_stop = false;
  }
  mode_change = true;
}

void setupODriveParams(ODriveArduino& odrive) {
  // AXIS0
  odrive_serial << "w axis0.motor.config.current_lim " << 60.0f << '\n';
  odrive_serial << "w axis0.encoder.config.cpr " << 42.0f << '\n';
  odrive_serial << "w axis0.controller.config.pos_gain " << 6.1f << '\n';
  odrive_serial << "w axis0.controller.config.vel_gain " << 0.07 << '\n';
  odrive_serial << "w axis0.controller.config.vel_integrator_gain " << 0.01f << '\n';
  // AXIS1
  odrive_serial << "w axis1.motor.config.current_lim " << 60.0f << '\n';
  odrive_serial << "w axis1.encoder.config.cpr " << 42.0f << '\n';
  odrive_serial << "w axis1.controller.config.pos_gain " << 6.1f << '\n';
  odrive_serial << "w axis1.controller.config.vel_gain " << 0.07 << '\n';
  odrive_serial << "w axis1.controller.config.vel_integrator_gain " << 0.01f << '\n';
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
