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
HardwareSerial& odrive_serial2 = Serial2; 

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
float left_vel_actual = 0;
float right_vel_actual = 0;

const float DAMPENING_THRESHOLD = 0.1;

// Handling errors
unsigned long prev_error_time = 0;
const unsigned long ERROR_CHECK_TIME = 5000; // 5 seconds

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

// For dampening
bool dampening_on_l = false;
bool dampening_on_r = false;

// Reset: Sets PC to 0
// https://www.instructables.com/two-ways-to-reset-arduino-in-software/
void(* reset_func) (void) = 0;


// =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= ROS FUNCTIONALITY BEGIN =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= //

ros::NodeHandle nh;
//subscriber and callback to recieve velocity messages and move the motors accordingly
void velCallback(const geometry_msgs::Twist& twist_msg) {
  lastData = millis();

  left_vel = LEFT_POLARITY * (twist_msg.linear.x - WHEEL_BASE * twist_msg.angular.z / 2.0);
  right_vel = RIGHT_POLARITY * (twist_msg.linear.x + WHEEL_BASE * twist_msg.angular.z / 2.0);

  // Dampening logic: can only be switched off here
  if (abs(left_vel) >= DAMPENING_THRESHOLD && dampening_on_l) {
    dampening_on_l = false;
    odrive_serial << "w axis0.controller.config.vel_gain " << 0.07 << '\n';
  }
  if (abs(right_vel) >= DAMPENING_THRESHOLD && dampening_on_r) {
    dampening_on_r = false;
    odrive_serial2 << "w axis0.controller.config.vel_gain " << 0.07 << '\n';
  }

  odrive.SetVelocity(0, left_vel * VEL_TO_RPS * eStopMultiplier);
  odrive2.SetVelocity(0, right_vel* VEL_TO_RPS * eStopMultiplier);
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

// =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= ROS FUNCTIONALITY END |==|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= //

// =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= LIGHT-RING BEGIN =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= //

#define PIXEL_PIN 9
#define PIXEL_COUNT 12
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
const long blink_interval = 500;
unsigned long prev_blink_time = 0;
bool light_on = false;
unsigned long current_time = 0;

// =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= LIGHT-RING END =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= //

// =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= SETUP BEGIN =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= //

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
  prev_error_time = 0;
  odrive_serial.begin(115200);
  odrive_serial2.begin(115200);

  // TODO: Should this even be here?
  // Serial to PC
  while (!Serial1 || !Serial2); // wait for Arduino Serial Monitor to open
  // Calibrate the ODrive
  setupODriveParams(odrive);
  setupODriveParams(odrive2);
  int requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
  //odrive.run_state(0, requested_state, true);
  //odrive.run_state(1, requested_state, true);
  odrive.run_state(0, requested_state, false);
  delay(19000);
  odrive2.run_state(0, requested_state, false);

  delay(19000);

  closedLoopControl(odrive);
  closedLoopControl(odrive2); 
  Serial.begin(115200);
}

// =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= SETUP END =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= //

// =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= LOOP BEGIN =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= //

void loop() {
  // ======================================== GENERAL PURPOSE BEGIN ======================================== //
  
  current_time = millis();
  
  // ======================================== GENERAL PURPOSE END ========================================== //

  // ======================================== ENCODER PUBLISHING BEGIN ===================================== //

  if (prev_time + pub_period <= current_time)
  {
    encoder_vel_msg.header.stamp = nh.now();
    encoder_vel_msg.header.frame_id = "encoders";

    // OLD VERSION
    //float left_vel_actual = LEFT_POLARITY * odrive.GetVelocity(0) / VEL_TO_RPS;
    //float right_vel_actual = RIGHT_POLARITY * odrive.GetVelocity(1) / VEL_TO_RPS;

    // NEW VERSION
    odrive_serial << "r axis0.sensorless_estimator.vel_estimate\n";
    left_vel_actual = LEFT_POLARITY * odrive.readFloat() / VEL_TO_RPS;
    odrive_serial2 << "r axis0.sensorless_estimator.vel_estimate\n";
    right_vel_actual = RIGHT_POLARITY * odrive2.readFloat() / VEL_TO_RPS;
    float linear = (left_vel_actual + right_vel_actual) / 2.0;
    float angular = (right_vel_actual - left_vel_actual) / 2.0;
    encoder_vel_msg.twist.twist.linear.x = linear;
    encoder_vel_msg.twist.twist.angular.z = angular;

    encoder_vel_pub.publish(&encoder_vel_msg);
    
    prev_time = current_time;
  }

  // ======================================== ENCODER PUBLISHING END ======================================= //

  // ======================================== ERROR HANDLING BEGIN ========================================= //

  // Error checking and reset if necessary; TODO: Make this more robust
  if (prev_error_time + ERROR_CHECK_TIME <= current_time) {
    bool error_detected = false;
    int errors[2] = { 0, 0 };
    
    // First read the errors


    errors[0] = readErrors(odrive, 0);
    errors[1] = readErrors(odrive2, 0);
    error_detected = errors[0] || errors[1] ? true : false;

    // Then clear the errors if needed
    if (error_detected) {

      // If any motor has an error stop both immediately
      odrive.SetVelocity(0, 0);
      odrive2.SetVelocity(0, 0);
      odrive_serial << "w axis0.controller.config.vel_gain " << 0.01 << '\n';
      odrive_serial2 << "w axis0.controller.config.vel_gain " << 0.01 << '\n';
      dampening_on_l = dampening_on_r = true;
      
      // Set the light purple
      strip.fill(strip.Color(255, 0, 255), 0, strip.numPixels());
      strip.show();

      // Actually clear the errors
      odrive_serial << "sc\n";
      odrive_serial2 << "sc\n";
      if (errors[0]) {
        // Then calibrate the motor that had the error
        odrive.run_state(0, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, false);
        delay(19000);
        odrive.run_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL, false);
      }
      
      if (errors[1]) {
        odrive2.run_state(0, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, false);
        delay(19000);
        odrive2.run_state(0, AXIS_STATE_CLOSED_LOOP_CONTROL, false);          
      }

      // Clear out the purple
      strip.clear();
      mode_change = true;
    }
    prev_error_time = current_time;
  }

  // ======================================== ERROR HANDLING END =========================================== //
  
  // ======================================== LIGHT SETTING BEGIN ========================================== //
  
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

  // ======================================== LIGHT SETTING END ============================================ //

  // ======================================== SOUND DAMPENING BEGIN ======================================== //

  // Logic for dampening the buzzing or not
  // Dampening can only be switched on here.
  // Switching it off happens when a new command is received
  // Dampening should only automatically be switched on if the
  // desired value is close to 0. That's also another condition.
  if (abs(left_vel_actual) < DAMPENING_THRESHOLD && abs(left_vel) < DAMPENING_THRESHOLD && !dampening_on_l) {
    dampening_on_l = true;
    odrive_serial << "w axis0.controller.config.vel_gain " << 0.01 << '\n';
  }

  if (abs(right_vel_actual) < DAMPENING_THRESHOLD && abs(right_vel) < DAMPENING_THRESHOLD && !dampening_on_r) {
    dampening_on_r = true;
    odrive_serial2 << "w axis0.controller.config.vel_gain " << 0.01 << '\n';
  }

  // ======================================== SOUND DAMPENING END ======================================== //

  // ======================================== ROS LOOP CODE BEGIN ======================================== //
  
  nh.spinOnce();
  delay(1);

  // ======================================== ROS LOOP CODE END ========================================== //
}
// =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= LOOP END =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= //

// =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= E-STOP BEGIN =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= //

unsigned long lastTimeStamp = millis();

// Interrupt e-stop function
void interruptEStop(){
  odrive.SetVelocity(0, 0);
  odrive2.SetVelocity(0, 0);
  if (digitalRead(2) == HIGH) {
    eStopMultiplier = 0;
    odrive_serial << "w axis0.controller.config.vel_gain " << 0.01 << '\n';
    odrive_serial2 << "w axis0.controller.config.vel_gain " << 0.01 << '\n';
    dampening_on_l = dampening_on_r = true;
    wireless_stop = true;
  }
  else if (digitalRead(2) == LOW) {
    eStopMultiplier = 1;
    wireless_stop = false;
  }
  mode_change = true;
}

// =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= E-STOP END =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= //

// =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= OTHER UTILITIES BEGIN =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= //

void setupODriveParams(ODriveArduino& odrive) {
  // AXIS0 - odrive
  //odrive_serial << "w axis0.motor.config.current_lim " << 60.0f << '\n';
  odrive_serial << "w axis0.encoder.config.cpr " << 42.0f << '\n';
  odrive_serial << "w axis0.controller.config.pos_gain " << 1.0f << '\n';
  odrive_serial << "w axis0.controller.config.vel_gain " << 0.01f << '\n';
  odrive_serial << "w axis0.controller.config.vel_integrator_gain " << 0.0f << '\n';
  // AXIS0 - odrive2
  //odrive_serial2 << "w axis0.motor.config.current_lim " << 60.0f << '\n';
  odrive_serial2 << "w axis0.encoder.config.cpr " << 42.0f << '\n';
  odrive_serial2 << "w axis0.controller.config.pos_gain " << 1.0f << '\n';
  odrive_serial2 << "w axis0.controller.config.vel_gain " << 0.01f << '\n';
  odrive_serial2 << "w axis0.controller.config.vel_integrator_gain " << 0.0f << '\n';
}

void closedLoopControl(ODriveArduino& odrive) {
  int requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrive.run_state(0, requested_state, false);
  
  odrive.SetVelocity(0, 0);
}

void colorWipe(uint32_t c) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
  }
}

int readErrors(ODriveArduino& odrive, int axis) {
  // TODO: Make this more robust to deal with various errors uniquely
  int error_code = 0;
  odrive_serial << "r axis" << axis << ".error\n";
  error_code |= odrive.readInt();

  odrive_serial << "r axis" << axis << ".motor.error\n";
  error_code |= odrive.readInt();

  odrive_serial << "r axis" << axis << ".sensorless.error\n";
  error_code |= odrive.readInt();

  odrive_serial << "r axis" << axis << ".encoder.error\n";
  error_code |= odrive.readInt();

  odrive_serial << "r axis" << axis << ".controller.error\n";
  error_code |= odrive.readInt();
  return error_code;
}

// =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= OTHER UTILITIES END =|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|=|= //
