//For this ISC3004 encoder https://www.phidgets.com/?&prodid=403
//Majority of the code is stolen from the following url
//https://forum.arduino.cc/t/how-to-calculate-speed-with-incremental-rotary-encoder/690091
//Added functionality that determines the distance and right_velocity
//Is a ROS publisher node called "encoder_vel_pub", to topic "/encoders/twist"

//SO MANY GLOBALS lol, too lazy to clean it up

//includes
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <sensor_msgs/JointState.h>

//ros stuff
ros::NodeHandle nh;
geometry_msgs::TwistWithCovarianceStamped encoder_vel_msg;
ros::Publisher encoder_vel_pub("/encoders/twist", &encoder_vel_msg);
unsigned long pub_period = 100; //ms between publish
float prev_pub_time = 0;

//Globals
volatile int long right_encoder_tick = 0;
float right_prev_time = 0;
float right_curr_time;
float right_prev_dist = 0;
float right_curr_dist = 0;
float right_vel = 0;

volatile int long left_encoder_tick = 0;
float left_prev_time = 0;
float left_curr_time;
float left_prev_dist = 0;
float left_curr_dist = 0;
float left_vel = 0;

//constants for the robot
double TICK_PER_REV = 720;
double WHEEL_DIAM = 0.312928; //12.32 inches in meter
int left_GREEN_PIN = 2;    //left A
int left_WHITE_PIN = 3;    //left B
int right_GREEN_PIN = 21;    //right A
int right_WHITE_PIN = 20;    //right B

void setup() {
    //set up pin modes
    pinMode(right_GREEN_PIN, INPUT_PULLUP); // right green
    pinMode(right_WHITE_PIN, INPUT_PULLUP); // right white
    pinMode(left_GREEN_PIN, INPUT_PULLUP); // left green
    pinMode(left_WHITE_PIN, INPUT_PULLUP); // left white

    Serial.begin(57600);
    //set up interrupt pins
    attachInterrupt(digitalPinToInterrupt(right_GREEN_PIN), ai0_right, RISING);
    attachInterrupt(digitalPinToInterrupt(right_WHITE_PIN), ai1_right, RISING);
    attachInterrupt(digitalPinToInterrupt(left_GREEN_PIN), ai0_left, RISING);
    attachInterrupt(digitalPinToInterrupt(left_WHITE_PIN), ai1_left, RISING);

    //init some vars
    right_curr_time = millis();
    left_curr_time = millis();
    
    //uncomment this section when you are ready for ROS
    
    //init ros
     nh.initNode();
     nh.advertise(encoder_vel_pub);
     for (int i = 0; i < 36; ++i) {
       encoder_vel_msg.twist.covariance[i] = 0;
     }
     //while (!Serial1); // wait for Arduino Serial Monitor to open
     
}

void loop() {
  
    //keep these here if you need instantaneous velocity
    update_right_dist_time_vel();
    update_left_dist_time_vel();

  //comment out when not debugging, currently right vel is not working 
  //Serial.println(right_vel, 10); 
  //Serial.println(left_vel, 10);

  //maybe need a noInterrupts() function here to protect from weird behavior
  //but it should be fine cause vel only update once per loop

  
  if (prev_pub_time + pub_period <= millis()){
      //uncomment this part and comment the above update functions if you need
      //average velocity
      
      /*
      update_right_dist_time_vel();
      update_left_dist_time_vel();
      */
      noInterrupts();
      encoder_vel_msg.header.stamp = nh.now();
      encoder_vel_msg.header.frame_id = "encoders";

      float linear = (left_vel + right_vel) / 2.0;
      float angular = (right_vel - left_vel) / 2.0;
      encoder_vel_msg.twist.twist.linear.x = linear;
      encoder_vel_msg.twist.twist.angular.z = angular;
      encoder_vel_pub.publish(&encoder_vel_msg);
      prev_pub_time = millis();
      nh.spinOnce();
      interrupts();
      
  }

}

void ai0_right() {
  if (digitalRead(right_WHITE_PIN) == HIGH && digitalRead(right_GREEN_PIN) == LOW) {
    right_encoder_tick ++;
  }
  else {
    right_encoder_tick --;
  }

  if (digitalRead(right_WHITE_PIN) == LOW && digitalRead(right_GREEN_PIN) == HIGH) {
    right_encoder_tick ++;
  }
  else {
    right_encoder_tick --;
  }
}
void ai1_right() {
  if (digitalRead(right_GREEN_PIN) == LOW && digitalRead(right_WHITE_PIN) == HIGH) {
    right_encoder_tick --;
  }
  else {
    right_encoder_tick ++;
  }

  if (digitalRead(right_GREEN_PIN) == HIGH && digitalRead(right_WHITE_PIN) == LOW) {
    right_encoder_tick --;
  }
  else {
    right_encoder_tick ++;
  }
}

void ai0_left() {
  if (digitalRead(left_WHITE_PIN) == HIGH && digitalRead(left_GREEN_PIN) == LOW) {
    left_encoder_tick ++;
  }
  else {
    left_encoder_tick --;
  }

  if (digitalRead(left_WHITE_PIN) == LOW && digitalRead(left_GREEN_PIN) == HIGH) {
    left_encoder_tick ++;
  }
  else {
    left_encoder_tick --;
  }
 
}
void ai1_left() {
  if (digitalRead(left_GREEN_PIN) == LOW && digitalRead(left_WHITE_PIN) == HIGH) {
    left_encoder_tick --;
  }
  else {
    left_encoder_tick ++;
  }

  if (digitalRead(left_GREEN_PIN) == HIGH && digitalRead(left_WHITE_PIN) == LOW) {
    left_encoder_tick --;
  }
  else {
    left_encoder_tick ++;
  }
}

void update_right_dist_time_vel(){
    right_prev_dist = right_curr_dist;
    right_curr_dist = right_encoder_tick / TICK_PER_REV * (PI * WHEEL_DIAM);
    right_prev_time = right_curr_time;
    right_curr_time = millis();
    right_vel = (right_curr_dist - right_prev_dist) / (right_curr_time - right_prev_time) * 1000; 
 }


void update_left_dist_time_vel(){
    left_prev_dist = left_curr_dist;
    left_curr_dist = left_encoder_tick / TICK_PER_REV * (PI * WHEEL_DIAM);
    left_prev_time = left_curr_time;
    left_curr_time = millis();
    left_vel = -1 * (left_curr_dist - left_prev_dist) / (left_curr_time - left_prev_time) * 1000; //idk why the encoders are flipped
 }
