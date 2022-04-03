[200~#include <ArduinoHardware.h>
#include <ArduinoTcpHardware.h>
#include <ros.h>

#include <geometry_msgs/Twist.h>

float WHEEL_BASE = 0.62;


float left_vel = 0;
float right_vel = 0;

ros::NodeHandle nh;
void velCallback(const geometry_msgs::Twist& twist_msg) {

  left_vel = twist_msg.linear.x - WHEEL_BASE * twist_msg.angular.z / 2.0;
  right_vel = twist_msg.linear.x + WHEEL_BASE * twist_msg.angular.z / 2.0;
  
  
}
ros::Subscriber<geometry_msgs::Twist> sub("/teleop/cmd_vel", velCallback);

unsigned long pub_period = 100;//ms between publishing
unsigned long prev_time;

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(sub);

  prev_time = 0;

}

void loop() {
  // put your main code here, to run repeatedly:


  //TODO O-drive motor control here


  //TODO publish encoder data
  if (prev_time + pub_period <= millis()) {
    //publish stuff eventually
    prev_time = millis();
  }
  
  nh.spinOnce();
  delay(1);

}
