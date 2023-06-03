#define USE_USBCON
#include <ros.h>
#include <ros/time.h>

#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

#include "SAMD51_InterruptTimer.h"

float rpm1 = 0;
float rpm2 = 0;

int p1, p2 = 0;

float wheel_base = 0.3  ;//m
float wheel_radius = 0.05 ; //m
float max_rpm = 100 ; //RPM

float req_v = 0.0 ; 
float req_w = 0.0 ;

//================PARAMETER =================
ros::NodeHandle  nh;
std_msgs::Int16 cmd_pub_msg;
ros::Publisher pub_cmd( "/command_feedback", &cmd_pub_msg);


volatile bool control_updated = false;

int motor1_dir_pin = 9;
int motor1_pwm_pin  = 5;
int motor2_dir_pin = 10;
int motor2_pwm_pin  = 6;

//============CODE =======================

void velCb( const geometry_msgs::Twist& vel_msg){
  control_updated = true;

  req_v = vel_msg.linear.x;
  req_w = vel_msg.angular.z;

  rpm1 = (((2 * req_v)-(req_w * wheel_base))/(2 * wheel_radius)) * 9.549297;  //Left Wheel // RPM
  rpm2 = (((2 * req_v)+(req_w * wheel_base))/(2 * wheel_radius)) * 9.549297;  //Right Wheel // RPM

  rpm1 = max(min(rpm1, max_rpm), -max_rpm);
  rpm2 = max(min(rpm2, max_rpm), -max_rpm);

  //Calculate Command Signal
  p1 = int(rpm1 / max_rpm * 100) ;
  p2 = int(rpm2 / max_rpm * 100) ;

   if (rpm1 == 0)
  {
    p1 = 0;
  }

  if (rpm1 == 0)
  {
    p1 = 0;
  }

  // ========== MOTOR COMMAND ==============
  if(p1 > 0)
  {
    digitalWrite(motor1_dir_pin, LOW);
    analogWrite(motor1_pwm_pin, abs(p1));
  }
  else if(p1 < 0)
  {
    digitalWrite(motor1_dir_pin, HIGH);
    analogWrite(motor1_pwm_pin, abs(p1));
  }
  else if(p1 == 0)
  {
    digitalWrite(motor1_dir_pin, HIGH);
    analogWrite(motor1_pwm_pin, 0);
  }

  if(p2 > 0)
  {
    digitalWrite(motor2_dir_pin, LOW);
    analogWrite(motor2_pwm_pin, abs(p2));
  }
  else if(p2 < 0)
  {
    digitalWrite(motor2_dir_pin, HIGH);
    analogWrite(motor2_pwm_pin, abs(p2));
  }
  else if(p2 == 0)
  {
    digitalWrite(motor2_dir_pin, HIGH);
    analogWrite(motor2_pwm_pin, 0);
  }

  //==========Serial1 Print=================
  Serial1.print("set velocity1=");
  Serial1.println(p1);
  Serial1.print("set velocity2=");
  Serial1.println(p2);  

  cmd_pub_msg.data = p2;
  pub_cmd.publish(&cmd_pub_msg);
}

ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", velCb );


void controlISR() {

  control_updated = true;

}

void setup() {

  Serial1.begin(115200);
  
  pinMode(motor1_dir_pin, OUTPUT);
  pinMode(motor1_pwm_pin, OUTPUT);
  pinMode(motor2_dir_pin, OUTPUT);
  pinMode(motor2_pwm_pin, OUTPUT);

  //===========ROS=============
  nh.initNode();

  nh.subscribe(vel_sub);
  nh.advertise(pub_cmd);
  TC.startTimer(1000, controlISR);

}

void loop() {

  if (control_updated) {

    /* No Feedback Control Loop Now**/
    noInterrupts();
    control_updated = false;
    interrupts();
  }

  
  nh.spinOnce();
    
}
