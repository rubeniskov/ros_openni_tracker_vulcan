/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <AccelStepper.h>
#include <ros.h>
#include <geometry_msgs/Point32.h>

ros::NodeHandle  nh;

unsigned long lastDetection = millis();
const unsigned long TIME_TO_IDLE = 5000;

AccelStepper XAxisStepper(1,4,5);


void initAxis(){
   XAxisStepper.setAcceleration(1000);
   XAxisStepper.setMaxSpeed(700);
}
void moveXAxis(int degrees){
    if(XAxisStepper.distanceToGo() == 0){
      XAxisStepper.moveTo(360 - (3454 / 360) * degrees);
  }
  
}

void moveTo(int degreesX, int degreesY) {
   moveXAxis(degreesX);
}


void servo_cb( const geometry_msgs::Point32& cmd_msg){
  lastDetection = millis();
  moveTo(cmd_msg.x * 60, 0);
  //servoX.write(cmd_msg.x * 180); //set servo angle, should be from 0-180
  //servoY.write(cmd_msg.y * 180); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<geometry_msgs::Point32> sub("vulcanuino", servo_cb);

void resetPosition(){
  //servoX.write(0);
  //servoY.write(0);
}

void setup(){
  pinMode(13, OUTPUT);

  initAxis();
  nh.initNode();
  nh.subscribe(sub);
  //servoX.attach(9); //attach it to pin 9
  //servoY.attach(10); //attach it to pin 10
  
  
}
bool dir = false;
void loop(){
  nh.spinOnce();
  if(millis() - lastDetection > TIME_TO_IDLE)
      moveTo(0, 0);
      
  //moveTo(360, 0);
  XAxisStepper.run();
}
