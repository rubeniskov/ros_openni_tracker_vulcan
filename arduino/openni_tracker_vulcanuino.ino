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

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

Servo servoX, servoY;

void servo_cb( const std_msgs::Float32& cmd_msg){
  servoX.write(cmd_msg.data * 180); //set servo angle, should be from 0-180
  servoY.write(cmd_msg.data * 180); //set servo angle, should be from 0-180  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::Float32> sub("chatter", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servoX.attach(9); //attach it to pin 9
  servoY.attach(10); //attach it to pin 10
}

void loop(){
  nh.spinOnce();
  delay(1);
}
