/*
   rosserial Servo Control Example

   This sketch demonstrates the control of hobby R/C servos
   using ROS and the arduiono

   For the full tutorial write up, visit
   www.ros.org/wiki/rosserial_arduino_demos

   For more information on the Arduino Servo Library
   Checkout :
   http://www.arduino.cc/en/Reference/Servo
*/

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Adafruit_SH1106.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/UInt32.h>

#define OLED_RESET 4
Adafruit_SH1106 display(OLED_RESET);
Servo servoX, servoY;
//ros::NodeHandle nh;
unsigned long lastDetection = millis();
const unsigned long TIME_TO_IDLE = 5000;

void servo_cb( const std_msgs::UInt32& cmd_msg) {
  lastDetection = millis();
  moveXAxis(cmd_msg.data);
  moveYAxis(cmd_msg.data);
  digitalWrite(13, HIGH - digitalRead(13)); //toggle led
}

void resetPosition() {
  servoX.write(0);
  servoY.write(0);
}

void moveXAxis(uint16_t degrees){
    degrees = degrees % 180;
    //servoX.write(degrees);

    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(1,32);
    char buffer[50];
    sprintf(buffer, "YAxis %d", degrees);
    display.println(buffer);
    display.display();
}

void moveYAxis(uint16_t degrees){
    degrees = degrees % 180;
    //servoY.write(degrees);  
        
    display.setTextSize(2);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(1,1);
    char buffer[50];
    sprintf(buffer, "XAxis %3d", degrees);
    display.println(buffer);
    display.display();
}

/*void moveTo(unsigned int degreesX, unsigned int degreesY) {
   moveXAxis(degreesX);
}*/




void setup() {
  //ros::NodeHandle  nh;  
  //ros::Subscriber<std_msgs::UInt32> sub("vulcanuino", servo_cb);
  servoX.attach(9); //attach it to pin 9
  servoY.attach(10); //attach it to pin 10
  pinMode(13, OUTPUT);

  display.begin(SH1106_SWITCHCAPVCC, 0x3C);
  display.display();
  //nh.initNode();
  //nh.subscribe(sub);

  int li =0;
  moveXAxis(0);
  while (true) {
      //nh.spinOnce();
      li = (li + 1) % (180 * 2);
      moveYAxis(li >= 180 ? 180 - li - 180 : li);
  }
  //moveYAxis(0);
  /*servoX.attach(9); //attach it to pin 9
  servoY.attach(10); //attach it to pin 10
  pinMode(13, OUTPUT);

  display.begin(SH1106_SWITCHCAPVCC, 0x3C);
  display.display();
  int li =0;
  moveXAxis(0);
  moveYAxis(0);
  while (true) {
    //nh.spinOnce();
    delay(1);
    moveXAxis(li = (li + 1) % 360);
    if (millis() - lastDetection > TIME_TO_IDLE) {
        moveXAxis(0);
        moveYAxis(0);
    }
  }*/
}

void loop() {}
