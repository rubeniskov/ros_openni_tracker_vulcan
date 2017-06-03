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

// incluir librerias
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif


#include <Servo.h> 
#include <AccelStepper.h>
#include <ros.h>
#include <geometry_msgs/Point32.h>
//inicializar nodo ROS
ros::NodeHandle  nh;

//Inicializar constantes globales
const unsigned long TIME_TO_IDLE = 5000;
const unsigned int STEPS_TO_360 = 3454;

//Inicializar variables globales
unsigned long lastDetection = millis();
unsigned int XAxisDegrees, YAxisDegrees;

AccelStepper XAxisStepper(1,4,5);
Servo YAxisServo;

//Funcion para inicializar los ejes X e Y
void initAxis(){
   XAxisStepper.setAcceleration(1000);
   XAxisStepper.setMaxSpeed(700);
   YAxisServo.attach(10); //attach it to pin 10
   //pinMode(2, INPUT); // Reset position reference
}

unsigned int XAxisDegrees2steps(unsigned int degrees){
   return STEPS_TO_360 / 360 * (degrees % 360);
}

unsigned int XAxisCurrentDegrees(){
   return XAxisDegrees2steps(XAxisStepper.currentPosition());
}

unsigned int XAxisTargetDegrees(){
   return XAxisDegrees2steps(XAxisStepper.targetPosition());
}


void XAxisResetPosition(){
    XAxisStepper.setCurrentPosition(0);
}

void XAxisSpinPositionReference(){
  if(digitalRead(2) == 1) 
      XAxisResetPosition; 
}

//Funcion para movimiento eje X
void moveXAxis(unsigned int degrees, float speed){
    unsigned int steps = XAxisDegrees2steps(degrees);
    // Hasta que no le queden 0 pasos de recorrido hasta llegar al ultimo valor no sigue adelante
    if(XAxisStepper.distanceToGo() == 0) {
      //if(speed > 0) XAxisStepper.setSpeed(speed);
      XAxisStepper.moveTo(steps);
  }
}

void moveXAxis(unsigned int degrees){
    moveXAxis(degrees, 0);
}

//Funcion movimiento eje Y con Servo
void moveYAxis(int degrees){
    YAxisServo.write(degrees % 180);
  
}

//funcion para mover los dos ejes
void moveTo(int degreesX, int degreesY) {
   moveXAxis(degreesX);
   moveYAxis(degreesY);
   
}

void idleState(){
    moveYAxis(0);
    //moveXAxis(0 : XAxisCurrentDegrees() == 0 ? 90, 0.00004);
    moveXAxis(90);
}

// Recibe el mensaje del topic al que nos suscribimos: vulcanuino
void subscriber_cb( const geometry_msgs::Point32& cmd_msg){
  // almacena el valor actual del tiempo en milisegundos
  lastDetection = millis();
  //llamar a la funcion de mover los ejes
  moveTo(cmd_msg.x * 60, cmd_msg.y * 180); 
  //cambia el valor del led para saber que estamos recibiendo mensajes ON/OFF
  digitalWrite(13, HIGH-digitalRead(13));  
}

void setup(){
  pinMode(13, OUTPUT);
  ros::Subscriber<geometry_msgs::Point32> sub("vulcanuino", subscriber_cb);
  initAxis();
  nh.initNode();
  nh.subscribe(sub); 
}

void loop(){
  nh.spinOnce();
  //XAxisSpinPositionReference();
  /*if((millis() - lastDetection) > TIME_TO_IDLE)
      idleState();*/
  moveXAxis(180);
  XAxisStepper.run();
}
