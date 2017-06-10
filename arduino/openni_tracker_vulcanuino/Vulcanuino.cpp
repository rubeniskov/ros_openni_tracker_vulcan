#include "Vulcanuino.h"

Vulcanuino::Vulcanuino(unsigned int xAxisStepperAPin, unsigned int xAxisStepperBPin, unsigned int xAxisRefPin, unsigned int yAxisServoPin) {
    this->_xAxisStepper = AccelStepper(1, D6, D7);
    this->_yAxisServo = Servo();
    this->_xAxisRefPin = xAxisRefPin;
    // this->_yAxisServo.attach(10);
}
//MAX SPEED 300
//MID SPEED 200
//LOW SPEED 100
Vulcanuino::~Vulcanuino(void) {

}

void Vulcanuino::begin(){
    pinMode(D4, INPUT); // X AXIS REF;
    // pinMode(9, INPUT); // SWITCH
    this->_oledDisplay.setAutoPageClear(0);
    this->_oledDisplay.begin();
    this->_oledDisplay.setFont(u8g2_font_7x14_tf);
    this->_oledDisplay.setFontDirection(0);
    this->_oledDisplay.firstPage();
    //Serial.begin(9600);
}

void Vulcanuino::axisInitialization(){
    if(!(this->_state & Vulcanuino::STATE_INITIALIZED)){
        if(this->_phase == Vulcanuino::PHASE_INITIALIZING){
            this->_xAxisStepper.setAcceleration(1000);
            this->_xAxisStepper.setMaxSpeed(300);
            // kalmanX.setAngle(0);
            this->_phase &= ~Vulcanuino::PHASE_INITIALIZING;
            this->_state |= Vulcanuino::STATE_INITIALIZED;
        } else if(this->_phase == 0) {
            this->_phase = Vulcanuino::PHASE_INITIALIZING;
        }
    }
}

void Vulcanuino::axisCalibration(){
    if(this->_state & Vulcanuino::STATE_INITIALIZED && !(this->_state & Vulcanuino::STATE_CALIBRATED)){
        if(this->_phase == Vulcanuino::PHASE_CALIBRATING){
            bool dircw = true;
            while (digitalRead(D4) == HIGH){
                if(dircw && this->xAxisCurrentDegrees() < 90){
                    this->_xAxisStepper.move(10);
                    dircw = false;
                } else {
                    this->_xAxisStepper.move(-10);
                }
                this->_xAxisStepper.run();
            }
            // this->_xAxisStepper.setCurrentPosition(0);
            // this->_xAxisStepper.runToNewPosition((dircw ? 1 : -1) * 40);
            this->_xAxisStepper.setCurrentPosition(this->xAxisDegrees2steps(180));
            this->_phase &= ~Vulcanuino::PHASE_CALIBRATING;
            this->_state |= Vulcanuino::STATE_CALIBRATED;
        } else if(this->_phase == 0) {
            this->_phase = Vulcanuino::PHASE_CALIBRATING;
        }
    }
}

void Vulcanuino::axisIdle() {
    if(this->_state == Vulcanuino::STATE_READY) {
        if(this->_phase == Vulcanuino::PHASE_IDLING){
            this->_xAxisStepper.setMaxSpeed(100);
            this->xAxisMoveTo(this->xAxisCurrentDegrees() >= 225 ? 135 : 225);
        } else if(this->_phase == 0) {
            this->_phase = Vulcanuino::PHASE_IDLING;
        }
    }
}

void Vulcanuino::axisManualControl() {
    // Serial.println(digitalRead(9));
    // Serial.println(analogRead(0));
    // Serial.println(analogRead(1) / 255.0 * 360);

    if(this->_phase == Vulcanuino::PHASE_MANUAL){
        while (digitalRead(9) == 1){
            // uint8_t x = analogRead(1);
            // uint8_t y = analogRead(0);
            // this->xAxisMoveTo(map(analogRead(1), 0, 1023, 0, 360));
            // this->_yAxisServo.write(map(analogRead(0), 0, 1023, 0, 180));
        }
        this->_phase &= ~Vulcanuino::PHASE_MANUAL;
    }

    if(this->_state == Vulcanuino::STATE_READY && this->_phase == 0 && digitalRead(9) == 1)
        this->_phase = Vulcanuino::PHASE_MANUAL;
}

unsigned int Vulcanuino::xAxisDegrees2steps(unsigned int degrees){
    return floor((float)(Vulcanuino::STEPS_TO_360 / 360.0) * degrees);
}

unsigned int Vulcanuino::xAxisSteps2Degrees(unsigned int steps){
    return ceil((float)(360.0 / Vulcanuino::STEPS_TO_360) * steps);
}

unsigned int Vulcanuino::xAxisCurrentDegrees(){
    return this->xAxisSteps2Degrees(this->_xAxisStepper.currentPosition());
}

unsigned int Vulcanuino::xAxisTargetDegrees(){
    return this->xAxisSteps2Degrees(this->_xAxisStepper.targetPosition());
}

bool Vulcanuino::xAxisMoveTo(unsigned int degrees){
    //if(this->_state & Vulcanuino::STATE_READY) {
        unsigned int steps = this->xAxisDegrees2steps(degrees);
        
        
        if(this->_xAxisStepper.distanceToGo() == 0) {
            this->_xAxisStepper.moveTo(steps);
        }
        this->_xAxisStepper.run();
    //}
    return false;
}

bool Vulcanuino::yAxisMoveTo(unsigned int degrees){
    this->_yAxisServo.write(degrees % 180);
}

bool Vulcanuino::axisMoveTo(unsigned int degreesX, unsigned int degreesY){
    this->xAxisMoveTo(degreesX);
    this->yAxisMoveTo(degreesY);
}

void Vulcanuino::menuSettings(){
    // this->_oledDisplay.setFont(u8g2_font_7x14_tf);
    // this->_oledDisplay.setFontRefHeightAll();
    // this->_oledDisplay.userInterfaceSelectionList("SETTINGS", 1, "CALIBRATE\nTRIM\nEXIT");
}

void Vulcanuino::menuSettingsTrim(){
    // this->_oledDisplay.setFont(u8g2_font_7x14_tf);
    // //this->_oledDisplay.setFontRefHeightAll();
    // this->_oledDisplay.userInterfaceSelectionList("TRIMMING", 1, "X Axis 0\nY Axis -1\nEXIT");
}

void Vulcanuino::draw(){

        // PHASE_CALIBRATING
        // PHASE_IDLING
        // PHASE_SETTING
        // PHASE_TRACKING:
    do {
    switch(this->_phase){
        case Vulcanuino::PHASE_INITIALIZING:
            this->_oledDisplay.clearBuffer();
            this->_oledDisplay.setFont(u8g2_font_7x14_tf);
            this->_oledDisplay.drawStr(30, 30, "Vulcanuino");
        break;
        case Vulcanuino::PHASE_CALIBRATING:
            this->_oledDisplay.clearBuffer();
            this->_oledDisplay.setFont(u8g2_font_7x14_tf);
            this->_oledDisplay.drawStr(30, 30,"CALIBRATING");
        break;
        case Vulcanuino::PHASE_MANUAL:
            this->_oledDisplay.clearBuffer();
            this->_oledDisplay.setFont(u8g2_font_7x14_tf);
            this->_oledDisplay.drawStr(30, 30,"MANUAL");
        break;
        default:
            this->_oledDisplay.clearBuffer();
            this->_oledDisplay.setFont(u8g2_font_7x14_tf);
            this->_oledDisplay.drawStr(0,15,"X Axis:");
            this->_oledDisplay.drawStr(64,15, u8g2_u8toa(this->xAxisTargetDegrees(), 3));
            this->_oledDisplay.drawStr(0,30,"Y Axis:");
            this->_oledDisplay.drawStr(64,30, u8g2_u8toa(0, 3));
            this->_oledDisplay.drawStr(0,45,"S:");
            this->_oledDisplay.drawStr(16,45, u8g2_u8toa(this->_state, 1));
            this->_oledDisplay.drawStr(60,45,"P:");
            this->_oledDisplay.drawStr(76,45, u8g2_u8toa(this->_phase, 1));
        break;
    }
    } while(this->_oledDisplay.nextPage());
}

void Vulcanuino::run() {
    this->axisInitialization();
    //this->axisCalibration();
    // this->axisManualControl();
    //this->axisIdle();
    // byte  packet[2];
    // uint16_t xAxisValue = 0;
    // uint8_t yAxisValue = 0;
    //
    // if (Serial.available() >= 2) {
    //     Serial.readBytes(packet, 2);
    //     xAxisValue = packet[0] << 1 | packet[1] & 1;
    //     yAxisValue = packet[1] >> 1;
    //     // Serial.println(xAxisValue, DEC);
    //     // Serial.println(yAxisValue, DEC);
    //     this->xAxisMoveTo(xAxisValue);
    //     this->_xAxisStepper.runToNewPosition(xAxisValue);
    // } else {
    //     // this->axisIdle();
    // }
    //if(!this->_xAxisStepper.isRunning())this->draw();
}
