#include "Vulcanuino.h"

Vulcanuino::Vulcanuino(unsigned int xAxisStepperAPin, unsigned int xAxisStepperBPin, unsigned int xAxisRefPin, unsigned int yAxisServoPin) {
    this->_xAxisStepper = AccelStepper(1, D6, D7);
    this->_yAxisServoA = Servo();
    this->_yAxisServoB = Servo();
    this->_yAxisServoA.attach(D0);
    this->_yAxisServoB.attach(D3);
    this->_xAxisRefPin = xAxisRefPin;
    this->_lastMessageTrackingTimestamp = millis();
}
//MAX SPEED 300
//MID SPEED 200
//LOW SPEED 100
Vulcanuino::~Vulcanuino(void) {

}

void Vulcanuino::begin(){
    pinMode(D4, INPUT); // X AXIS REF;
    this->_oledDisplay.setAutoPageClear(0);
    this->_oledDisplay.begin();
    this->_oledDisplay.setFont(u8g2_font_7x14_tf);
    this->_oledDisplay.setFontDirection(0);
    this->_oledDisplay.firstPage();
}

void Vulcanuino::axisInitialization(){
    if(!(this->_state & Vulcanuino::STATE_INITIALIZED)){
        if(this->_phase == Vulcanuino::PHASE_INITIALIZING){
            this->_xAxisStepper.setAcceleration(300);
            this->_xAxisStepper.setMaxSpeed(100);
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
            if (digitalRead(D4) == HIGH){
                // this->_xAxisStepper.setMaxSpeed(1000);
                //this->_xAxisStepper.move(100);
                this->_xAxisStepper.setMaxSpeed(200);
                this->xAxisMoveTo(this->xAxisCurrentDegrees() >= 360 ? 0 : 360);
                this->yAxisMoveTo(90);
            } else {
                this->_xAxisStepper.setCurrentPosition(this->xAxisDegrees2steps(180));
                this->_phase &= ~Vulcanuino::PHASE_CALIBRATING;
                this->_state |= Vulcanuino::STATE_CALIBRATED;
            }
        } else if(this->_phase == 0) {
            this->_xAxisStepper.setCurrentPosition(this->xAxisDegrees2steps(0));
            this->_phase = Vulcanuino::PHASE_CALIBRATING;
        }
    }
    // else if(digitalRead(D4) == LOW){
    //       //this->_xAxisStepper.setCurrentPosition(this->xAxisDegrees2steps(180));
    // }
}

void Vulcanuino::trackUser(unsigned int degreesX, unsigned int degreesY) {
    this->_lastMessageTrackingTimestamp = millis();
    if(this->_phase != Vulcanuino::PHASE_TRACKING){
        this->_xAxisStepper.setMaxSpeed(300);
        this->_xAxisStepper.stop();
    }
    this->axisMoveTo(degreesX, degreesY);
    this->_phase = Vulcanuino::PHASE_TRACKING;
}

void Vulcanuino::axisIdle() {
    if(this->_state == Vulcanuino::STATE_READY) {
        if(this->_phase == Vulcanuino::PHASE_IDLING){
            this->_xAxisStepper.setMaxSpeed(300);
                                                                // 135
            this->xAxisMoveTo(this->xAxisCurrentDegrees() >= 225 ? 135 : 225);
            this->yAxisMoveTo(90);
        } else if(millis() - this->_lastMessageTrackingTimestamp > Vulcanuino::TIME_TO_IDLE) {
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
    if(this->_state & Vulcanuino::STATE_READY) {
        unsigned int steps = this->xAxisDegrees2steps(degrees);
        if(this->_xAxisStepper.distanceToGo() == 0) {
            this->_xAxisStepper.moveTo(steps);
        } else if(digitalRead(D4) == LOW){
            this->_xAxisStepper.updateCurrentPosition(this->xAxisDegrees2steps(180));
        }
    }
}

bool Vulcanuino::yAxisMoveTo(unsigned int degrees){
    degrees = min(max(degrees % 180, 70), 120);
    this->_yAxisServoA.write(degrees);
    this->_yAxisServoB.write(degrees);
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
    this->axisCalibration();
    this->axisIdle();
    this->_xAxisStepper.run();
    if(!this->_xAxisStepper.isRunning())this->draw();
}
