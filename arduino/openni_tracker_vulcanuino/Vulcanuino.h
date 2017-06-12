#ifndef Vulcanuino_h
#define Vulcanuino_h

#include <ESP8266WiFi.h>
#include <Servo.h>
#include <AccelStepper.h>
#include <U8g2lib.h>
#include <Wire.h>

#undef round

class Vulcanuino
{
    static const unsigned int STEPS_TO_360         PROGMEM = 3454;
    static const unsigned int TIME_TO_IDLE         PROGMEM = 3000;
    // PHASE CONSTANTS
    static const unsigned int PHASE_INITIALIZING   PROGMEM = 0x1;
    static const unsigned int PHASE_CALIBRATING    PROGMEM = 0x2;
    static const unsigned int PHASE_IDLING         PROGMEM = 0x3;
    static const unsigned int PHASE_SETTING        PROGMEM = 0x4;
    static const unsigned int PHASE_TRACKING       PROGMEM = 0x5;
    static const unsigned int PHASE_MANUAL         PROGMEM = 0x6;

    // STATE CONSTANTS
    static const unsigned int STATE_INITIALIZED    PROGMEM = 0x1;
    static const unsigned int STATE_CALIBRATED     PROGMEM = 0x2;
    static const unsigned int STATE_READY          PROGMEM = 0x3;
public:
    Vulcanuino(unsigned int xAxisStepperAPin = 4, unsigned int xAxisStepperBPin = 5, unsigned int xAxisRefPin = 2, unsigned int yAxisServoPin = 10);
    ~Vulcanuino();
    void run();
    void begin();
    unsigned int xAxisCurrentDegrees();
    unsigned int xAxisTargetDegrees();

    void axisCalibration();
    void axisInitialization();
    void axisIdle();
    void axisManualControl();

    void draw();
    void menuSettings();
    void menuSettingsTrim();

    void trackUser(unsigned int degreesX = 0, unsigned int degreesY = 0);
    bool axisMoveTo(unsigned int degreesX = 0, unsigned int degreesY = 0);
    bool xAxisMoveTo(unsigned int degrees);
    bool yAxisMoveTo(unsigned int degrees);
protected:
    unsigned int xAxisDegrees2steps(unsigned int degrees);
    unsigned int xAxisSteps2Degrees(unsigned int steps);
private:
    unsigned int _xAxisRefPin;
    unsigned int _state = 0, _phase = 0;
    AccelStepper _xAxisStepper;
    Servo _yAxisServoA, _yAxisServoB;
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C _oledDisplay = U8G2_SSD1306_128X64_NONAME_F_HW_I2C(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
    unsigned long _lastMessageTrackingTimestamp;
};

#endif
