#ifndef DRIVE_MOTOR_H
#define DRIVE_MOTOR_H

#include "Arduino.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define DEFAULT_DRIVE_PWM_FREQUENCY 100 //very high frequencies might blow up the RZ7886 driver chip. 5k is def bad, 1k is kinda bad.
#define DEFAULT_DRIVE_PWM_RESOLUTION LEDC_TIMER_8_BIT

class DriveMotor{
  public:
    DriveMotor();
    // bool setup(int forwardPin, int backwardPin, int forwardChannel, int backwardChannel,
    //            int pwmFrequency = DEFAULT_DRIVE_PWM_FREQUENCY,
    //            int pwmResolution = DEFAULT_DRIVE_PWM_RESOLUTION);
    bool init(int PinA, int PinB, 
              ledc_channel_t channelA,
              int pwm, int sense, bool invert = false);
    void loop(int speed, bool enable = true);
    void wake();
    void consider_close(int duration, int initialDelay = 0);
    void consider_open(int duration, int initialDelay = 0);
    void reportCurrent();
    bool isMoving();
    bool isOpening();
    int getState();
    void stallProtect();
        void stop();


    
  private:
    bool _isSetup;
    ledc_channel_t _pwmChannel;
    ledc_timer_t _timer;
    int _maxCommand = (1<<DEFAULT_DRIVE_PWM_RESOLUTION) -1;
    int _lastSpeed = 0;
    int _sel0 = D10;
    int _pwmPin;
    int _sensePin;
    int _pinA;
    int _pinB;
    int _lastcommand = 2; //0 = close, 1 = open, 2 = brake, 3 = coast
    bool _invert;
    int _time_since_called = 0;
    int _start_time = 0;
    const int _stallADC = 4000;
    unsigned long _stallStartMs = 0;
    const unsigned long _stallTimeoutMs = 500;
    bool _isMoving = false;

    void open();
    void close();
    void setState(int state);
};

#endif
