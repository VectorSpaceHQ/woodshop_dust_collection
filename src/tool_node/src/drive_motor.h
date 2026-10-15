#ifndef DRIVE_MOTOR_H
#define DRIVE_MOTOR_H

#include "Arduino.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <stdint.h>


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
    void enforceTravelLimits();
        void stop();


    
  private:
    bool _isSetup;
    ledc_channel_t _pwmChannel;
    ledc_timer_t _timer;
    int _maxCommand = (1<<DEFAULT_DRIVE_PWM_RESOLUTION) -1;
    // Start harder than we finish: full duty for BREAKAWAY_MS to get the gate
    // moving, then RUN_DUTY_PERCENT of that for the rest of the travel, so the
    // torque when hitting an end stop is ~1/1.2 (about 20% less) of the start torque.
    static const unsigned long BREAKAWAY_MS = 1000;
    static const int RUN_DUTY_PERCENT = 83; // 100 / 1.2
    int _currentDuty = 0;
    void setDuty(int duty);
    int _lastSpeed = 0;
    int _sel0 = D10;
    int _pwmPin;
    int _sensePin;
    int _pinA;
    int _pinB;
    int _lastcommand = 4; //0 = close, 1 = open, 2 = brake, 3 = coast
    bool _invert;
    int _time_since_called = 0;
    int _start_time = 0;

    // Stall window: STALL_WINDOW_SIZE samples taken every STALL_SAMPLE_INTERVAL_MS
    // = 1 s of history (2 bytes/sample -> 1 KB). Stall is only evaluated once the
    // window has filled, so start-up inrush can't trip it.
    static const int STALL_WINDOW_SIZE = 100;
    static const unsigned long STALL_SAMPLE_INTERVAL_MS = 2;
    const int _stallADC = 4000;
    // Overcurrent detection is count-based, not average-based: the driver's own
    // overcurrent protection makes the current-sense pin burst to full scale and
    // then collapse to ~0 while it retries, so an average never gets high.
    // Stall = at least STALL_HIGH_COUNT of the last STALL_WINDOW_SIZE samples
    // were >= _stallADC, judged only after the start-up blanking time.
    static const int STALL_HIGH_COUNT = 10;
    static const unsigned long STALL_BLANKING_MS = 100;
    int _stallHighCount = 0;
    static const unsigned long MAX_CONTINUOUS_OPEN_MS = 45UL * 1000UL;
    static const unsigned long MAX_CONTINUOUS_CLOSE_MS = 45UL * 1000UL;
    static const unsigned long CLOSE_LOCKOUT_MS = 5UL * 60UL * 1000UL;
    static const unsigned long CLOSE_RETRY_MS = 10UL * 60UL * 1000UL;
    uint16_t _stallSamples[STALL_WINDOW_SIZE];
    int _stallSampleIndex = 0;
    int _stallSampleCount = 0;
    uint32_t _stallSampleSum = 0;
    unsigned long _lastStallSampleMs = 0;
    bool _isMoving = false;
    bool _fullyOpen = false;
    bool _fullyClosed = false;
    unsigned long _openStartedAtMs = 0;
    unsigned long _closeStartedAtMs = 0;
    unsigned long _closeAllowedAfterMs = 0;

    void open();
    void close();
    void setState(int state);
    void resetStallSamples();
};

#endif
