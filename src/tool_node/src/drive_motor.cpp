#include "Arduino.h"
#include "drive_motor.h"
#include "limits.h"


DriveMotor::DriveMotor(){
  _isSetup = false;
}


bool DriveMotor::init(int PinA, int PinB, 
                ledc_channel_t channelA,
                int pwm, int sense, bool invert)
{
    Serial.println("Initializing drive motor");
    _isSetup = true;
    _pwmChannel = channelA;
    _pwmPin = pwm;
    _pinA = PinA;
    _pinB = PinB;
    _invert = invert;
    _sensePin = sense;

    pinMode(PinA, OUTPUT);
    pinMode(PinB, OUTPUT);
    pinMode(_sensePin, INPUT);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = DEFAULT_DRIVE_PWM_RESOLUTION,
        .timer_num        = LEDC_TIMER_2,
        .freq_hz          = DEFAULT_DRIVE_PWM_FREQUENCY,  // Set carrier frequency of PWM
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel1 = {
        .gpio_num       = _pwmPin,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = _pwmChannel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_2,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    this->wake(); // wake up motor driver
    this->stop(); // ensure no stale motion flag or output state

    return _isSetup;
}

void DriveMotor::consider_close(int duration, int initialDelay){
    if (this->getState() != 0) {
        _start_time = millis() / 1000;
        _time_since_called = 0;
        this->setState(0);
    }
    else {
        _time_since_called = (millis() / 1000) - _start_time;
    }

    if (_time_since_called < initialDelay) {
        // not yet ready to close
        _isMoving = false;
        this->setState(2);
        return;
    }

    int activeMotionTime = _time_since_called - initialDelay;

    if (activeMotionTime < duration) {
        this->close();
        this->stallProtect();
    }
    else {
        // duration has elapsed, stop the motor
        this->stop();
    }
}


void DriveMotor::close(){
        _isMoving = true;
        Serial.println("Closing gate");
        this->setState(0);
        digitalWrite(_pinA, 0);
        digitalWrite(_pinB, 1);
        digitalWrite(_sel0, 0);

        ESP_ERROR_CHECK( ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, _maxCommand) );
        ESP_ERROR_CHECK( ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel) );
}

void DriveMotor::consider_open(int duration, int initialDelay){
    // duration in seconds
    //
    if (this->getState() != 1) {
        // motor is not already opening, so reset the timer
        Serial.println("Open requested, starting timer");
        _start_time = millis() / 1000;
        _time_since_called = 0;
        this->setState(1);
    }
    else {
        _time_since_called = (millis() / 1000) - _start_time;
    }

    if (_time_since_called < duration) {
        this->open();
    }
    else {
        this->stop();
    }
    
}

void DriveMotor::open(){
    Serial.println("Opening gate");
    _isMoving = true;
    this->setState(1);
    digitalWrite(_pinA, 1);
    digitalWrite(_pinB, 0);
    digitalWrite(_sel0, 1);

    ESP_ERROR_CHECK( ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, _maxCommand) );
    ESP_ERROR_CHECK( ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel) );
}

void DriveMotor::stop(){
    Serial.println("Stopping gate");
    this->setState(2);
    _isMoving = false;
    
    _stallStartMs = 0;
    _lastcommand = 2;

    // coast
    digitalWrite(_pinA, 0);
    digitalWrite(_pinB, 0);
    ESP_ERROR_CHECK( ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, 0) );
    ESP_ERROR_CHECK( ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel) );
}

void DriveMotor::stallProtect(){
    int current = analogRead(_sensePin);
    if (current >= _stallADC) {
        if (_stallStartMs == 0) {
            _stallStartMs = millis();
        } else if ((millis() - _stallStartMs) >= _stallTimeoutMs) {
            Serial.println("Stall detected during close, stopping motor");
            this->open();
            this->stop();
            return;
        }
    } else {
        _stallStartMs = 0;
    }
}

void DriveMotor::reportCurrent(){
    int current = analogRead(_sensePin);
    Serial.print("Current: ");
    Serial.println(current);
}

bool DriveMotor::isMoving(){
    return _isMoving;
}

bool DriveMotor::isOpening(){
    return _isMoving && (this->getState() == 1);
}


void DriveMotor::wake(){
    // The VNH7100 must be woken out of standby.
    // Toggle INA from 0 to 1
    // Toggle PWM from 0 to 1 with a 20us delay.
    pinMode(_sel0, OUTPUT);
    _isMoving = false;
    _lastcommand = 2;

    ESP_ERROR_CHECK( ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, 0) );
    ESP_ERROR_CHECK( ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, _maxCommand) );

    digitalWrite(_sel0, 0);
    delayMicroseconds(20);
    digitalWrite(_sel0, 1);

    Serial.println("Drive motor enabled");
}