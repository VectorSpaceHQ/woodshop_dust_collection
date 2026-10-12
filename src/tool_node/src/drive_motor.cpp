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

    return _isSetup;
}

void DriveMotor::close(int duration, int initialDelay){
    if (_state == CLOSED) {
        return;
    }
    if (_state != CLOSE_PENDING && _state != CLOSING) {
        _start_time = millis() / 1000;
        _state = CLOSE_PENDING;
    }
    int timeSinceCalled = (millis() / 1000) - _start_time;

    if (timeSinceCalled < initialDelay) {
        // not yet ready to close
        return;
    }

    _state = CLOSING;
    int activeMotionTime = timeSinceCalled - initialDelay;
    Serial.println("Closing gate, active motion time: " + String(activeMotionTime) + " seconds");

    if (activeMotionTime < duration) {
        // Apply closing signal for duration seconds.
        Serial.println("Closing gate");
        digitalWrite(_pinA, 0);
        digitalWrite(_pinB, 1);
        digitalWrite(_sel0, 0);

        ESP_ERROR_CHECK( ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, 32767) );
        ESP_ERROR_CHECK( ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel) );
        
        // protect against stall condition
        int current = analogRead(_sensePin);
        if (current >= _stallADC) {
            if (_stallStartMs == 0) {
                _stallStartMs = millis();
            } else if ((millis() - _stallStartMs) >= _stallTimeoutMs) {
                Serial.println("Stall detected during close, stopping motor");
                this->open(0.5);
                this->finish(CLOSED);
                return;
            }
        } else {
            _stallStartMs = 0;
        }
    }
    else {
        // duration has elapsed, stop the motor
        this->finish(CLOSED);
    }
}

void DriveMotor::open(int duration){
    // duration in seconds

    if (_state == OPEN) {
        return;
    }
    if (_state != OPENING) {
        Serial.println("Opening gate");
        _start_time = millis() / 1000;
        _state = OPENING;
    }
    int timeSinceCalled = (millis() / 1000) - _start_time;

    if (timeSinceCalled < duration) {
        digitalWrite(_pinA, 1);
        digitalWrite(_pinB, 0);
        digitalWrite(_sel0, 1);

        ESP_ERROR_CHECK( ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, -32767) );
        ESP_ERROR_CHECK( ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel) );

        int current = analogRead(_sensePin);
        if (current >= _stallADC) {
            if (_stallStartMs == 0) {
                //Serial.println("possible stall detected during open, starting stall timer");
                _stallStartMs = millis();
            } else if ((millis() - _stallStartMs) >= _stallTimeoutMs) {
                // Hitting the end stop is the normal end of travel.
                Serial.println("Stall detected during open, stopping motor");
                this->finish(OPEN);
                return;
            }
        } else {
            _stallStartMs = 0;
        }
    }
    else {
        this->finish(OPEN);
    }

}

void DriveMotor::stop(){
    this->coast();
    _state = STOPPED;
}

void DriveMotor::finish(GateState endState){
    this->coast();
    _state = endState;
}

void DriveMotor::coast(){
    _stallStartMs = 0;

    digitalWrite(_pinA, 0);
    digitalWrite(_pinB, 0);
    ESP_ERROR_CHECK( ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, 0) );
    ESP_ERROR_CHECK( ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel) );
}

void DriveMotor::reportCurrent(){
    int current = analogRead(_sensePin);
    Serial.print("Current: ");
    Serial.println(current);
}

bool DriveMotor::isMoving(){
    return _state == OPENING || _state == CLOSING;
}

bool DriveMotor::isOpening(){
    return _state == OPENING;
}


void DriveMotor::wake(){
    // The VNH7100 must be woken out of standby.
    // Toggle INA from 0 to 1
    // Toggle PWM from 0 to 1 with a 20us delay.
    pinMode(_sel0, OUTPUT);

    ESP_ERROR_CHECK( ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, 0) );
    ESP_ERROR_CHECK( ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, _maxCommand) );

    digitalWrite(_sel0, 0);
    delayMicroseconds(20);
    digitalWrite(_sel0, 1);

    Serial.println("Drive motor enabled");
}