#include "Arduino.h"
#include "drive_motor.h"
#include "limits.h"

DriveMotor::DriveMotor()
{
    _isSetup = false;
    _stallSamples.resize(STALL_WINDOW_SIZE, 0);
}

// void DriveMotor::loop(int speed, bool enable) {
//     if (!enable) {
//         this->stop();
//         return;
//     }

//     if (speed > 0) {
//         this->consider_open(abs(speed));
//     } else if (speed < 0) {
//         this->consider_close(abs(speed));
//     } else {
//         this->stop();
//     }
// }

int DriveMotor::getState()
{
    return _lastcommand;
}

void DriveMotor::setState(int state)
{
    if (state < 0)
    {
        state = 0;
    }
    if (state > 3)
    {
        state = 3;
    }
    _lastcommand = state;
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
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = DEFAULT_DRIVE_PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_2,
        .freq_hz = DEFAULT_DRIVE_PWM_FREQUENCY, // Set carrier frequency of PWM
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel1 = {
        .gpio_num = _pwmPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = _pwmChannel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_2,
        .duty = 0, // Set duty to 0%
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    this->wake(); // wake up motor driver
    this->stop(); // ensure no stale motion flag or output state

    return _isSetup;
}

void DriveMotor::consider_close(int duration, int initialDelay)
{
    if ((long)(millis() - _closeAllowedAfterMs) < 0)
    {
        return;
    }

    if (_fullyClosed)
    {
        return;
    }
    if (this->getState() != 0)
    {
        _start_time = millis() / 1000;
        _time_since_called = 0;
        this->setState(0);
    }
    else
    {
        _time_since_called = (millis() / 1000) - _start_time;
    }

    if (_time_since_called < initialDelay)
    {
        // not yet ready to close

        return;
    }
    

    int activeMotionTime = _time_since_called - initialDelay;

    if (activeMotionTime < duration)
    {
        this->close();
        // this->stallProtect();
    }
    else
    {
        // duration has elapsed: travel is complete, so don't restart the close
        _fullyClosed = true;
        this->stop();
    }
}

void DriveMotor::close()
{
    if ((this->getState() == 0 && _isMoving) || _fullyClosed)
    {
        // Already closing; do not restart the close cycle on every loop.
        return;
    }
    _closeStartedAtMs = millis();
    _isMoving = true;
    _fullyOpen = false;
    this->resetStallSamples();
    Serial.println("Closing gate");
    this->setState(0);
    digitalWrite(_pinA, 0);
    digitalWrite(_pinB, 1);
    digitalWrite(_sel0, 0);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, _maxCommand));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel));
}

void DriveMotor::consider_open(int duration, int initialDelay)
{
    if (_fullyOpen)
    {
        Serial.println("Gate is fully open, not opening further");
        delayMicroseconds(100000);
        return;
    }

    // duration in seconds
    //
    if (this->getState() != 1)
    {
        // motor is not already opening, so reset the timer
        Serial.println("Open requested, starting timer");
        _start_time = millis() / 1000;
        _time_since_called = 0;
        this->setState(1);
    }
    else
    {
        _time_since_called = (millis() / 1000) - _start_time;
    }
    

    if (_time_since_called < duration)
    {
        this->open();
        // this->stallProtect();
    }
    else
    {
        // duration has elapsed: travel is complete, so don't restart the open
        _fullyOpen = true;
        this->stop();
    }
}

void DriveMotor::open()
{
    if ((this->getState() == 1 && _isMoving) || _fullyOpen)
    {
        // Already opening; do not restart the open cycle on every loop.
        return;
    }

    Serial.println("Opening gate at millis=" + String(millis()));
    _openStartedAtMs = millis();
    _closeAllowedAfterMs = millis() + CLOSE_LOCKOUT_MS;
    _isMoving = true;
    _fullyClosed = false;
    this->resetStallSamples();
    this->setState(1);
    digitalWrite(_pinA, 1);
    digitalWrite(_pinB, 0);
    digitalWrite(_sel0, 1);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, _maxCommand));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel));
}

void DriveMotor::stop()
{
    if (this->getState() == 2)
    {
        // Already stopped; do not restart the stop cycle on every loop.
        return;
    }

    Serial.println("Stopping gate");
    this->setState(2);
    _isMoving = false;

    // coast
    digitalWrite(_pinA, 0);
    digitalWrite(_pinB, 0);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel));
}

void DriveMotor::enforceTravelLimits()
{
    // Runs every loop, so a move is stopped even when nothing is calling
    // consider_open()/consider_close() (close lockout, last gate open).
    if ((this->getState() == 1 && _isMoving) && ((long)(millis() - _openStartedAtMs) >= (long)MAX_CONTINUOUS_OPEN_MS))
    {
        Serial.println("Open command hit 45 second limit, stopping gate");
        _fullyOpen = true;
        this->stop();
    }
    else if ((this->getState() == 0 && _isMoving) && ((long)(millis() - _closeStartedAtMs) >= (long)MAX_CONTINUOUS_CLOSE_MS))
    {
        Serial.println("Close command hit 45 second limit, stopping gate");
        _fullyClosed = true;
        this->stop();
    }
}

void DriveMotor::stallProtect()
{
    int current = analogRead(_sensePin);

    _stallSamples.push_back(current);
    if (_stallSamples.size() > STALL_WINDOW_SIZE) {
        _stallSamples.erase(_stallSamples.begin());
    }
    _stallSampleSum = std::accumulate(_stallSamples.begin(), _stallSamples.end(), 0);
     
    if (!_isMoving || _stallSamples.size() < STALL_WINDOW_SIZE)
    {
        return;
    }

    int avgCurrent = _stallSampleSum / _stallSamples.size();

    if (avgCurrent > _stallADC)
    {
        Serial.println("Stall detected by running average: " + String(avgCurrent));

        if (_lastcommand == 0)
        {
            _fullyClosed = true;
        }
        else if (_lastcommand == 1)
        {
            _fullyOpen = true;
        }

        this->stop();
    }
}

void DriveMotor::resetStallSamples()
{
    _stallSampleIndex = 0;
    _stallSampleCount = 0;
    _stallSampleSum = 0;

    for (int i = 0; i < STALL_WINDOW_SIZE; i++)
    {
        _stallSamples[i] = 0;
    }
}

void DriveMotor::reportCurrent()
{
    int current = analogRead(_sensePin);
    Serial.print("Current: ");
    Serial.println(current);
}

bool DriveMotor::isMoving()
{
    return _isMoving;
}

bool DriveMotor::isOpening()
{
    return _isMoving && (this->getState() == 1);
}

void DriveMotor::wake()
{
    // The VNH7100 must be woken out of standby.
    // Toggle INA from 0 to 1
    // Toggle PWM from 0 to 1 with a 20us delay.
    pinMode(_sel0, OUTPUT);
    _isMoving = false;
    this->setState(2);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _pwmChannel, _maxCommand));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, _pwmChannel));

    digitalWrite(_sel0, 0);
    delayMicroseconds(20);
    digitalWrite(_sel0, 1);

    Serial.println("Drive motor enabled");
}