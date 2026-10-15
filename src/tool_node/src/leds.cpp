#include "leds.h"

LED::LED(){
}

bool LED::init(int ledPin){
    _ledPin = ledPin;
    pinMode(_ledPin, OUTPUT);
    return true;
}

void LED::on(){
    _ledState = HIGH;
    digitalWrite(_ledPin, _ledState);
}

void LED::off(){
    _ledState = LOW;
    digitalWrite(_ledPin, _ledState);
}

void LED::toggle(){
    if(_ledState == LOW){
        _ledState = HIGH;
    }
    else{
        _ledState = LOW;
    }
    digitalWrite(_ledPin, _ledState);
}

void LED::heartbeat(){
    // Time-based so the blink rate doesn't depend on how fast loop() runs.
    unsigned long now = millis();
    if ((now - _lastBeatMs) >= HEARTBEAT_TOGGLE_MS){
        this->toggle();
        _lastBeatMs = now;
    }
}
