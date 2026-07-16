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
    _blinkCounter += 1;
    if (_blinkCounter >= 5000){
        this->toggle();
        _blinkCounter = 0;
    }
}