#ifndef LEDS_H
#define LEDS_H
#include "Arduino.h"

class LED{
    public:
        LED();
        bool init(int ledPin);
        void loop();
        void blink();
        void toggle();
        void heartbeat();
        void on();
        void off();
    private:
        int _ledPin;
        int _ledState = LOW;
        int _blinkCounter = 0;
        int _heartbeatCounter = 0;
    };

#endif