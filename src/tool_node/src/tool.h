#include "EspMQTTClient.h"

class Tool{
    public:
        Tool();
        int PIN;
        String name;
        bool tool_state = false;
        bool old_tool_state = false;
        String message;

        unsigned long currentTime;
        unsigned long cloopTime = millis();

        void set_pin(int pin);
        void report_on(EspMQTTClient &clt);
        void report_off(EspMQTTClient &clt);
        void set_on();
        void set_off();
        void measure_state();
        bool report_state();
};
