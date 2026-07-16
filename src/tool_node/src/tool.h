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
        void declare_last_gate_open(EspMQTTClient &clt);
        void handle_dust_collection_message(const String &incomingMessage);
        bool is_last_gate_open();
    private:
        bool _last_gate_open = false;
};
