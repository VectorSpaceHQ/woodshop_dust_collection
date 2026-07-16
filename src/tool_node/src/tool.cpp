#include "tool.h"

Tool::Tool(){
    // Constructor
}

void Tool::set_pin(int pin){
        PIN = pin;
        pinMode(pin, INPUT_PULLUP);
    }


void Tool::report_on(EspMQTTClient &clt){
        tool_state = true;
        String tool_state_str = "ON";

        message = name + ", " + tool_state_str;
        if (tool_state != old_tool_state){
            Serial.println("tool state has changed");
            clt.publish("tools/dust_collection", message);
            old_tool_state = tool_state;
        }
    }
    
    void Tool::report_off(EspMQTTClient &clt){
        tool_state = false;
        String tool_state_str = "OFF";

        message = name + ", " + tool_state_str;
        if (tool_state != old_tool_state){
            Serial.println("tool state has changed");
            clt.publish("tools/dust_collection", message);
            old_tool_state = tool_state;
        }
    }

    bool Tool::report_state(){
        Serial.println(tool_state);
        return tool_state;
    }