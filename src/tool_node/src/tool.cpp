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

    void Tool::declare_last_gate_open(EspMQTTClient &clt){
        _last_gate_open = true;
        String gate_message = name + " is the last gate open";
        clt.publish("tools/dust_collection", gate_message);
    }

    void Tool::handle_dust_collection_message(const String &incomingMessage){
        const String marker = " is the last gate open";
        int markerIndex = incomingMessage.indexOf(marker);
        if (markerIndex == -1) {
            return;
        }

        String declaringTool = incomingMessage.substring(0, markerIndex);
        if (declaringTool != name) {
            _last_gate_open = false;
        }
    }

    bool Tool::is_last_gate_open(){
        //return _last_gate_open;
        return false; // testing
    }