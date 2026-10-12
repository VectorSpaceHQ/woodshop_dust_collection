#include "tool.h"

Tool::Tool(){
    // Constructor
}

void Tool::set_pin(int pin){
        PIN = pin;
        pinMode(pin, INPUT_PULLUP);
    }


void Tool::report_on(EspMQTTClient &clt){
    tool_on = true;
        String tool_state_str = "ON";

        message = name + ", " + tool_state_str;
        if (tool_on != old_tool_state){
            Serial.println("MQTT tool state has changed to ON");
            clt.publish("tools/dust_collection", message);
            old_tool_state = tool_on;
        }
    }
    
    void Tool::report_off(EspMQTTClient &clt){
        tool_on = false;
        String tool_state_str = "OFF";

        message = name + ", " + tool_state_str;
        if (tool_on != old_tool_state){
            Serial.println("MQTT tool state has changed to OFF");
            clt.publish("tools/dust_collection", message);
            old_tool_state = tool_on;
        }
    }

    bool Tool::report_state(){
        Serial.println(tool_on);
        return tool_on;
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
            Serial.println("Another tool (" + declaringTool + ") has declared itself as the last gate open. This tool (" + name + ") is no longer the last gate open.");
        }
    }

    bool Tool::is_last_gate_open(){
        return _last_gate_open;
        //return false; // testing
    }