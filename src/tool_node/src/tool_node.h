#ifndef TOOL_NODE_H
#define TOOL_NODE_H

#include <Arduino.h>
#include "EspMQTTClient.h"
#include "drive_motor.h"
#include "leds.h"
#include "tool.h"

class ToolNode {
  public:
    ToolNode(EspMQTTClient& mqttClient, const char* toolName);
    void setup();
    void loop();
    void onStatusMessageReceived(const String& message);
    void onDustCollectionMessageReceived(const String& message);

  private:
    static const int TOOL_CURR_SENSE_PIN = D3;
    static const int VAC_CNTRL_PIN = D4;
    static const int LED1_PIN = D6;
    static const int LED2_PIN = D8;
    static const int GATE_MOTOR_CURR_SENSE_PIN = D2;
    static const int OPEN_TIME = 40;
    static const int CLOSE_TIME = 40;
    static const int GATE_DELAY = 120;

    EspMQTTClient& _mqttClient;
    const char* _toolName;
    Tool _tool;
    DriveMotor _gateMotor;
    LED _led2; // heartbeat
    LED _led1; // gate open indicator
};

#endif
