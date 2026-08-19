/********************************************************************
 * Current Sense Assembly - Vector Space Wood Shop
 *
 * Built by Chaz Fisher and Adam Spontarelli for the Vector Space Makerspace
 * 2004 Memorial Ave, Lynchburg, VA
 *
 * This code is part of an automated dust collection system, intended to run on an ESP32-C3. A current sensor measures
 * current being drawn by the stationary tool (table saw, etc) and provides a digital signal to the ESP when the tool
 * is turned on. The ESP provides an output to an RC servo to open the dust collector gate, and a signal via an optocoupler
 * to turn on the central vacuum. When the current falls to zero, dealys are counted down before closing the gate and
 * turning off vacuum.
 *
 ***********************************************************************/



#include <Arduino.h>
#include "EspMQTTClient.h"
#include "tool_node.h"

#define TOOL_NAME "Router Table"

EspMQTTClient espclient(
    "VS-2",
    "fourhundredtwo",
    "10.0.0.218",  // MQTT Broker server ip
    TOOL_NAME,     // Client name that uniquely identify your device
    1883              // The MQTT port, default to 1883. this line can be omitted
    );

#define TIMER_INTERRUPT_DEBUG       1

ToolNode toolNode(espclient, TOOL_NAME);

void onStatusMessageReceived(const String& message) {
  toolNode.onStatusMessageReceived(message);
}

void onDustCollectionMessageReceived(const String& message) {
  toolNode.onDustCollectionMessageReceived(message);
}
    
    
void onConnectionEstablished()
{
    String msg = String(TOOL_NAME) + " Connected";
    espclient.publish("tools/dust_collection", msg);
    espclient.subscribe("tools/dust_collection/status", onStatusMessageReceived);
  espclient.subscribe("tools/dust_collection", onDustCollectionMessageReceived);
}


void setup()
{
  toolNode.setup();
}

void loop(){
  espclient.loop();
  toolNode.loop();
}


void ask_anyone_open(){
  espclient.publish("tools/dust_collection", "anyone open?");
}

void answer_anyone_open(){
  String msg = String(TOOL_NAME) + " is open";
  espclient.publish("tools/dust_collection", msg);
}
