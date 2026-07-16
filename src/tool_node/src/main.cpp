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
#include "tool.h"
#include "EspMQTTClient.h"
#include "drive_motor.h"
#include "leds.h"

#define TOOL_NAME "Router Table"

EspMQTTClient espclient(
    "VS-2",
    "fourhundredtwo",
    "10.0.0.218",  // MQTT Broker server ip
    TOOL_NAME,     // Client name that uniquely identify your device
    1883              // The MQTT port, default to 1883. this line can be omitted
    );

#define TIMER_INTERRUPT_DEBUG       1

// Initializations for this application

// Turn-off timer values, in 100 msec "ticks"
#define   GATE_DELAY    120 //  delay in seconds to close gate after tool turns off

bool toolOn = false;
bool gateOpen = false;
int gatePosition = 0;
int gateCounter = 0;
int vacCounter = 0;
int vacCntrl = 0;
int currSense = 0;
int openTime = 60; //seconds
int closeTime = 60; //seconds
const int toolCurrSensePin = D3;
const int vacCntrlPin = D4;
const int led1Pin = D6;
const int led2Pin = D8;
const int gateMotorCurrSensePin = D2;
DriveMotor gateMotor;
Tool tool;
LED led2;
LED led1;


void onStatusMessageReceived(const String& message) {
  if (message.indexOf("REPORT STATUS") != -1){
      String msg;
      if (tool.tool_state == true){
        msg = tool.name + ", Current state: ON"; 
        espclient.publish("tools/dust_collection/status", msg);
      }
      else{
        msg = tool.name + ", Current state: OFF"; 
        espclient.publish("tools/dust_collection/status", msg);
      }
      return;
    }
    delay(1000); // debounce
}

void onDustCollectionMessageReceived(const String& message) {
  tool.handle_dust_collection_message(message);
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
  tool.name = TOOL_NAME;
  bool startupOK = true;

  Serial.begin(115200);
  while (!Serial);

  delay(200);

  Serial.print("\nStarting Current Sensor Assembly");
  delay(200);

  // Setup Digital I/O pins
  pinMode(toolCurrSensePin, INPUT_PULLUP);  // Use the internal pull-up, so 3.3V doesn't have to be routed on the board.
  pinMode(vacCntrlPin, OUTPUT);
  pinMode(gateMotorCurrSensePin, INPUT_PULLUP);


  startupOK &= gateMotor.init(D0, D1, LEDC_CHANNEL_2, D7, gateMotorCurrSensePin, false);

  // Optional functionalities of EspMQTTClient
  //espclient.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  espclient.enableOTA(); // Enable OTA (Over The Air) updates. Password defaults to MQTTPassword. Port is the default OTA port. Can be overridden with enableOTA("password", port).
  
  led2.init(led2Pin);
  led1.init(led1Pin);
  led1.toggle();
  delay(2000); // time for wifi

} // End of setup

void loop(){
  espclient.loop();
  led2.heartbeat();
  currSense = digitalRead(toolCurrSensePin);
  bool wasOpening = gateMotor.isOpening();

  if (currSense == HIGH) // Current sense input is High, indicating current >= 1.5 A
  {
    tool.report_on(espclient);
    gateMotor.open(openTime);
  }
  else  // Current sensor input is Lo, indicating the tool has been turned off
  {
    
    if (!tool.is_last_gate_open()) {
      tool.report_off(espclient);
      gateMotor.close(closeTime, GATE_DELAY);
    }
  }

  bool isOpening = gateMotor.isOpening();
  if (!wasOpening && isOpening) {
    tool.declare_last_gate_open(espclient);
  }

  if (gateMotor.isMoving()) {
    led1.on();
  } else {
    led1.off();
  }
}


void ask_anyone_open(){
  espclient.publish("tools/dust_collection", "anyone open?");
}

void answer_anyone_open(){
  String msg = String(TOOL_NAME) + " is open";
  espclient.publish("tools/dust_collection", msg);
}