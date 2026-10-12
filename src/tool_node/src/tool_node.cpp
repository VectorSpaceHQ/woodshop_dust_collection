#include "tool_node.h"

ToolNode::ToolNode(EspMQTTClient& mqttClient, const char* toolName)
    : _mqttClient(mqttClient), _toolName(toolName) {
}

void ToolNode::setup() {
  _tool.name = _toolName;
  bool startupOK = true;

  Serial.begin(115200);
  while (!Serial);

  delay(200);

  Serial.print("\nStarting Current Sensor Assembly");
  delay(200);

  pinMode(TOOL_CURR_SENSE_PIN, INPUT_PULLUP);
  pinMode(VAC_CNTRL_PIN, OUTPUT);
  pinMode(GATE_MOTOR_CURR_SENSE_PIN, INPUT_PULLUP);

  _toolSenseLastReading = digitalRead(TOOL_CURR_SENSE_PIN);
  _toolSenseStableState = _toolSenseLastReading;
  _toolSenseLastChangeMs = millis();

  startupOK &= _gateMotor.init(D0, D1, LEDC_CHANNEL_2, D7, GATE_MOTOR_CURR_SENSE_PIN, false);

  _mqttClient.enableOTA();

  _led2.init(LED2_PIN);
  _led1.init(LED1_PIN);
  _led1.off();
  delay(2000); // time for wifi

  (void)startupOK;

}

void ToolNode::loop() {
  _led2.heartbeat();
  _gateMotor.stallProtect();

  int currSense = digitalRead(TOOL_CURR_SENSE_PIN);
  if (currSense != _toolSenseLastReading) {
    _toolSenseLastReading = currSense;
    _toolSenseLastChangeMs = millis();
  }

  if ((millis() - _toolSenseLastChangeMs) >= TOOL_STATE_DEBOUNCE_MS) {
    _toolSenseStableState = _toolSenseLastReading;
  }

  currSense = _toolSenseStableState;
  bool wasOpening = _gateMotor.isOpening();

  if (currSense == HIGH)
  {
    if (!_tool.tool_on) {
      _tool.report_on(_mqttClient);
    }
    _gateMotor.consider_open(OPEN_TIME);
    
    
  }
  else if (!_tool.is_last_gate_open()) // no current and not last gate open, so close
  {
    if (_tool.tool_on) {
      Serial.println("Tool is off and I'm not last, so closing gate");
      _tool.report_off(_mqttClient);
    }
    _gateMotor.consider_close(CLOSE_TIME, 0);
  }
  else
  {
    if (_tool.tool_on) {
      Serial.println("Tool is off, but I'm last, so not closing gate");
      _tool.report_off(_mqttClient);
    }
    //_gateMotor.stop();
  }

  bool isOpening = _gateMotor.isOpening();
  if (!wasOpening && isOpening) {
    // when current is sensed, declare this tool as the last gate open
    _tool.declare_last_gate_open(_mqttClient);
  }

  if (_gateMotor.isMoving()) {
    _led1.on();
  } else {
    _led1.off();
  }
}

void ToolNode::onStatusMessageReceived(const String& message) {
  if (message.indexOf("REPORT STATUS") == -1){
    return;
  }

  String msg;
  if (_tool.tool_on == true){
    msg = _tool.name + ", Current state: ON";
  }
  else{
    msg = _tool.name + ", Current state: OFF";
  }
  _mqttClient.publish("tools/dust_collection/status", msg);
}

void ToolNode::onDustCollectionMessageReceived(const String& message) {
  _tool.handle_dust_collection_message(message);
}
