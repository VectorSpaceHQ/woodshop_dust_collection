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

  startupOK &= _gateMotor.init(D0, D1, LEDC_CHANNEL_2, D7, GATE_MOTOR_CURR_SENSE_PIN, false);

  _mqttClient.enableOTA();

  _led2.init(LED2_PIN);
  _led1.init(LED1_PIN);
  _led1.off();
  delay(2000); // time for wifi

  (void)startupOK;

  _gateMotor.consider_close(20,0);
}

void ToolNode::loop() {
  _led2.heartbeat();
  _gateMotor.stallProtect();


  int currSense = digitalRead(TOOL_CURR_SENSE_PIN);
  bool wasOpening = _gateMotor.isOpening();

  if (currSense == HIGH)
  {
    if (!_tool.tool_state) {
      _tool.report_on(_mqttClient);
    }
    _gateMotor.consider_open(OPEN_TIME);
  }
  else if (!_tool.is_last_gate_open())
  {
    if (_tool.tool_state) {
      _tool.report_off(_mqttClient);
    }

    if (_gateMotor.isOpening() || _gateMotor.isMoving()) {
      _gateMotor.consider_open(OPEN_TIME);
    } else {
      _gateMotor.consider_close(CLOSE_TIME, GATE_DELAY);
    }
  }
  else
  {
    Serial.println("Tool is off, but I'm last, so not closing gate");
    _gateMotor.stop();
  }

  bool isOpening = _gateMotor.isOpening();
  if (!wasOpening && isOpening) {
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
  if (_tool.tool_state == true){
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
