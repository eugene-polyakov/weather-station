#include "EspMQTTClient.h"
#include <ArduinoJson.h>
#include "secrets.h"

#define tank_temp_current_topic "tank/temp/current"
#define tank_temp_state_topic "tank/temp/state"
#define tank_temp_command_topic "tank/temp/command"
#define tank_mode_command_topic "tank/mode/set"
#define tank_target_temp_topic "tank/target/temp"
#define tank_mode_state_topic "tank/mode"
#define tank_relay_command_topic "tank/relay"

char* tank_mode = "";

#define RELAY_PIN 15

EspMQTTClient client(
  SECRET_SSID,
  SECRET_WIFI_PASSWORD,
  SECRET_MQTT_BROKER,  // MQTT Broker server ip
  SECRET_MQTT_USER,   // Can be omitted if not needed
  SECRET_MQTT_PWD,   // Can be omitted if not needed
  "roof_device"      // Client name that uniquely identify your device
);

void setup() {
  Serial.begin(9600);
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.setMaxPacketSize(4096);  Serial.begin(9600);
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.setMaxPacketSize(4096);
}

void loop() {
  client.loop();
  
}

float tank_target_temp = 0;
bool tank_power_command_on = false;
bool relay_on = false;

void onConnectionEstablished()
{
  sendSensorDefs();
  Serial.print("connection established");
  client.subscribe(tank_mode_command_topic, [](const String & payload) {
    tank_power_command_on = payload.equals("electric");
    client.publish(tank_mode_state_topic, payload);
  });
    client.subscribe(tank_temp_command_topic, [](const String & payload) {
    tank_target_temp = payload.toFloat();
    client.publish(tank_temp_state_topic, payload);
  });
  client.subscribe(tank_relay_command_topic, [](const String & payload) {
    relay_on = payload.equals("on");
    digitalWrite(RELAY_PIN, relay_on ? HIGH : LOW);
  });

}


void sendSensorDefs() {
  JsonDocument doc;
  char buf[4096];
  

  JsonArray arr = doc.createNestedArray("modes");
  arr.add("off");
  arr.add("electric");

  doc["name"] = "Heating";
  doc["mode_state_topic"] = tank_mode_state_topic;
  doc["icon"] = "mdi:storage-tank";
  doc["mode_command_topic"] = tank_mode_command_topic;
  doc["unique_id"] = "tank01ae";
  doc["temperature_unit"] = "C";
  doc["temperature_state_topic"] = tank_temp_state_topic;
  doc["current_temperature_topic"] = tank_temp_current_topic;
  doc["temperature_command_topic"] = tank_temp_command_topic;
  doc["optimistic"] = false;
  doc["device"]["identifiers"] = "tank01";
  doc["device"]["name"] = "Water Tank";  

  serializeJson(doc, buf);
  client.publish("homeassistant/water_heater/tank/config", buf);
}
