#include <ModbusRTUMaster.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include "EspMQTTClient.h"
#include <DallasTemperature.h>
#include <ArduinoJson.h>

// RS485 pins
#define RX        14   //Serial Receive pin
#define TX        16    //Serial Transmit pin
#define DE        17    //RS485 Direction control

#define RAIN_POWER_PIN  13
#define RAIN_DATA_PIN   34

#define RELAY_PIN       15
//#define TANK_CURRENT_PIN      33
#define ONE_WIRE_BUS    32


uint32_t lastSentTime = 0UL;

ModbusRTUMaster master(Serial1, DE);

Adafruit_BMP280 bmp;
Adafruit_AHTX0 aht;


EspMQTTClient client(
EspMQTTClient client(
  SECRET_SSID,
  SECRET_WIFI_PASSWORD,
  SECRET_MQTT_BROKER,  // MQTT Broker server ip
  SECRET_MQTT_USER,   // Can be omitted if not needed
  SECRET_MQTT_PWD,   // Can be omitted if not needed
  "roof_device"      // Client name that uniquely identify your device
);
   // Can be omitted if not needed
  "roof_device"      // Client name that uniquely identify your device
);


#define temp_topic "outside_temp"
#define pressure_topic "outside_pressure"
#define humidity_topic  "outside_humidity"
#define wind_speed_topic  "outside_wind_speed"
#define wind_direction_topic  "outside_wind_direction"
#define rssi_topic "outside_rssi"
#define rain_topic "outside_rain"

//#define tank_temp_command_topic "tank/temp/set"
//#define tank_temp_state_topic "tank/temp/get"
#define tank_temp_sensor_topic "tank/temp"
//#define tank_mode_command_topic "tank/mode/set"
//#define tank_mode_state_topic "tank/mode"
//
//#define tank_relay_command_topic "tank/relay/set"
//#define tank_current_topic "tank/current"


const char* wind_directions[] = {"S", "SE", "E", "NE", "N", "NW", "W", "SW"};
const long interval = 10000;

const int cycles_to_repost_topic_defs = 100;

bool needToSendSensorDefs = true;
int cycle = 0;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);

float tank_target_temp = 0;
bool tank_power_command_on = false;
bool relay_on = false;

void setup() {
  Serial.begin(9600);
  client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
  client.setMaxPacketSize(4096);

  pinMode(RAIN_POWER_PIN, OUTPUT);
//  pinMode(RELAY_PIN, OUTPUT);
  pinMode(RAIN_DATA_PIN, INPUT);
//  pinMode(TANK_CURRENT_PIN, INPUT);

  master.begin(4800, SERIAL_8N1, RX, TX, false);

  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(1000);
  }

  Serial.println("AHT10 or AHT20 found");


  Serial.println("Connecting to BP280...");
  setupBMP();
  Serial.println("BMP280 found");

  tempSensors.begin();

}

void loop() {
  client.loop();

  if ((millis() - lastSentTime > interval) && client.isMqttConnected()) {
    if (needToSendSensorDefs) {
      sendSensorDefs();
      needToSendSensorDefs = false;
    }
    uint16_t buf[8];
    ///// 0. Turn rain power on to warm up
    digitalWrite(RAIN_POWER_PIN, HIGH);


    ///// 1. Read wind data from rs 485 bus

    int16_t rs485_value = -1;

    // wind direction
    bool result = master.readHoldingRegisters(1, 0, buf, 7);
    if (!result) {
      Serial.println("RS485 Read NOT OK, device 1");
    } else {
      rs485_value = buf[0];
    }
    bool error = rs485_value < 0 || rs485_value > 7;
    if (error) {
      postMqtt(wind_direction_topic, rs485_value);
    } else {
      client.publish(wind_direction_topic, wind_directions[rs485_value]);
    }

    rs485_value = -1;
    // wind speed
    result = master.readHoldingRegisters(2, 0, buf, 7);
    if (!result) {
      Serial.println("RS485 Read NOT OK, device 2");
    } else {
      rs485_value = buf[0];
    }

    postMqtt(wind_speed_topic, (float)rs485_value / (float)10.0);

    //// 2. read pressure/humitidy/air temp from ATH/BMP board

    float pressure = bmp.readPressure();

    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    postMqtt(temp_topic, temp.temperature);
    postMqtt(humidity_topic, humidity.relative_humidity);
    postMqtt(pressure_topic, pressure);


    postMqtt(rssi_topic, WiFi.RSSI());



    /// 3. Hot water tank temperature

    tempSensors.requestTemperatures();
    float tempC = tempSensors.getTempCByIndex(0);
    postMqtt(tank_temp_sensor_topic, tempC);


    //// 4. Read rain level out of warmed up board

//    uint16_t rain_level = analogRead(RAIN_DATA_PIN);
    uint16_t rain_level = digitalRead(RAIN_DATA_PIN);
    postMqtt(rain_topic, rain_level);

    //// 5. Read current
    
//    uint16_t tank_current = 0;
//    for (int i = 0; i < 100; i++) {
//      uint16_t c = analogRead(TANK_CURRENT_PIN);
//      if (c > tank_current) { tank_current = c; }
//      delay(1);
//    }
//    postMqtt(tank_current_topic, tank_current);


//    /// 6. Debug  
//    char debug_buf[512];
//    snprintf(debug_buf, sizeof(debug_buf), "Relay %d, power command %d, target temp %.2f", relay_on, tank_power_command_on, tank_target_temp);
//    client.publish("tank/debug", debug_buf);

    /// 7. Housekeeping

    cycle++;
    if (cycle > cycles_to_repost_topic_defs) {
      cycle = 0;
      needToSendSensorDefs = true;
    }
    Serial.print("cycle ");
    Serial.println(cycle);
    digitalWrite(RAIN_POWER_PIN, LOW);
    lastSentTime = millis();
   }

}

void onConnectionEstablished()
{
//  Serial.print("connection established");
//  // Subscribe to "mytopic/test" and display received message to Serial
//  client.subscribe(tank_mode_command_topic, [](const String & payload) {
//    tank_power_command_on = payload.equals("electric");
//    client.publish(tank_mode_state_topic, payload);
//  });
//    client.subscribe(tank_temp_command_topic, [](const String & payload) {
//    tank_target_temp = payload.toFloat();
//    client.publish(tank_temp_state_topic, payload);
//  });
//  client.subscribe(tank_relay_command_topic, [](const String & payload) {
//    relay_on = payload.equals("on");
//    digitalWrite(RELAY_PIN, relay_on ? HIGH : LOW);
//  });
}




void setupBMP() {
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}


void sendSensorDefs() {

  char buf[4096];

  JsonDocument ws;
  ws["name"] = "Weather Station";  
  ws["identifiers"] = "weather_station";
  
  JsonDocument doc;

  doc["device_class"] = "humidity";
  doc["state_topic"] = humidity_topic;
  doc["unit_of_measurement"] = "%";
  doc["unique_id"] = "humidity01ae";
  doc["device"] = ws;

  serializeJson(doc, buf);
  client.publish("homeassistant/sensor/" humidity_topic "/config", buf);

  doc.clear();

  
  doc["device_class"] = "temperature";
  doc["state_topic"] = temp_topic;
  doc["unit_of_measurement"] = "°C";
  doc["unique_id"] = "temp01ae";
  doc["device"] = ws;
  

  serializeJson(doc, buf);
  client.publish("homeassistant/sensor/" temp_topic "/config", buf);

  doc.clear();


  doc["device_class"] = "pressure";
  doc["state_topic"] = pressure_topic;
  doc["unit_of_measurement"] = "Pa";
  doc["unique_id"] = "pressure01ae";
  doc["device"] = ws;

  serializeJson(doc, buf);
  client.publish("homeassistant/sensor/" pressure_topic "/config", buf);

  doc.clear();


  doc["name"] = "Wind direction";
  doc["state_topic"] = wind_direction_topic;
  doc["icon"] = "mdi:windsock";
  doc["unique_id"] = "wind_dir01ae";
  doc["device"] = ws;

  serializeJson(doc, buf);
  client.publish("homeassistant/sensor/" wind_direction_topic "/config", buf);

  doc.clear();


  doc["name"] = "Wind speed";
  doc["state_topic"] = wind_speed_topic;
  doc["unit_of_measurement"] = "m/s";
  doc["icon"] = "mdi:weather-windy";
  doc["unique_id"] = "wind_speed01ae";
  doc["device"] = ws;
  
  serializeJson(doc, buf);
  client.publish("homeassistant/sensor/" wind_speed_topic "/config", buf);


  doc.clear();


  doc["name"] = "Rain level";
  doc["state_topic"] = rain_topic;
  doc["icon"] = "mdi:weather-rainy";
  doc["unique_id"] = "rain01ae";
  doc["device"] = ws;

  serializeJson(doc, buf);
  client.publish("homeassistant/sensor/" rain_topic "/config", buf);


  doc.clear();

//  JsonArray arr = doc.createNestedArray("modes");
//  arr.add("off");
//  arr.add("electric");
//
//  doc["name"] = "Heating";
//  doc["mode_state_topic"] = tank_mode_state_topic;
//  doc["icon"] = "mdi:storage-tank";
//  doc["mode_command_topic"] = tank_mode_command_topic;
//  doc["unique_id"] = "tank01ae";
//  doc["temperature_unit"] = "C";
//  doc["temperature_state_topic"] = tank_temp_state_topic;
//  doc["current_temperature_topic"] = tank_temp_sensor_topic;
//  doc["temperature_command_topic"] = tank_temp_command_topic;
//  doc["optimistic"] = false;
//  doc["device"]["identifiers"] = "tank01";
//  doc["device"]["name"] = "Water Tank";  
//
//  serializeJson(doc, buf);
//  client.publish("homeassistant/water_heater/tank/config", buf);
//
//  doc.clear();

  doc["name"] = "Tank Temp";
  doc["device_class"] = "temperature";
  doc["state_topic"] = tank_temp_sensor_topic;
  doc["unit_of_measurement"] = "°C";
  doc["unique_id"] = "tank_temp01ae";
  doc["device"]["identifiers"] = "tank01";
  doc["device"]["name"] = "Water Tank";  
  

  serializeJson(doc, buf);
  client.publish("homeassistant/sensor/" temp_topic "/config", buf);

  doc.clear();

}


void postMqtt(const char* topic, float value) {
  char buf[10];
  snprintf(buf, sizeof(buf), "%.2f", value);
  client.publish(topic, buf);
}


void postMqtt(const char* topic, int16_t value) {
  char buf[10];
  snprintf(buf, sizeof(buf), "%d", value);
  client.publish(topic, buf);
}

void postMqtt(const char* topic, uint16_t value) {
  char buf[10];
  snprintf(buf, sizeof(buf), "%d", value);
  client.publish(topic, buf);
}

void postMqtt(const char* topic, int8_t value) {
  char buf[10];
  snprintf(buf, sizeof(buf), "%d", value);
  client.publish(topic, buf);
}
