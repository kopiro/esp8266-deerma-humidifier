#include "Arduino.h"
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include <SoftwareSerial.h>

/*
Firmware documentation:
https://iot.mi.com/new/doc/embedded-development/wifi/module-dev/serial-communication
*/

#define SOFTWARE_VERSION "2025.04.03"

#define FLASH_PIN 0
#define STATUS_PIN 16

#define MQTT_POLLING_TIMEOUT 60000

SoftwareSerial SerialDebug(3, 1);

unsigned long currentTime = millis();

#define BOARD_PREFIX "esp8266-deerma-humidifier"

char mqtt_server[64];
char mqtt_user[64];
char mqtt_pass[64];

WiFiManagerParameter wifi_param_mqtt_server("mqtt_server", "MQTT server", mqtt_server, 64);
WiFiManagerParameter wifi_param_mqtt_user("mqtt_user", "MQTT username", mqtt_user, 64);
WiFiManagerParameter wifi_param_mqtt_pass("mqtt_pass", "MQTT password", mqtt_pass, 64);


String BOARD_ID;

String MQTT_TOPIC_STATE;
String MQTT_TOPIC_COMMAND;
String MQTT_TOPIC_DEBUG;
String MQTT_TOPIC_AVAILABILITY;

String MQTT_TOPIC_AUTOCONF_HUMIDITY_SENSOR;
String MQTT_TOPIC_AUTOCONF_TEMPERATURE_SENSOR;
String MQTT_TOPIC_AUTOCONF_WIFI_SENSOR;
String MQTT_TOPIC_AUTOCONF_WATER_TANK_SENSOR;
String MQTT_TOPIC_AUTOCONF_HUMIDIFIER;
String MQTT_TOPIC_AUTOCONF_SOUND_SWITCH;
String MQTT_TOPIC_AUTOCONF_LED_SWITCH;

WiFiManager wifiManager(SerialDebug);
WiFiClient wifiClient;
PubSubClient mqttClient;


enum humMode_t { unknown = -1, low = 1, medium = 2, high = 3, setpoint = 4 };
struct humidifierState_t {
  boolean powerOn;

  humMode_t mode = (humMode_t)-1;

  int humiditySetpoint = -1;

  int currentHumidity = -1;
  int currentTemperature = -1;

  boolean soundEnabled;
  boolean ledEnabled;

  boolean waterTankInstalled;
  boolean waterTankEmpty;
};

// Global state
humidifierState_t state;

unsigned long mqttLastTime = millis();

void sendMQTTMessage(String topic, String message, bool retained) {
  SerialDebug.printf("MQTT message - topic: <%s>, message: <%s> -> ", topic,
                     message);
  if (mqttClient.publish(topic.c_str(), message.c_str(), retained)) {
    SerialDebug.println("sent");
  } else {
    SerialDebug.println("error");
  }
}

void sendHAAutoDiscovery() {
  char mqttPayload[2048];
  DynamicJsonDocument device(256);
  DynamicJsonDocument autoconfPayload(1024);
  StaticJsonDocument<64> identifiersDoc;
  JsonArray identifiers = identifiersDoc.to<JsonArray>();

  identifiers.add(BOARD_ID);

  device["identifiers"] = identifiers;
  device["manufacturer"] = "Deerma";
  device["model"] = "Mi Smart Antibacterial Humidifier";
  device["name"] = BOARD_ID;
  device["sw_version"] = SOFTWARE_VERSION;

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["name"] = BOARD_ID + String(" Humidity");
  autoconfPayload["device_class"] = "humidity";
  autoconfPayload["unit_of_measurement"] = "%";
  autoconfPayload["value_template"] = "{{value_json.humidity}}";
  autoconfPayload["unique_id"] = BOARD_ID + String("_humidity");

  serializeJson(autoconfPayload, mqttPayload);
  sendMQTTMessage(MQTT_TOPIC_AUTOCONF_HUMIDITY_SENSOR, mqttPayload, true);

  autoconfPayload.clear();

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["name"] = BOARD_ID + String(" Temperature");
  autoconfPayload["device_class"] = "temperature";
  autoconfPayload["unit_of_measurement"] = "°C";
  autoconfPayload["value_template"] = "{{value_json.temperature}}";
  autoconfPayload["unique_id"] = BOARD_ID + String("_temperature");

  serializeJson(autoconfPayload, mqttPayload);
  sendMQTTMessage(MQTT_TOPIC_AUTOCONF_TEMPERATURE_SENSOR, mqttPayload, true);

  autoconfPayload.clear();

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["name"] = BOARD_ID + String(" WiFi");
  autoconfPayload["value_template"] = "{{value_json.wifi.rssi}}";
  autoconfPayload["unique_id"] = BOARD_ID + String("_wifi");
  autoconfPayload["unit_of_measurement"] = "dBm";
  autoconfPayload["json_attributes_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["json_attributes_template"] =
      "{\"ssid\": \"{{value_json.wifi.ssid}}\", \"ip\": "
      "\"{{value_json.wifi.ip}}\"}";
  autoconfPayload["icon"] = "mdi:wifi";
  autoconfPayload["entity_category"] = "diagnostic";

  serializeJson(autoconfPayload, mqttPayload);
  sendMQTTMessage(MQTT_TOPIC_AUTOCONF_WIFI_SENSOR, mqttPayload, true);

  autoconfPayload.clear();

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["name"] = BOARD_ID + String(" Water Tank");
  autoconfPayload["value_template"] = "{{value_json.waterTank}}";
  autoconfPayload["unique_id"] = BOARD_ID + String("_water_tank");
  autoconfPayload["icon"] = "mdi:cup-water";
  autoconfPayload["entity_category"] = "diagnostic";

  serializeJson(autoconfPayload, mqttPayload);
  sendMQTTMessage(MQTT_TOPIC_AUTOCONF_WATER_TANK_SENSOR, mqttPayload, true);

  autoconfPayload.clear();

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["command_topic"] = MQTT_TOPIC_COMMAND;
  autoconfPayload["name"] = BOARD_ID + String(" Sound");
  autoconfPayload["value_template"] = "{{value_json.sound}}";
  autoconfPayload["unique_id"] = BOARD_ID + String("_sound");
  autoconfPayload["payload_on"] = "{\"sound\": \"on\"}";
  autoconfPayload["payload_off"] = "{\"sound\": \"off\"}";
  autoconfPayload["state_on"] = "on";
  autoconfPayload["state_off"] = "off";
  autoconfPayload["icon"] = "mdi:volume-high";
  autoconfPayload["entity_category"] = "config";

  serializeJson(autoconfPayload, mqttPayload);
  sendMQTTMessage(MQTT_TOPIC_AUTOCONF_SOUND_SWITCH, mqttPayload, true);

  autoconfPayload.clear();

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["command_topic"] = MQTT_TOPIC_COMMAND;
  autoconfPayload["name"] = BOARD_ID + String(" LED");
  autoconfPayload["value_template"] = "{{value_json.led}}";
  autoconfPayload["unique_id"] = BOARD_ID + String("_led");
  autoconfPayload["payload_on"] = "{\"led\": \"on\"}";
  autoconfPayload["payload_off"] = "{\"led\": \"off\"}";
  autoconfPayload["state_on"] = "on";
  autoconfPayload["state_off"] = "off";
  autoconfPayload["icon"] = "mdi:led-outline";
  autoconfPayload["entity_category"] = "config";

  serializeJson(autoconfPayload, mqttPayload);
  sendMQTTMessage(MQTT_TOPIC_AUTOCONF_LED_SWITCH, mqttPayload, true);

  autoconfPayload.clear();

  autoconfPayload["device"] = device.as<JsonObject>();
  autoconfPayload["availability_topic"] = MQTT_TOPIC_AVAILABILITY;
  autoconfPayload["name"] = BOARD_ID + String(" Humidifier");
  autoconfPayload["unique_id"] = BOARD_ID + String("_humidifier");
  autoconfPayload["device_class"] = "humidifier";

  autoconfPayload["max_humidity"] = 99;
  autoconfPayload["min_humidity"] = 0;

  autoconfPayload["state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["state_value_template"] =
      "{\"state\": \"{{value_json.state}}\"}";
  autoconfPayload["payload_on"] = "{\"state\": \"on\"}";
  autoconfPayload["payload_off"] = "{\"state\": \"off\"}";
  autoconfPayload["command_topic"] = MQTT_TOPIC_COMMAND;
  autoconfPayload["state_value_template"] =
      "{\"state\": \"{{value_json.state}}\"}";
  autoconfPayload["payload_on"] = "{\"state\": \"on\"}";
  autoconfPayload["payload_off"] = "{\"state\": \"off\"}";

  StaticJsonDocument<64> modesDoc;
  JsonArray modes = modesDoc.to<JsonArray>();

  autoconfPayload["target_humidity_state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["target_humidity_command_topic"] = MQTT_TOPIC_COMMAND;
  autoconfPayload["target_humidity_state_template"] =
      "{{value_json.humiditySetpoint | int}}";
  autoconfPayload["target_humidity_command_template"] =
      "{\"humiditySetpoint\": {{value | int}}}";

  modes.add("setpoint");
  modes.add("low");
  modes.add("medium");
  modes.add("high");

  autoconfPayload["modes"] = modes;
  autoconfPayload["mode_state_topic"] = MQTT_TOPIC_STATE;
  autoconfPayload["mode_command_topic"] = MQTT_TOPIC_COMMAND;
  autoconfPayload["mode_state_template"] = "{{value_json.mode}}";
  autoconfPayload["mode_command_template"] = "{\"mode\": \"{{value}}\"}";

  serializeJson(autoconfPayload, mqttPayload);
  sendMQTTMessage(MQTT_TOPIC_AUTOCONF_HUMIDIFIER, mqttPayload, true);
}

char serialRxBuf[255];
char serialTxBuf[255];

boolean DebugEnabled = false;
boolean UARTEnabled = true;

#define PROP_POWER_SIID 2
#define PROP_POWER_PIID 1

#define PROP_READ_SIID 3
#define PROP_HUMIDITY_PIID 1
#define PROP_TEMPERATURE_PIID 7

#define PROP_SET_SIID 2
#define PROP_HUMIDITY_MODE_PIID 5
#define PROP_HUMIDITY_SETPOINT_PIID 6

#define PROP_SOUND_SIID 5
#define PROP_SOUND_ENABLED_PIID 1

#define PROP_LED_SIID 6
#define PROP_LED_ENABLED_PIID 1

#define PROP_WATER_TANK_SIID 7
#define PROP_WATER_TANK_EMPTY_PIID 1
#define PROP_WATER_TANK_REMOVED_PIID 2

const int DOWNSTREAM_QUEUE_SIZE = 50;
const int DOWNSTREAM_QUEUE_ELEM_SIZE = 51;

char downstreamQueue[DOWNSTREAM_QUEUE_SIZE][DOWNSTREAM_QUEUE_ELEM_SIZE];
int downstreamQueueIndex = -1;
char nextDownstreamMessage[DOWNSTREAM_QUEUE_ELEM_SIZE] = "";

void saveConfig() {
  SerialDebug.println("Saving config...");

  // Collect parameters
  strcpy(mqtt_server, wifi_param_mqtt_server.getValue());
  strcpy(mqtt_user, wifi_param_mqtt_user.getValue());
  strcpy(mqtt_pass, wifi_param_mqtt_pass.getValue());
  
  DynamicJsonDocument json(512);
  json["mqtt_server"] = wifi_param_mqtt_server.getValue();
  json["mqtt_user"] = wifi_param_mqtt_user.getValue();
  json["mqtt_pass"] = wifi_param_mqtt_pass.getValue();

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    SerialDebug.println("Failed to open config file for writing");
    return;
  }

  serializeJson(json, configFile);
  configFile.close();

  SerialDebug.printf("Saved JSON: %s\n", json.as<String>().c_str());
}

void loadConfig() {
  SerialDebug.println("Loading config");

  // Initialize SPIFFS
  if (!SPIFFS.begin()) {
    SerialDebug.println("Failed to initialize SPIFFS");
    return;
  }

  if (!SPIFFS.exists("/config.json")) {
    SerialDebug.println("Config file not found, please configure the ESP by connecting to its Wi-Fi hotspot");
    return;
  }

  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    SerialDebug.println("Failed to open config file");
    return;
  }
  
  const size_t size = configFile.size();
  std::unique_ptr<char[]> buf(new char[size]);

  configFile.readBytes(buf.get(), size);
  DynamicJsonDocument json(512);

  if (DeserializationError::Ok != deserializeJson(json, buf.get())) {
    SerialDebug.println("Failed to parse config fileDebug");
    return;
  }

  // Copy the values to the global variables
  strcpy(mqtt_server, json["mqtt_server"]);
  strcpy(mqtt_user, json["mqtt_user"]);
  strcpy(mqtt_pass, json["mqtt_pass"]);

  wifi_param_mqtt_server.setValue(mqtt_server, sizeof(mqtt_server));
  wifi_param_mqtt_user.setValue(mqtt_user, sizeof(mqtt_user));
  wifi_param_mqtt_pass.setValue(mqtt_pass, sizeof(mqtt_pass));

  SerialDebug.printf("Config JSON: %s\n", json.as<String>().c_str());
}

void setupMDNS() {
  if (MDNS.begin(BOARD_ID)) {
    SerialDebug.println("MDNS responder started");
  } else {
    SerialDebug.println("MDNS responder got an error");
  }
}

void setupGeneric() {
  SerialDebug.begin(9600);
  SerialDebug.println("BOOT");

  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, HIGH);

  pinMode(FLASH_PIN, INPUT_PULLUP);

  BOARD_ID = String(BOARD_PREFIX) + "-" + String(ESP.getChipId(), HEX);
  SerialDebug.printf("Board Identifier: %s\n", BOARD_ID.c_str());

  MQTT_TOPIC_STATE = BOARD_ID + "/state";
  MQTT_TOPIC_COMMAND = BOARD_ID + "/command";
  MQTT_TOPIC_DEBUG = BOARD_ID + "/debug";
  MQTT_TOPIC_AVAILABILITY = BOARD_ID + "/availability";

  // Home Assistant auto-discovery topics
  MQTT_TOPIC_AUTOCONF_TEMPERATURE_SENSOR = String("homeassistant/sensor/") + 
                                           BOARD_ID + 
                                           String("/temperature/config");
  MQTT_TOPIC_AUTOCONF_HUMIDITY_SENSOR =
      String("homeassistant/sensor/") + BOARD_ID + String("/humidity/config");
  MQTT_TOPIC_AUTOCONF_WIFI_SENSOR =
      String("homeassistant/sensor/") + BOARD_ID + String("/wifi/config");
  MQTT_TOPIC_AUTOCONF_WATER_TANK_SENSOR =
      String("homeassistant/binary_sensor/") + BOARD_ID +
      String("/water_tank/config");
  MQTT_TOPIC_AUTOCONF_HUMIDIFIER =
      String("homeassistant/switch/") + BOARD_ID + String("/humidifier/config");
  MQTT_TOPIC_AUTOCONF_SOUND_SWITCH =
      String("homeassistant/switch/") + BOARD_ID + String("/sound/config");
  MQTT_TOPIC_AUTOCONF_LED_SWITCH =
      String("homeassistant/switch/") + BOARD_ID + String("/led/config");

  loadConfig();
}

void setupWifi() {
  if (WiFi.status() == WL_NO_SHIELD) {
    SerialDebug.println("WiFi shield not present");
    return;
  }

  WiFi.hostname(BOARD_ID);

  wifiManager.setConfigPortalBlocking(false);
  wifiManager.setDebugOutput(true);
  wifiManager.setSaveParamsCallback(saveConfig);

  wifiManager.addParameter(&wifi_param_mqtt_server);
  wifiManager.addParameter(&wifi_param_mqtt_user);
  wifiManager.addParameter(&wifi_param_mqtt_pass);

  if (WiFi.status() == WL_CONNECTED || wifiManager.autoConnect(BOARD_ID.c_str())) {
    // If we are connected to autoConnect, start the web portal for the configuration screen,
    // as otherwise it would not be accessible.
    WiFi.mode(WIFI_STA); // Force the connection to be STA, not AP
    wifiManager.startWebPortal();
  } else {
    SerialDebug.println("Failed to connect to WiFi");
  }
}

void loopWifi() { wifiManager.process(); }


boolean shouldUpdateState = false;

void queueDownstreamMessage(const char *message) {
  if (downstreamQueueIndex >= DOWNSTREAM_QUEUE_SIZE - 1) {
    SerialDebug.printf("Error: Queue is full. Dropping message: <%s>\n",
                       message);
    return;
  }

  downstreamQueueIndex++;
  strncpy(downstreamQueue[downstreamQueueIndex], message, DOWNSTREAM_QUEUE_ELEM_SIZE - 1);
  downstreamQueue[downstreamQueueIndex][DOWNSTREAM_QUEUE_ELEM_SIZE - 1] = '\0';
}


void sendNetworkStatus(boolean isConnected) {
  queueDownstreamMessage("MIIO_net_change cloud");
}

void clearDownstreamQueueAtIndex(int index) {
  memset(downstreamQueue[index], 0, sizeof(downstreamQueue[index]));
}

void fillNextDownstreamMessage() {
  if (downstreamQueueIndex < 0) {
    strncpy(nextDownstreamMessage, "none", DOWNSTREAM_QUEUE_ELEM_SIZE - 1);
    nextDownstreamMessage[DOWNSTREAM_QUEUE_ELEM_SIZE - 1] = '\0';
  } else if (downstreamQueueIndex == 0) {
    strncpy(nextDownstreamMessage, downstreamQueue[0], DOWNSTREAM_QUEUE_ELEM_SIZE - 1);
    nextDownstreamMessage[DOWNSTREAM_QUEUE_ELEM_SIZE - 1] = '\0';
    downstreamQueueIndex--;
  } else {
    strncpy(nextDownstreamMessage, downstreamQueue[0], DOWNSTREAM_QUEUE_ELEM_SIZE - 1);
    nextDownstreamMessage[DOWNSTREAM_QUEUE_ELEM_SIZE - 1] = '\0';
    for (int i = 0; i < downstreamQueueIndex; i++) {
      strncpy(downstreamQueue[i], downstreamQueue[i + 1], DOWNSTREAM_QUEUE_ELEM_SIZE - 1);
      downstreamQueue[i][DOWNSTREAM_QUEUE_ELEM_SIZE - 1] = '\0';
    }
    downstreamQueueIndex--;
  }
}

void resetWifiSettingsAndReboot() {
  SerialDebug.println("Resetting WiFi settings and rebooting");
  wifiManager.resetSettings();
  sendNetworkStatus(false);
  delay(3000);
  ESP.restart();
}

void mqttConnect() {
  if (strlen(mqtt_server) == 0) {
    SerialDebug.println("MQTT server not configured, skipping connection");
    return;
  }
  
  SerialDebug.printf("Connecting to MQTT server: host = %s (user: %s : pass: %s)... ", mqtt_server, mqtt_user, mqtt_pass);

  if (mqttClient.connect(BOARD_ID.c_str(), mqtt_user, mqtt_pass, MQTT_TOPIC_AVAILABILITY.c_str(), 1, true, "offline")) {
    SerialDebug.println("connected");

    mqttClient.subscribe(MQTT_TOPIC_COMMAND.c_str());

    // Subscribe to HA restart and send the availability message
    mqttClient.subscribe("homeassistant/status");
    sendHAAutoDiscovery();

    sendMQTTMessage(MQTT_TOPIC_AVAILABILITY, "online", true);

    sendNetworkStatus(true);
  } else {
    SerialDebug.println("Unable to connect to MQTT broker");
  }
}


void loopMDNS() { MDNS.update(); }

void loopMQTT() {
  if (mqttClient.connected()) {
    mqttClient.loop();
    return;
  }

  if (millis() < mqttLastTime + MQTT_POLLING_TIMEOUT) {
    return;
  }

  SerialDebug.println("Connection to MQTT lost, reconnecting...");
  mqttLastTime = millis();

  mqttConnect();
}

void queuePropertyChange(int siid, int piid, const char *value) {
  String msg = "set_properties " + String(siid) + " " + String(piid) + " " +
               String(value);
  queueDownstreamMessage(msg.c_str());
}

void queuePropertyChange(int siid, int piid, char *value) {
  String msg = "set_properties " + String(siid) + " " + String(piid) + " " +
               String(value);
  queueDownstreamMessage(msg.c_str());
}

void setPowerState(boolean powerOn) {
  queuePropertyChange(PROP_POWER_SIID, PROP_POWER_PIID,
                      powerOn ? "true" : "false");
}

void setLEDState(boolean ledEnabled) {
  queuePropertyChange(PROP_LED_SIID, PROP_LED_ENABLED_PIID,
                      ledEnabled ? "true" : "false");
}

void setSoundState(boolean soundEnabled) {
  queuePropertyChange(PROP_SOUND_SIID, PROP_SOUND_ENABLED_PIID,
                      soundEnabled ? "true" : "false");
}

void setHumidityMode(humMode_t mode) {
  queuePropertyChange(PROP_SET_SIID, PROP_HUMIDITY_MODE_PIID,
                      String(mode).c_str());
}

void setHumiditySetpoint(uint8_t value) {
  uint8_t clampedValue = value > 60 ? 60 : (value < 40 ? 40 : value);
  queuePropertyChange(PROP_SET_SIID, PROP_HUMIDITY_SETPOINT_PIID,
                      String(clampedValue).c_str());
}

void publishState() {
  SerialDebug.println("Publishing new state");

  DynamicJsonDocument wifiJson(192);
  DynamicJsonDocument stateJson(604);
  String payload;

  wifiJson["ssid"] = WiFi.SSID();
  wifiJson["ip"] = WiFi.localIP().toString();
  wifiJson["rssi"] = WiFi.RSSI();

  stateJson["state"] = state.powerOn ? "on" : "off";

  switch (state.mode) {
  case (humMode_t)setpoint:
    stateJson["mode"] = "setpoint";
    break;
  case (humMode_t)low:
    stateJson["mode"] = "low";
    break;
  case (humMode_t)medium:
    stateJson["mode"] = "medium";
    break;
  case (humMode_t)high:
    stateJson["mode"] = "high";
    break;
  default:
    stateJson["mode"] = "unknown";
  }

  stateJson["humiditySetpoint"] = state.humiditySetpoint;

  stateJson["humidity"] = state.currentHumidity;
  stateJson["temperature"] = state.currentTemperature;

  stateJson["sound"] = state.soundEnabled ? "on" : "off";
  stateJson["led"] = state.ledEnabled ? "on" : "off";

  stateJson["waterTank"] = state.waterTankInstalled
                               ? (state.waterTankEmpty ? "empty" : "full")
                               : "missing";

  stateJson["wifi"] = wifiJson.as<JsonObject>();

  stateJson["__debug"] = DebugEnabled;
  stateJson["__uart"] = UARTEnabled;

  serializeJson(stateJson, payload);
  SerialDebug.printf("State JSON: %s\n", payload.c_str());

  sendMQTTMessage(MQTT_TOPIC_STATE, payload, false);
}


void loopUART() {
  if (!Serial.available()) {
    return;
  }

  memset(serialRxBuf, 0, sizeof(serialRxBuf));
  Serial.readBytesUntil('\r', serialRxBuf, 250);

  if (DebugEnabled) {
    sendMQTTMessage(MQTT_TOPIC_DEBUG, serialRxBuf, false);
  }

  SerialDebug.printf("UART says: <%s>\n", serialRxBuf);

  if (strncmp(serialRxBuf, "properties_changed", 18) == 0) {
    int propSiid = 0;
    int propPiid = 0;
    char propValue[6] = "";

    int propChanged = sscanf(serialRxBuf, "properties_changed %d %d %s",
                             &propSiid, &propPiid, propValue);

    if (propChanged == 3) {
      SerialDebug.printf("Property changed: %d %d %s\n", propSiid, propPiid,
                         propValue);

      if (propSiid == PROP_POWER_SIID && propPiid == PROP_POWER_PIID) {
        state.powerOn = strncmp(propValue, "true", 4) == 0;
        SerialDebug.printf("New power status: <%s>\n",
                           state.powerOn ? "on" : "off");
      } else if (propSiid == PROP_SET_SIID &&
                 propPiid == PROP_HUMIDITY_MODE_PIID) {
        state.mode = (humMode_t)atoi(propValue);
        SerialDebug.printf("New humidityMode: <%d>\n", state.mode);
      } else if (propSiid == PROP_SET_SIID &&
                 propPiid == PROP_HUMIDITY_SETPOINT_PIID) {
        state.humiditySetpoint = atoi(propValue);
        SerialDebug.printf("New humiditySetpoint: <%d>\n",
                           state.humiditySetpoint);
      } else if (propSiid == PROP_READ_SIID && propPiid == PROP_HUMIDITY_PIID) {
        state.currentHumidity = atoi(propValue);
        SerialDebug.printf("New currentHumidity: <%d>\n",
                           state.currentHumidity);
      } else if (propSiid == PROP_READ_SIID &&
                 propPiid == PROP_TEMPERATURE_PIID) {
        state.currentTemperature = atoi(propValue);
        SerialDebug.printf("New currentTemperature: <%d>\n",
                           state.currentTemperature);
      } else if (propSiid == PROP_LED_SIID &&
                 propPiid == PROP_LED_ENABLED_PIID) {
        state.ledEnabled = strncmp(propValue, "true", 4) == 0;
        SerialDebug.printf("New ledEnabled: <%s>\n",
                           state.ledEnabled ? "true" : "false");
      } else if (propSiid == PROP_SOUND_SIID &&
                 propPiid == PROP_SOUND_ENABLED_PIID) {
        state.soundEnabled = strncmp(propValue, "true", 4) == 0;
        SerialDebug.printf("New soundEnabled: <%s>\n",
                           state.soundEnabled ? "true" : "false");
      } else if (propSiid == PROP_WATER_TANK_SIID &&
                 propPiid == PROP_WATER_TANK_REMOVED_PIID) {
        state.waterTankInstalled = !(strncmp(propValue, "true", 4) == 0);
        SerialDebug.printf("New waterTankInstalled: <%s>\n",
                           state.waterTankInstalled ? "true" : "false");
      } else if (propSiid == PROP_WATER_TANK_SIID &&
                 propPiid == PROP_WATER_TANK_EMPTY_PIID) {
        state.waterTankEmpty = strncmp(propValue, "true", 4) == 0;
        SerialDebug.printf("New waterTankEmpty: <%s>\n",
                           state.waterTankEmpty ? "true" : "false");
      } else {
        SerialDebug.printf("Unknown property: <%s>\n", serialRxBuf);
      }

      shouldUpdateState = true;
    }

    Serial.print("ok\r");
    return;
  }

  if (strncmp(serialRxBuf, "get_down", 8) == 0) {
    if (shouldUpdateState == true) {
      shouldUpdateState = false;
      publishState();
    }

    fillNextDownstreamMessage();

    if (strncmp(nextDownstreamMessage, "none", 4) != 0) {
      SerialDebug.printf("Sending: %s\n", nextDownstreamMessage);
    }

    memset(serialTxBuf, 0, sizeof(serialTxBuf));
    snprintf(serialTxBuf, sizeof(serialTxBuf), "down %s\r", nextDownstreamMessage);
    Serial.print(serialTxBuf);

    return;
  }

  if (strncmp(serialRxBuf, "net", 3) == 0) {
    // We need to always respond with cloud because otherwise the connection to the humidifier will break for some reason
    Serial.print("cloud\r");
    return;
  }

  if (strncmp(serialRxBuf, "mcu_version", 11) == 0 ||
      strncmp(serialRxBuf, "model", 5) == 0 ||
      strncmp(serialRxBuf, "event_occured", 13) == 0) {
    Serial.print("ok\r");
    return;
  }

  if (strncmp(serialRxBuf, "restore", 7) == 0) {
    resetWifiSettingsAndReboot();
    return;
  }

  if (strncmp(serialRxBuf, "result", 6) == 0) {
    Serial.print("ok\r");
    return;
  }

  SerialDebug.printf("UART unexpected: %s\n", serialRxBuf);
}

void mqttCallback(char *_topic, byte *_payload, unsigned int length) {
  String topic = String(_topic);
  String payload = String((char*)_payload).substring(0, length);

  SerialDebug.printf("MQTT callback with topic <%s> and payload <%s>\n", topic.c_str(), payload.c_str());

  if (topic == "homeassistant/status") {
    if (payload == "online") {
      sendHAAutoDiscovery();
      sendMQTTMessage(MQTT_TOPIC_AVAILABILITY, "online", true);
    }
    return;
  }

  if (topic == "command") {
    DynamicJsonDocument commandJson(256);
    DeserializationError err = deserializeJson(commandJson, _payload);

    if (err) {
      SerialDebug.println("Error deserializing JSON");
      return;
    }

    String stateCommand = commandJson["state"].as<String>();
    String modeCommand = commandJson["mode"].as<String>();
    String soundCommand = commandJson["sound"].as<String>();
    String ledCommand = commandJson["led"].as<String>();
    String command = commandJson["command"].as<String>();

    long humiditySetpointCommand = commandJson["humiditySetpoint"] | -1;

    if (stateCommand == "off") {
      setPowerState(false);
    } else if (stateCommand == "on") {
      setPowerState(true);
    }

    if (modeCommand == "low") {
      setHumidityMode((humMode_t)low);
    } else if (modeCommand == "medium") {
      setHumidityMode((humMode_t)medium);
    } else if (modeCommand == "high") {
      setHumidityMode((humMode_t)high);
    } else if (modeCommand == "setpoint") {
      setHumidityMode((humMode_t)setpoint);
    }

    if (soundCommand == "off") {
      setSoundState(false);
    } else if (soundCommand == "on") {
      setSoundState(true);
    }

    if (ledCommand == "off") {
      setLEDState(false);
    } else if (ledCommand == "on") {
      setLEDState(true);
    }

    if (humiditySetpointCommand > -1) {
      setHumiditySetpoint((uint8_t)humiditySetpointCommand);
    }

    // Internal commands
    if (command == "__reboot") {
      ESP.restart();
    } else if (command == "__reset") {
      resetWifiSettingsAndReboot();
    } else if (command == "__debug") {
      DebugEnabled = true;
    } else if (command == "__undebug") {
      DebugEnabled = false;
    } else if (command == "__enableUART") {
      UARTEnabled = true;
    } else if (command == "__disableUART") {
      UARTEnabled = false;
    } else if (command == "__publishState") {
      publishState();
    }
  }
}

void setupMQTT() {
  mqttClient.setClient(wifiClient);

  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setKeepAlive(10);
  mqttClient.setBufferSize(2048);
  mqttClient.setCallback(mqttCallback);

  mqttConnect();
}


void setupUART() {
  Serial.begin(115200);
  Serial.swap();
  delay(1000);
  sendNetworkStatus(false);
}

void setupOTA() {
  ArduinoOTA.onStart([]() {});
  ArduinoOTA.onEnd([]() {});
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {});
  ArduinoOTA.onError([](ota_error_t error) {});
  ArduinoOTA.setHostname(BOARD_ID.c_str());
  ArduinoOTA.setPassword(BOARD_ID.c_str());
  ArduinoOTA.begin();
}

void loopOTA() { ArduinoOTA.handle(); }


void setup() {
  setupUART();

  setupGeneric();

  setupWifi();
  setupMDNS();
  setupOTA();
  setupMQTT();
}

void loop() {
  if (UARTEnabled) {
    loopUART();
  }

  loopWifi();
  loopMDNS();
  loopOTA();
  loopMQTT();
}