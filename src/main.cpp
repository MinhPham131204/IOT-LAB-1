#include <Arduino.h>
#include <WiFi.h>
#include <DHT20.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

#include "Arduino_MQTT_Client.h"
#include "Server_Side_RPC.h"
#include "ThingsBoard.h"

bool isFirmwareUpgradeTriggered = false;
unsigned long lastSendTime = 0;
const unsigned long interval = 2000;

const char* ssid = "ACLAB";
const char* password = "ACLAB2023";
const char* mqttServer = "app.coreiot.io";
const int mqttPort = 1883;
const char* mqttUsername = "mze9614291gw4wsthfrz";
const char* mqttPassword = "";
const char* otaTopic = "v1/devices/me/attributes";

// MQTT client cho ThingsBoard
WiFiClient espClient;
PubSubClient mqttClient(espClient);

Arduino_MQTT_Client MQTTClient(espClient);

// ThingsBoard và RPC setup
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 3U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;

Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
const std::array<IAPI_Implementation*, 1U> apis = { &rpc };
ThingsBoard tb(MQTTClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

// DHT20 Sensor
DHT20 dht20;
QueueHandle_t sensorQueue;

struct SensorData {
  double temperature;
  double humidity;
};

void readTempAndHumi() {
  dht20.read();
  double temperature = dht20.getTemperature();
  double humidity = dht20.getHumidity();

  Serial.printf("Temp: %.2f °C | Humidity: %.2f %%\n", temperature, humidity);

  SensorData data = { temperature, humidity };
  xQueueSend(sensorQueue, &data, portMAX_DELAY);

  vTaskDelay(2000);
}

void sendData() {
  SensorData data;
  if (xQueueReceive(sensorQueue, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
    tb.sendTelemetryData("temperature", data.temperature);
    tb.sendTelemetryData("humidity", data.humidity);
    Serial.printf("Sent -> Temp: %.2f°C, Humi: %.2f%%\n", data.temperature, data.humidity);
  }
  tb.loop();  // Duy trì kết nối MQTT
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
}

void connectToMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT...");
    if (mqttClient.connect("ESP32Client", mqttUsername, mqttPassword)) {
      Serial.println("Connected to MQTT!");
      mqttClient.subscribe(otaTopic);
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, otaTopic) == 0) {
    if (!isFirmwareUpgradeTriggered) {
      isFirmwareUpgradeTriggered = true;
      Serial.println("Firmware upgrade triggered!");
      // OTA update sẽ được xử lý tự động trong loop()
    }
  }
}

void connectToThingsBoard() {
  while (!tb.connected()) {
    Serial.println("Connecting to ThingsBoard...");
    if (tb.connect(mqttServer, mqttUsername, mqttPort)) {
      Serial.println("Connected to ThingsBoard!");
    } else {
      Serial.println("Failed to connect. Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21,22);
  dht20.begin();
  connectToWiFi();
  connectToMQTT();
  mqttClient.setCallback(callback);

  // Thiết lập OTA
  ArduinoOTA.setPort(3232);
  ArduinoOTA.setHostname("ESP32Device");

  ArduinoOTA.onStart([]() {
    Serial.println("OTA Update started...");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("OTA Update completed!");
    isFirmwareUpgradeTriggered = false;
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  sensorQueue = xQueueCreate(5, sizeof(SensorData));
  dht20.begin();
}

void loop() {
  if (!mqttClient.connected()) {
    connectToMQTT();
  }

  mqttClient.loop();
  ArduinoOTA.handle();
  // unsigned long now = millis();

  // if (now - lastSendTime >= interval) {
  //   readTempAndHumi();
  //   sendData();
  //   lastSendTime = now;
  // }

  // tb.loop();
}
