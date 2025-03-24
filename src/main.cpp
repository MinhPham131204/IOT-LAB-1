#ifdef ESP8266
#include <ESP8266WiFi.h>
#else
#ifdef ESP32
#include <WiFi.h>
#include <WiFiClientSecure.h>
#endif // ESP32
#endif // ESP8266

#include <Arduino.h>
#include <DHT20.h>

#include "Arduino_MQTT_Client.h"
#include "Server_Side_RPC.h"
#include "ThingsBoard.h"

#define ENCRYPTED false


constexpr char WIFI_SSID[] = "LAPTOP-U7SSM0OR 7033";
constexpr char WIFI_PASSWORD[] = "12345678";

// See https://thingsboard.io/docs/getting-started-guides/helloworld/
// to understand how to obtain an access token
const char* TOKEN = "mze9614291gw4wsthfrz";

// Thingsboard we want to establish a connection too
const char* THINGSBOARD_SERVER = "app.coreiot.io";

// // // MQTT port used to communicate with the server, 1883 is the default unencrypted MQTT port,
// // // whereas 8883 would be the default encrypted SSL MQTT port
#if ENCRYPTED
const uint16_t THINGSBOARD_PORT = 8883;
#else
const uint16_t THINGSBOARD_PORT = 1883;
#endif

constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

constexpr const char RPC_JSON_METHOD[] = "example_json";
constexpr const char RPC_TEMPERATURE_METHOD[] = "example_set_temperature";
constexpr const char RPC_SWITCH_METHOD[] = "example_set_switch";
constexpr const char RPC_TEMPERATURE_KEY[] = "temp";
constexpr const char RPC_SWITCH_KEY[] = "switch";
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 3U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;


// Initialize underlying client, used to establish a connection
#if ENCRYPTED
WiFiClientSecure espClient;
#else
WiFiClient espClient;
#endif
// Initalize the Mqtt client instance
Arduino_MQTT_Client mqttClient(espClient);
// Initialize used apis
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
const std::array<IAPI_Implementation*, 1U> apis = {
    &rpc
};
// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

DHT20 dht20;
QueueHandle_t sensorQueue;

struct SensorData {
  double temperature;
  double humidity;
};

void TaskTemperature_Humidity(void *pvParameters){
  while(1){
    dht20.read();
    double temperature = dht20.getTemperature();
    double humidity = dht20.getHumidity();

    Serial.print("Temp: "); Serial.print(temperature); Serial.print(" *C ");
    Serial.print(" Humidity: "); Serial.print(humidity); Serial.print(" %");
    Serial.println();

    SensorData data = { temperature, humidity };
    xQueueSend(sensorQueue, &data, portMAX_DELAY);
    
    vTaskDelay(2000);
  }

}

void TaskSendData(void *pvParameters) {
  while(1) {
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      Serial.print("Connecting WiFi");
      while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
      }
      Serial.println("\nWiFi Connected");
    }

    if (!tb.connected()) {
      Serial.println("Connecting ThingsBoard...");
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("ThingsBoard connect failed");
      }
      Serial.println("Connected to ThingsBoard!");
    }
    SensorData data;
    if (xQueueReceive(sensorQueue, &data, pdMS_TO_TICKS(1000)) == pdTRUE) {
      tb.sendTelemetryData("temperature", data.temperature);
      tb.sendTelemetryData("humidity", data.humidity);
      Serial.printf("Sent -> Temp: %.2f°C, Humi: %.2f%%\n", data.temperature, data.humidity);
    }
    tb.loop(); // giữ kết nối MQTT
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(21,22);
  dht20.begin();

  sensorQueue = xQueueCreate(5, sizeof(SensorData));
  xTaskCreate(TaskSendData, "Send Data", 4096, NULL, 2, NULL);
  xTaskCreate(TaskTemperature_Humidity, "Read_Temp_Humi", 4096, NULL, 1, NULL);

}

void loop() {

}