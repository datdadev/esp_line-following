#include "network_handler.h"
#include "sensor_driver.h"
#include "system_config.h"
#include "motor_driver.h"
#include "servo_driver.h"
#include "robot_state.h"
#include "dashboard_html.h"  // your embedded HTML file
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// ===================== GLOBAL VARIABLES =====================
extern State state;          // Reference to state variable defined in main.cpp
extern float currentSpeed;   // Reference to speed variable defined in main.cpp

// ===================== NETWORK CONFIG =====================
#include "wifi_credentials.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// ===================== WEBSERVER & WEBSOCKET =====================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ===================== WEBSOCKET EVENT HANDLER =====================
void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                    AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n",
                  client->id(), client->remoteIP().toString().c_str());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0;
      String message = (char*)data;
      Serial.printf("WebSocket message: %s\n", message.c_str());

      // === Command handling ===
      if (message.indexOf("{\"command\":\"start\"}") != -1) {
        Serial.println("Start command received");
      } else if (message.indexOf("{\"command\":\"stop\"}") != -1) {
        Serial.println("Stop command received");
        state = STOP;
      } else if (message.indexOf("{\"command\":\"forward\"}") != -1) {
        setMotor(PWM_NORMAL);
        setServoAngle(90);
      } else if (message.indexOf("{\"command\":\"left\"}") != -1) {
        setServoAngle(70);
      } else if (message.indexOf("{\"command\":\"right\"}") != -1) {
        setServoAngle(110);
      }
    }
  }
}

// ===================== SEND TELEMETRY =====================
void sendTelemetry() {
  String json = "{";
  json += "\"sensors\":[";

  for (int i = 0; i < NUM_IR; i++) {
    json += lineSensor[i];
    if (i < NUM_IR - 1) json += ",";
  }
  json += "],";

  json += "\"midSensors\":[";
  for (int i = 0; i < NUM_IR; i++) {
    json += midSensor[i];
    if (i < NUM_IR - 1) json += ",";
  }
  json += "],";

  json += "\"speed\":" + String(currentSpeed) + ",";
  json += "\"battery\":8.4,";

  String stateStr = "IDLE";
  switch (state) {
    case INIT: stateStr = "INIT"; break;
    case IDLE: stateStr = "IDLE"; break;
    case LINE_FOLLOW: stateStr = "LINE_FOLLOW"; break;
    case AVOID_PREPARE: stateStr = "AVOID_PREPARE"; break;
    case AVOID_PATH: stateStr = "AVOID_PATH"; break;
    case MERGE_SEARCH: stateStr = "MERGE_SEARCH"; break;
    case TURN_LEFT_PREPARE: stateStr = "TURN_LEFT_PREPARE"; break;
    case SLOW_DOWN: stateStr = "SLOW_DOWN"; break;
    case STOP: stateStr = "STOP"; break;
    case LOST_LINE: stateStr = "LOST_LINE"; break;
  }

  json += "\"state\":\"" + stateStr + "\",";
  bool isObstacle = (sonarDistance > 0 && sonarDistance < SONAR_TH_OBS);
  json += "\"obstacle\":" + String(isObstacle ? "true" : "false") + ",";
  json += "\"ultrasonic\":" + String(sonarDistance) + ",";

  int16_t currentPWM = 0;
  if (getMotor(&currentPWM) == ERROR_SUCCESS)
    json += "\"pwm\":" + String(currentPWM) + ",";
  else
    json += "\"pwm\":0,";

  int8_t currentServoAngle = 0;
  if (getCurrentServoAngle(&currentServoAngle) == ERROR_SUCCESS)
    json += "\"servoAngle\":" + String(currentServoAngle) + ",";
  else
    json += "\"servoAngle\":90,";

  // Add line following error (e2) to telemetry
  json += "\"e2\":" + String(lineFollowError);

  json += "}";

  ws.textAll(json);
}

// ===================== WIFI SETUP =====================
int8_t initNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  uint32_t timeout = 20000;
  uint32_t startTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startTime < timeout) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" Failed to connect to WiFi");
    return ERROR_GENERAL_FAILURE;
  }

  Serial.println();
  Serial.print("Connected to WiFi. IP address: ");
  Serial.println(WiFi.localIP());

  return ERROR_SUCCESS;
}

// ===================== SERVER SETUP (NO FILESYSTEM) =====================
void setupWebSocket() {
  ws.onEvent(webSocketEvent);
  server.addHandler(&ws);

  // Serve dashboard directly from flash (no LittleFS)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", dashboard_html);  // dashboard_html in PROGMEM
  });

  server.begin();
  Serial.println("Server started (PROGMEM mode)");
}
