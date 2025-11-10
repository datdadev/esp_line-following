#ifndef NETWORK_HANDLER_H
#define NETWORK_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "error_codes.h"

// ===================== NETWORK CONFIG =====================
extern const char* ssid;
extern const char* password;

// ===================== WEBSERVER & WEBSOCKET =====================
extern AsyncWebServer server;
extern AsyncWebSocket ws;

// ===================== EXTERNAL VARIABLES =====================
extern float lineFollowError;  // Error value from line following (e2)

// ===================== FUNCTION DECLARATIONS =====================
int8_t initNetwork();  // Returns error code
void sendTelemetry();
void setupWebSocket();
void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);

#endif // NETWORK_HANDLER_H