#include "network_handler.h"
#include "sensor_driver.h"
#include "system_config.h"
#include "motor_driver.h"
#include "servo_driver.h"
#include "robot_state.h"
#include <ArduinoJson.h>

// ===================== GLOBAL VARIABLES =====================
extern State state;  // Reference to the state variable defined in main.cpp
extern float currentSpeed;  // Reference to the speed variable defined in main.cpp

// ===================== NETWORK CONFIG =====================
#include "wifi_credentials.h"  // Include credentials from separate header file

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

// ===================== WEBSERVER & WEBSOCKET =====================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");



// ===================== WEBSOCKET EVENT HANDLER =====================
void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if(type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
  } else if(type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client #%u disconnected\n", client->id());
  } else if(type == WS_EVT_DATA) {
    // Decode the WebSocket message as a JSON string
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    if(info->final && info->index == 0 && info->len == len) {
      if(info->opcode == WS_TEXT) {
        data[len] = 0;
        String message = (char*)data;
        Serial.printf("WebSocket message: %s\n", message.c_str());
        
        // Parse and handle control commands
        if (message.indexOf("{\"command\":\"start\"}") != -1) {
          // This would need to change the global state variable
          // We'll send a special command that gets handled in main loop
          Serial.println("Start command received");
        } else if (message.indexOf("{\"command\":\"stop\"}") != -1) {
          Serial.println("Stop command received");
          state = STOP;  // Set the robot state to stop
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
}

// ===================== HTML DASHBOARD =====================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Dashboard</title>
    <!-- Using Tailwind CSS for a modern, responsive UI -->
    <script src="https://cdn.tailwindcss.com"></script>
    <style>
        /* Custom transition for sensor bars */
        .sensor-bar {
            transition: height 0.15s ease, background-color 0.15s ease;
        }
    </style>
</head>
<body class="bg-gray-900 text-gray-100 font-sans">

    <div class="container mx-auto p-4 max-w-6xl">
        
        <!-- Header -->
        <header class="text-center mb-4">
            <h1 class="text-3xl font-bold text-white">🤖 ESP32 Robot Dashboard</h1>
            <p id="connection-status" class="text-lg text-red-500 font-semibold">🔴 Disconnected</p>
        </header>

        <!-- Main Grid Layout -->
        <div class="grid grid-cols-1 lg:grid-cols-3 gap-6">

            <!-- Column 1: Controls -->
            <div class="lg:col-span-1 space-y-6">
                
                <!-- Emergency Stop -->
                <div class="bg-gray-800 p-6 rounded-lg shadow-lg">
                    <button id="btn-emergency" class="btn bg-red-600 hover:bg-red-700 text-white p-4 rounded-lg text-xl font-bold w-full flex items-center justify-center space-x-2">
                        <span>EMERGENCY STOP</span>
                    </button>
                </div>

            </div>

            <!-- Column 2: Telemetry & Sensors -->
            <div class="lg:col-span-2 space-y-6">

                <!-- Telemetry Cards -->
                <div class="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4">
                    <!-- State -->
                    <div class="bg-gray-800 p-4 rounded-lg shadow-lg text-center">
                        <div class="text-sm font-medium text-gray-400 uppercase">State</div>
                        <div id="telemetry-state" class="text-2xl font-bold text-white p-2 mt-2 rounded-md bg-gray-700">IDLE</div>
                    </div>
                    <!-- Obstacle -->
                    <div class="bg-gray-800 p-4 rounded-lg shadow-lg text-center">
                        <div class="text-sm font-medium text-gray-400 uppercase">Obstacle</div>
                        <div id="telemetry-obstacle" class="text-2xl font-bold text-green-400 mt-2">✅ CLEAR</div>
                    </div>
                    <!-- Speed -->
                    <div class="bg-gray-800 p-4 rounded-lg shadow-lg text-center">
                        <div class="text-sm font-medium text-gray-400 uppercase">Speed</div>
                        <div id="telemetry-speed" class="text-3xl font-bold text-white mt-2">0.00</div>
                        <div class="text-xs text-gray-400">m/s</div>
                    </div>
                    <!-- Battery -->
                    <div class="bg-gray-800 p-4 rounded-lg shadow-lg text-center">
                        <div class="text-sm font-medium text-gray-400 uppercase">Battery</div>
                        <div id="telemetry-battery" class="text-3xl font-bold text-white mt-2">0.00</div>
                        <div class="text-xs text-gray-400">Volts</div>
                    </div>
                </div>

                <!-- Sensor Array -->
                <div class="bg-gray-800 p-6 rounded-lg shadow-lg">
                    <h2 class="text-xl font-semibold mb-4 text-blue-300">Sensor Array</h2>
                    <div class="bg-gray-900 border border-gray-700 rounded-lg p-4 h-64 flex items-end justify-center space-x-2" id="sensor-array-container">
                        <!-- Sensor bars will be injected here by JavaScript -->
                    </div>
                    <p class="text-center text-sm text-gray-400 mt-2">
                        Bright bars (high) = Black line detected (low sensor value)
                    </p>
                </div>
            </div>

        </div> <!-- /Main Grid -->
    </div> <!-- /Container -->

    <script>
        class RobotDashboard {
            constructor() {
                this.ws = null;
                this.isConnected = false;
                
                // Map state strings to Tailwind CSS classes for color-coding
                this.stateClasses = {
                    'INIT': 'bg-gray-600',
                    'IDLE': 'bg-blue-600',
                    'LINE_FOLLOW': 'bg-green-600',
                    'AVOID_PREPARE': 'bg-yellow-500 text-gray-900',
                    'AVOID_PATH': 'bg-yellow-600 text-gray-900',
                    'MERGE_SEARCH': 'bg-purple-500',
                    'TURN_LEFT_PREPARE': 'bg-indigo-500',
                    'SLOW_DOWN': 'bg-teal-500',
                    'STOP': 'bg-red-600',
                    'LOST_LINE': 'bg-orange-500',
                    'default': 'bg-gray-700'
                };
                
                this.init();
            }

            init() {
                this.setupWebSocket(); // Use actual WebSocket connection on ESP32
                this.bindEventListeners();
                this.renderSensorArray([0, 0, 0, 0, 0, 0, 0]); // Initial render
                
                // Connection status will be updated by WebSocket events
                this.setConnectionStatus(false); // Start as disconnected
            }

            setupWebSocket() {
                // This is the original function, which would be used on the ESP32
                const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
                const wsUrl = `${protocol}//${window.location.hostname}/ws`;
                
                try {
                    this.ws = new WebSocket(wsUrl);

                    this.ws.onopen = () => {
                        console.log('Connected to WebSocket');
                        this.setConnectionStatus(true);
                    };

                    this.ws.onmessage = (event) => {
                        try {
                            const data = JSON.parse(event.data);
                            this.updateUI(data);
                        } catch (e) {
                            console.error('Error parsing WebSocket message:', e);
                        }
                    };

                    this.ws.onclose = () => {
                        console.log('WebSocket connection closed');
                        this.setConnectionStatus(false);
                        // Attempt to reconnect after 3 seconds
                        setTimeout(() => this.setupWebSocket(), 3000);
                    };

                    this.ws.onerror = (error) => {
                        console.error('WebSocket error:', error);
                        this.setConnectionStatus(false);
                    };
                } catch (e) {
                    console.error('Failed to create WebSocket connection:', e);
                    this.setConnectionStatus(false);
                }
            }

            bindEventListeners() {
                // Emergency Stop
                document.getElementById('btn-emergency').addEventListener('click', () => this.sendCommand('stop'));
            }

            setConnectionStatus(connected) {
                this.isConnected = connected;
                const statusEl = document.getElementById('connection-status');
                if (connected) {
                    statusEl.textContent = '🟢 Connected to Robot';
                    statusEl.className = 'text-lg text-green-400 font-semibold';
                } else {
                    // Show a special message for preview mode
                    statusEl.textContent = '🔴 Disconnected (Preview Mode)';
                    statusEl.className = 'text-lg text-red-400 font-semibold';
                }
            }

            updateUI(data) {
                if (data.speed != null) {
                    document.getElementById('telemetry-speed').textContent = data.speed.toFixed(2);
                }
                if (data.battery != null) {
                    document.getElementById('telemetry-battery').textContent = data.battery.toFixed(2);
                }
                if (data.state) {
                    const stateEl = document.getElementById('telemetry-state');
                    stateEl.textContent = data.state;
                    // Apply color-coding based on state
                    const stateClass = this.stateClasses[data.state] || this.stateClasses['default'];
                    stateEl.className = `text-xl font-bold text-white p-2 mt-2 rounded-md ${stateClass}`;
                }
                if (data.obstacle != null) {
                    const obstacleEl = document.getElementById('telemetry-obstacle');
                    if (data.obstacle) {
                        obstacleEl.textContent = '⚠️ DETECTED';
                        obstacleEl.className = 'text-2xl font-bold text-red-400 mt-2';
                    } else {
                        obstacleEl.textContent = '✅ CLEAR';
                        obstacleEl.className = 'text-2xl font-bold text-green-400 mt-2';
                    }
                }
                if (data.sensors && Array.isArray(data.sensors)) {
                    this.renderSensorArray(data.sensors);
                }
            }

            renderSensorArray(sensors) {
                const sensorContainer = document.getElementById('sensor-array-container');
                sensorContainer.innerHTML = ''; // Clear previous bars
                
                // Assuming sensor values are 0-1023, where < 500 is "black"
                const SENSOR_MAX = 1023;
                const LINE_THRESHOLD = 500; // Adjust this threshold to match your robot
                
                sensors.forEach((value, index) => {
                    // Invert the value: low value (black) = high bar
                    const heightPercent = Math.min(100, (SENSOR_MAX - value) / SENSOR_MAX * 100);
                    const isOverLine = value < LINE_THRESHOLD;

                    const bar = document.createElement('div');
                    bar.className = 'sensor-bar w-full rounded-t-md relative';
                    
                    // Set height
                    bar.style.height = `${heightPercent}%`;
                    
                    // Set color based on whether it detects the line
                    bar.style.backgroundColor = isOverLine ? '#34d399' : '#374151'; // green-400 or gray-700
                    
                    // Add a label inside the bar to show the raw value
                    const valueLabel = document.createElement('span');
                    valueLabel.className = 'absolute bottom-1 left-0 right-0 text-center text-xs text-white/70';
                    valueLabel.textContent = value;
                    
                    bar.appendChild(valueLabel);
                    sensorContainer.appendChild(bar);
                });
            }

            sendCommand(cmd) {
                // Mock send command for preview
                console.log('Mock Send command:', cmd);
                // Simulate state changes on button press
                if (cmd === 'start') {
                    this.updateUI({ state: 'LINE_FOLLOW' });
                }
                if (cmd === 'stop') {
                    this.updateUI({ state: 'IDLE', speed: 0.0 });
                }
                if (cmd === 'down') {
                    // Assuming "down" means stop or reverse, just stop for simulation
                    this.updateUI({ state: 'IDLE', speed: 0.0 });
                    console.log('Mock: Robot moving backward or stopping');
                }
            }

            sendCommand(cmd) {
                // Send command to ESP32 via WebSocket
                if (this.ws && this.ws.readyState === WebSocket.OPEN) {
                    const msg = { command: cmd };
                    this.ws.send(JSON.stringify(msg));
                    console.log('Sent command:', cmd);
                } else {
                    console.error('WebSocket is not connected');
                    this.setConnectionStatus(false);
                }
            }
        }

        // Initialize the dashboard when the page loads
        document.addEventListener('DOMContentLoaded', () => {
            new RobotDashboard();
        });
    </script>
</body>
</html>
)rawliteral";



// ===================== SEND TELEMETRY =====================
void sendTelemetry() {
  // Create JSON telemetry string
  String json = "{";
  json += "\"sensors\":[";
  
  // Add sensor values
  for (int i = 0; i < NUM_IR; i++) {
    json += lineSensor[i];
    if (i < NUM_IR - 1) json += ",";
  }
  json += "],";
  json += "\"speed\":" + String(currentSpeed) + ",";
  json += "\"battery\":" + String(8.4) + ",";  // Simulated battery voltage
  
  // Determine state string
  String stateStr = "IDLE";
  switch(state) {
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
  json += "\"obstacle\":" + String(detectObstacle() ? "true" : "false");
  json += "}";
  
  // Send to all WebSocket clients
  ws.textAll(json);
}

// ===================== WIFI SETUP =====================
int8_t initNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
  uint32_t timeout = 20000; // 20 second timeout
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

// ===================== WEBSOCKET SETUP =====================
void setupWebSocket() {
  ws.onEvent(webSocketEvent);
  server.addHandler(&ws);
  
  // Serve the main page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    AsyncWebServerResponse *response = request->beginResponse(200, "text/html", index_html);
    request->send(response);
  });
}