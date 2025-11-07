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
    <!-- Include Chart.js for real-time plotting -->
    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <!-- Include chartjs-plugin-zoom for interactive zooming and panning -->
    <script src="https://cdn.jsdelivr.net/npm/chartjs-plugin-zoom@2.0.1"></script>
    <style>
        /* Custom transition for sensor bars */
        .sensor-bar {
            transition: height 0.15s ease, background-color 0.15s ease;
        }
        
        /* Styles for plot containers and tabs */
        .plot-container {
            margin-top: 1rem;
        }
        
        .hidden {
            display: none;
        }
        
        .tab-btn.active {
            border-bottom: 2px solid #3b82f6;
            color: white;
        }
    </style>
</head>
<body class="bg-gray-900 text-gray-100 font-sans">

    <div class="container mx-auto p-8 max-w-[95%]">
        
        <!-- Header -->
        <header class="text-center mb-4">
            <h1 class="text-3xl font-bold text-white">Dashboard</h1>
            <p id="connection-status" class="text-lg text-red-500 font-semibold">🔴 Disconnected</p>
        </header>

        <!-- Emergency Stop -->
        <div class="bg-gray-800 p-6 rounded-lg shadow-lg mb-6">
            <button id="btn-emergency" class="btn bg-red-600 hover:bg-red-700 text-white p-4 rounded-lg text-xl font-bold w-full flex items-center justify-center space-x-2">
                <span>EMERGENCY STOP</span>
            </button>
        </div>

        <!-- Main Grid Layout -->
        <div class="grid grid-cols-1 lg:grid-cols-2 gap-6">

            <!-- Column 1: Real-time Plots -->
            <div class="space-y-6">
                <!-- Real-time Plots -->
                <div class="bg-gray-800 p-6 rounded-lg shadow-lg">
                    <h2 class="text-xl font-semibold mb-4 text-blue-300">Real-time Plots</h2>
                    <!-- Tabs for Ultrasonic, PWM, and Servo -->
                    <div class="flex border-b border-gray-700 mb-4">
                        <button id="ultrasonic-tab" class="tab-btn px-4 py-2 font-medium text-gray-300 border-b-2 border-transparent hover:border-gray-500 hover:text-white">Ultrasonic</button>
                        <button id="servo-tab" class="tab-btn px-4 py-2 font-medium text-gray-300 border-b-2 border-transparent hover:border-gray-500 hover:text-white">Servo Angle</button>
                        <button id="pwm-tab" class="tab-btn px-4 py-2 font-medium text-gray-300 border-b-2 border-transparent hover:border-gray-500 hover:text-white">PWM</button>
                    </div>
                    
                    <!-- Ultrasonic Distance Plot -->
                    <div id="ultrasonic-plot-container" class="plot-container">
                        <div class="bg-gray-900 border border-gray-700 rounded-lg p-4 h-[28rem] relative">
                            <canvas id="ultrasonic-plot" style="height: 100%; width: 100%;"></canvas>
                            <div id="ultrasonic-plot-legend" class="absolute bottom-2 left-2 text-xs text-gray-400">Distance (mm)</div>
                        </div>
                        <div class="flex justify-between items-center mt-2">
                            <div class="text-center text-lg font-semibold text-white" id="ultrasonic-value">0 mm</div>
                            <button onclick="window.dashboard.ultrasonicChart.resetZoom()" 
                                    class="bg-gray-700 hover:bg-gray-600 text-sm px-2 py-1 rounded">
                                Reset Zoom
                            </button>
                        </div>
                    </div>

                    <!-- PWM Plot -->
                    <div id="pwm-plot-container" class="plot-container hidden">
                        <div class="bg-gray-900 border border-gray-700 rounded-lg p-4 h-[28rem] relative">
                            <canvas id="pwm-plot" style="height: 100%; width: 100%;"></canvas>
                            <div id="pwm-plot-legend" class="absolute bottom-2 left-2 text-xs text-gray-400">PWM Value</div>
                        </div>
                        <div class="flex justify-between items-center mt-2">
                            <div class="text-center text-lg font-semibold text-white" id="pwm-value">0</div>
                            <button onclick="window.dashboard.pwmChart.resetZoom()" 
                                    class="bg-gray-700 hover:bg-gray-600 text-sm px-2 py-1 rounded">
                                Reset Zoom
                            </button>
                        </div>
                    </div>
                    
                    <!-- Servo Angle Plot -->
                    <div id="servo-plot-container" class="plot-container hidden">
                        <div class="bg-gray-900 border border-gray-700 rounded-lg p-4 h-[28rem] relative">
                            <canvas id="servo-plot" style="height: 100%; width: 100%;"></canvas>
                            <div id="servo-plot-legend" class="absolute bottom-2 left-2 text-xs text-gray-400">Servo Angle (degrees)</div>
                        </div>
                        <div class="flex justify-between items-center mt-2">
                            <div class="text-center text-lg font-semibold text-white" id="servo-value">90°</div>
                            <button onclick="window.dashboard.servoChart.resetZoom()" 
                                    class="bg-gray-700 hover:bg-gray-600 text-sm px-2 py-1 rounded">
                                Reset Zoom
                            </button>
                        </div>
                    </div>
                </div>
            </div>

            <!-- Column 2: Telemetry & Sensors -->
            <div class="space-y-6">

                <!-- Telemetry Cards -->
                <div class="grid grid-cols-3 gap-2 w-full">
                    <!-- State -->
                    <div class="bg-gray-800 p-4 rounded-lg shadow-lg text-center flex-1">
                        <div class="text-sm font-medium text-gray-400 uppercase">State</div>
                        <div id="telemetry-state"
                            class="text-2xl font-bold text-white p-2 mt-2 rounded-md bg-gray-700 min-w-[120px]">
                            IDLE
                        </div>
                    </div>

                    <!-- Obstacle -->
                    <div class="bg-gray-800 p-4 rounded-lg shadow-lg text-center flex-1">
                        <div class="text-sm font-medium text-gray-400 uppercase">Obstacle</div>
                        <div id="telemetry-obstacle"
                            class="text-2xl font-bold text-green-400 mt-2 p-2 rounded-md bg-gray-700 min-w-[120px]">
                            CLEAR
                        </div>
                    </div>

                    <!-- Speed -->
                    <div class="bg-gray-800 p-4 rounded-lg shadow-lg text-center flex-1">
                        <div class="text-sm font-medium text-gray-400 uppercase">Speed</div>
                        <div id="telemetry-speed"
                            class="text-3xl font-bold text-white mt-2 p-2 rounded-md bg-gray-700 min-w-[120px]">
                            0.00
                        </div>
                    </div>
                </div>

                <!-- Sensor Arrays -->
                <div class="bg-gray-800 p-6 rounded-lg shadow-lg">
                    <h2 class="text-xl font-semibold mb-4 text-blue-300">Sensor Arrays</h2>
                    <div class="mb-6">
                        <h3 class="font-medium text-gray-300 mb-2">Front Sensors</h3>
                        <div class="bg-gray-900 border border-gray-700 rounded-lg p-4 h-[9rem] flex items-end justify-center space-x-2" id="sensor-array-container">
                            <!-- Sensor bars will be injected here by JavaScript -->
                        </div>
                    </div>
                    <div>
                        <h3 class="font-medium text-gray-300 mb-2">Middle Sensors</h3>
                        <div class="bg-gray-900 border border-gray-700 rounded-lg p-4 h-[9rem] flex items-end justify-center space-x-2" id="mid-sensor-array-container">
                            <!-- Middle sensor bars will be injected here by JavaScript -->
                        </div>
                    </div>
                    <p class="text-center text-sm text-gray-400 mt-2">
                        Bright bars (high) = Black line detected (high sensor value)
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
                this.renderSensorArray([0, 0, 0, 0, 0, 0, 0]); // Initial render front sensors
                this.renderMidSensorArray([0, 0, 0, 0, 0, 0, 0]); // Initial render middle sensors
                
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
                if (data.state) {
                    const stateEl = document.getElementById('telemetry-state');
                    stateEl.textContent = data.state;
                    // Apply color-coding based on state
                    const stateClass = this.stateClasses[data.state] || this.stateClasses['default'];
                    stateEl.className = `text-2xl font-bold text-white p-2 mt-2 rounded-lg ${stateClass}`;
                }
                if (data.obstacle != null) {
                    const obstacleEl = document.getElementById('telemetry-obstacle');
                    if (data.obstacle) {
                        obstacleEl.textContent = 'DETECTED';
                        obstacleEl.className = 'mt-2 text-2xl font-bold text-red-400 p-2 rounded-lg bg-gray-700 shadow-inner';
                    } else {
                        obstacleEl.textContent = 'CLEAR';
                        obstacleEl.className = 'mt-2 text-2xl font-bold text-green-400 p-2 rounded-lg bg-gray-700 shadow-inner';
                    }
                }
                if (data.sensors && Array.isArray(data.sensors)) {
                    this.renderSensorArray(data.sensors);
                }
                if (data.midSensors && Array.isArray(data.midSensors)) {
                    this.renderMidSensorArray(data.midSensors);
                }
                if (data.ultrasonic !== undefined) {
                    this.updatePlotData(data.ultrasonic, undefined, undefined);
                }
                if (data.pwm !== undefined) {
                    this.updatePlotData(undefined, data.pwm, undefined);
                }
                if (data.servoAngle !== undefined) {
                    this.updatePlotData(undefined, undefined, data.servoAngle);
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
                    const heightPercent = Math.min(100, value / SENSOR_MAX * 100);
                    const isOverLine = value > LINE_THRESHOLD; // now black is when ADC is high

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

            renderMidSensorArray(sensors) {
                const sensorContainer = document.getElementById('mid-sensor-array-container');
                sensorContainer.innerHTML = ''; // Clear previous bars
                
                // Assuming sensor values are 0-1023, where < 500 is "black"
                const SENSOR_MAX = 1023;
                const LINE_THRESHOLD = 500; // Adjust this threshold to match your robot
                
                sensors.forEach((value, index) => {
                    // Invert the value: low value (black) = high bar
                    const heightPercent = Math.min(100, value / SENSOR_MAX * 100);
                    const isOverLine = value > LINE_THRESHOLD; // now black is when ADC is high

                    const bar = document.createElement('div');
                    bar.className = 'sensor-bar w-full rounded-t-md relative';
                    
                    // Set height
                    bar.style.height = `${heightPercent}%`;
                    
                    // Set color based on whether it detects the line
                    bar.style.backgroundColor = isOverLine ? '#60a5fa' : '#374151'; // blue-400 or gray-700 (different color from front sensors)
                    
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

            // Chart.js plotting functionality for ultrasonic, PWM, and servoAngle data
            initializePlots() {
                // Initialize data arrays to hold the last 100 data points
                this.ultrasonicData = [];
                this.pwmData = [];
                this.servoData = [];
                this.timestampData = [];
                
                // Create Chart.js instances
                this.createUltrasonicChart();
                this.createPwmChart();
                this.createServoChart();
                
                // Add event listeners for tab switching
                document.getElementById('ultrasonic-tab').addEventListener('click', () => {
                    this.switchTab('ultrasonic');
                });
                document.getElementById('pwm-tab').addEventListener('click', () => {
                    this.switchTab('pwm');
                });
                document.getElementById('servo-tab').addEventListener('click', () => {
                    this.switchTab('servo');
                });
                
                // Initially activate ultrasonic tab
                this.switchTab('ultrasonic');
            }
            
            createUltrasonicChart() {
                const ctx = document.getElementById('ultrasonic-plot');
                this.ultrasonicChart = new Chart(ctx, {
                    type: 'line',
                    data: {
                        labels: [],
                        datasets: [{
                            label: 'Ultrasonic Distance (mm)',
                            data: [],
                            borderColor: '#3b82f6',
                            backgroundColor: 'rgba(59, 130, 246, 0.1)',
                            borderWidth: 2,
                            fill: false,
                            tension: 0.1,
                            pointRadius: 0,
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        scales: {
                            x: {
                                display: true,
                                title: {
                                    display: true,
                                    text: 'Time'
                                },
                                grid: {
                                    color: 'rgba(55, 65, 81, 0.5)'
                                }
                            },
                            y: {
                                display: true,
                                title: {
                                    display: true,
                                    text: 'Distance (mm)'
                                },
                                min: 0,
                                max: 4000,
                                grid: {
                                    color: 'rgba(55, 65, 81, 0.5)'
                                }
                            }
                        },
                        plugins: {
                            legend: {
                                labels: {
                                    color: '#d1d5db'
                                }
                            },
                            zoom: {
                                pan: {
                                    enabled: true,
                                    mode: 'xy', // 'x', 'y', or 'xy'
                                    modifierKey: 'ctrl' // Optional: require Ctrl key to pan
                                },
                                zoom: {
                                    wheel: {
                                        enabled: true, // Scroll to zoom
                                    },
                                    pinch: {
                                        enabled: true, // Pinch on touch devices
                                    },
                                    mode: 'xy', // Zoom both axes
                                }
                            }
                        }
                    }
                });
            }
            
            createPwmChart() {
                const ctx = document.getElementById('pwm-plot');
                this.pwmChart = new Chart(ctx, {
                    type: 'line',
                    data: {
                        labels: [],
                        datasets: [{
                            label: 'PWM Value',
                            data: [],
                            borderColor: '#10b981',
                            backgroundColor: 'rgba(16, 185, 129, 0.1)',
                            borderWidth: 2,
                            fill: false,
                            tension: 0.1,
                            pointRadius: 0,
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        scales: {
                            x: {
                                display: true,
                                title: {
                                    display: true,
                                    text: 'Time'
                                },
                                grid: {
                                    color: 'rgba(55, 65, 81, 0.5)'
                                }
                            },
                            y: {
                                display: true,
                                title: {
                                    display: true,
                                    text: 'PWM Value'
                                },
                                min: -255,
                                max: 255,
                                grid: {
                                    color: 'rgba(55, 65, 81, 0.5)'
                                }
                            }
                        },
                        plugins: {
                            legend: {
                                labels: {
                                    color: '#d1d5db'
                                }
                            },
                            zoom: {
                                pan: {
                                    enabled: true,
                                    mode: 'xy', // 'x', 'y', or 'xy'
                                    modifierKey: 'ctrl' // Optional: require Ctrl key to pan
                                },
                                zoom: {
                                    wheel: {
                                        enabled: true, // Scroll to zoom
                                    },
                                    pinch: {
                                        enabled: true, // Pinch on touch devices
                                    },
                                    mode: 'xy', // Zoom both axes
                                }
                            }
                        }
                    }
                });
            }
            
            createServoChart() {
                const ctx = document.getElementById('servo-plot');
                this.servoChart = new Chart(ctx, {
                    type: 'line',
                    data: {
                        labels: [],
                        datasets: [{
                            label: 'Servo Angle (degrees)',
                            data: [],
                            borderColor: '#f97316',
                            backgroundColor: 'rgba(249, 115, 22, 0.1)',
                            borderWidth: 2,
                            fill: false,
                            tension: 0.1,
                            pointRadius: 0,
                        }]
                    },
                    options: {
                        responsive: true,
                        maintainAspectRatio: false,
                        scales: {
                            x: {
                                display: true,
                                title: {
                                    display: true,
                                    text: 'Time'
                                },
                                grid: {
                                    color: 'rgba(55, 65, 81, 0.5)'
                                }
                            },
                            y: {
                                display: true,
                                title: {
                                    display: true,
                                    text: 'Angle (degrees)'
                                },
                                min: 0,
                                max: 180,
                                grid: {
                                    color: 'rgba(55, 65, 81, 0.5)'
                                }
                            }
                        },
                        plugins: {
                            legend: {
                                labels: {
                                    color: '#d1d5db'
                                }
                            },
                            zoom: {
                                pan: {
                                    enabled: true,
                                    mode: 'xy', // 'x', 'y', or 'xy'
                                    modifierKey: 'ctrl' // Optional: require Ctrl key to pan
                                },
                                zoom: {
                                    wheel: {
                                        enabled: true, // Scroll to zoom
                                    },
                                    pinch: {
                                        enabled: true, // Pinch on touch devices
                                    },
                                    mode: 'xy', // Zoom both axes
                                }
                            }
                        }
                    }
                });
            }
            
            updatePlotData(ultrasonicDist, pwmValue, servoAngle) {
                const timestamp = Date.now(); // Current timestamp in milliseconds
                const timeString = new Date(timestamp).toLocaleTimeString();
                
                if (ultrasonicDist !== undefined) {
                    this.ultrasonicData.push(ultrasonicDist);
                    this.timestampData.push(timeString);

                    // Maintain only the last 100 data points
                    if (this.ultrasonicData.length > 200) {
                        this.ultrasonicData.shift();
                        this.timestampData.shift();
                    }

                    // Dynamically calculate Y-axis min/max (with a margin)
                    const minVal = Math.min(...this.ultrasonicData);
                    const maxVal = Math.max(...this.ultrasonicData);
                    const margin = (maxVal - minVal) * 0.2 || 50;  // 20% margin, or default 50mm

                    // Apply the new scale range
                    this.ultrasonicChart.options.scales.y.min = Math.max(0, minVal - margin);
                    this.ultrasonicChart.options.scales.y.max = maxVal + margin;

                    // Update chart data and re-render
                    this.ultrasonicChart.data.labels = [...this.timestampData];
                    this.ultrasonicChart.data.datasets[0].data = [...this.ultrasonicData];
                    this.ultrasonicChart.update('none');

                    // Update the displayed value
                    document.getElementById('ultrasonic-value').textContent = ultrasonicDist.toFixed(0) + ' mm';
                }

                
                if (pwmValue !== undefined) {
                    this.pwmData.push(pwmValue);
                    if (ultrasonicDist === undefined && servoAngle === undefined) {
                        this.timestampData.push(timeString);
                    }
                    
                    // Maintain only the last 100 data points
                    if (this.pwmData.length > 100) {
                        this.pwmData.shift();
                        if (ultrasonicDist === undefined && servoAngle === undefined) {
                            this.timestampData.shift();
                        }
                    }
                    
                    // Update the chart data
                    this.pwmChart.data.labels = [...this.timestampData];
                    this.pwmChart.data.datasets[0].data = [...this.pwmData];
                    this.pwmChart.update('none');
                    
                    // Update the current value display
                    document.getElementById('pwm-value').textContent = pwmValue.toFixed(0);
                }
                
                if (servoAngle !== undefined) {
                    this.servoData.push(servoAngle);
                    if (ultrasonicDist === undefined && pwmValue === undefined) {
                        this.timestampData.push(timeString);
                    }
                    
                    // Maintain only the last 100 data points
                    if (this.servoData.length > 100) {
                        this.servoData.shift();
                        if (ultrasonicDist === undefined && pwmValue === undefined) {
                            this.timestampData.shift();
                        }
                    }
                    
                    // Update the chart data
                    this.servoChart.data.labels = [...this.timestampData];
                    this.servoChart.data.datasets[0].data = [...this.servoData];
                    this.servoChart.update('none');
                    
                    // Update the current value display
                    document.getElementById('servo-value').textContent = servoAngle.toFixed(0) + '°';
                }
            }
            
            switchTab(tabName) {
                // Hide all plot containers
                document.getElementById('ultrasonic-plot-container').classList.add('hidden');
                document.getElementById('pwm-plot-container').classList.add('hidden');
                document.getElementById('servo-plot-container').classList.add('hidden');
                
                // Remove active class from all tabs
                document.getElementById('ultrasonic-tab').classList.remove('active');
                document.getElementById('pwm-tab').classList.remove('active');
                document.getElementById('servo-tab').classList.remove('active');
                
                // Show selected plot container and activate tab
                if (tabName === 'ultrasonic') {
                    document.getElementById('ultrasonic-plot-container').classList.remove('hidden');
                    document.getElementById('ultrasonic-tab').classList.add('active');
                } else if (tabName === 'pwm') {
                    document.getElementById('pwm-plot-container').classList.remove('hidden');
                    document.getElementById('pwm-tab').classList.add('active');
                } else if (tabName === 'servo') {
                    document.getElementById('servo-plot-container').classList.remove('hidden');
                    document.getElementById('servo-tab').classList.add('active');
                }
            }
        }

        // Initialize the dashboard when the page loads
        document.addEventListener('DOMContentLoaded', () => {
            const dashboard = new RobotDashboard();
            // Initialize plots after dashboard is created
            dashboard.initializePlots();
            // Make dashboard accessible globally for reset buttons
            window.dashboard = dashboard;
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
  
  // Add front sensor values
  for (int i = 0; i < NUM_IR; i++) {
    json += lineSensor[i];
    if (i < NUM_IR - 1) json += ",";
  }
  json += "],";
  
  // Add middle sensor values
  json += "\"midSensors\":[";
  for (int i = 0; i < NUM_IR; i++) {
    json += midSensor[i];
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
  // Determine obstacle status based on the global sonarDistance value using the system-configured threshold
  // to avoid calling detectObstacle() which would trigger another sensor reading
  bool isObstacle = (sonarDistance > 0 && sonarDistance < SONAR_TH_OBS);
  json += "\"obstacle\":" + String(isObstacle ? "true" : "false") + ",";
  
  // Add ultrasonic distance data - using the already updated global variable
  // instead of calling getSonarDistance() again to avoid redundant sensor readings
  json += "\"ultrasonic\":" + String(sonarDistance) + ",";
  
  // Add PWM data
  int16_t currentPWM = 0;
  if (getMotor(&currentPWM) == ERROR_SUCCESS) {
    json += "\"pwm\":" + String(currentPWM) + ",";
  } else {
    json += "\"pwm\":0,";
  }
  
  // Add servo angle data
  int8_t currentServoAngle = 0;
  if (getCurrentServoAngle(&currentServoAngle) == ERROR_SUCCESS) {
    json += "\"servoAngle\":" + String(currentServoAngle);
  } else {
    json += "\"servoAngle\":90"; // Default to center position
  }
  
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