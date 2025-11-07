import asyncio
import json
import random
import websockets
from websockets.server import serve
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import time

connected_clients = set()

states = [
    'INIT', 'IDLE', 'LINE_FOLLOW', 'AVOID_PREPARE', 'AVOID_PATH',
    'MERGE_SEARCH', 'TURN_LEFT_PREPARE', 'SLOW_DOWN', 'STOP', 'LOST_LINE'
]
state_index = 0

# persistent dummy data for smooth variation
front_sensors = [500 + random.randint(-50, 50) for _ in range(7)]
mid_sensors = [500 + random.randint(-50, 50) for _ in range(7)]
speed = 0.5
pwm = 128
servo_angle = 90
ultrasonic = 800

async def handle_websocket(websocket, path):
    print(f"Client connected: {websocket.remote_address}")
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            print(f"Received: {message}")
    except websockets.exceptions.ConnectionClosed:
        print(f"Client disconnected: {websocket.remote_address}")
    finally:
        connected_clients.discard(websocket)

def smooth_update(value, min_val, max_val, step=10):
    """Small random drift around the current value"""
    value += random.randint(-step, step)
    return max(min(value, max_val), min_val)

def generate_telemetry_data():
    global state_index, front_sensors, mid_sensors, speed, pwm, servo_angle, ultrasonic

    # smoothly vary sensor values
    front_sensors = [smooth_update(v, 0, 1023, 20) for v in front_sensors]
    mid_sensors = [smooth_update(v, 0, 1023, 20) for v in mid_sensors]

    # simulate changing speed and control
    speed = round(smooth_update(speed, 0, 1.5, 5) / 100, 2)
    pwm = smooth_update(pwm, 0, 255, 5)
    servo_angle = smooth_update(servo_angle, 60, 120, 3)
    ultrasonic = smooth_update(ultrasonic, 100, 1500, 50)

    # change state occasionally
    if random.random() < 0.05:
        state_index = (state_index + 1) % len(states)

    data = {
        "sensors": front_sensors,
        "midSensors": mid_sensors,
        "speed": speed,
        "state": states[state_index],
        "obstacle": ultrasonic < 300,
        "ultrasonic": ultrasonic,
        "pwm": pwm,
        "servoAngle": servo_angle
    }
    return data

async def broadcast_telemetry():
    while True:
        if connected_clients:
            data = generate_telemetry_data()
            message = json.dumps(data)
            for client in connected_clients.copy():
                try:
                    await client.send(message)
                except websockets.exceptions.ConnectionClosed:
                    connected_clients.discard(client)
        await asyncio.sleep(0.5)  # 2 Hz update rate for smoother display

def run_websocket_server():
    async def start_server():
        server = await serve(handle_websocket, "localhost", 8765)
        print("WebSocket server started on ws://localhost:8765")
        await asyncio.gather(server.wait_closed(), broadcast_telemetry())
    asyncio.run(start_server())

class DashboardHTTPRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        super().end_headers()
    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self.path = '/dashboard.html'
        return super().do_GET()

def run_http_server():
    server_address = ('', 8080)
    httpd = HTTPServer(server_address, DashboardHTTPRequestHandler)
    print("HTTP server started on http://localhost:8080")
    httpd.serve_forever()

if __name__ == "__main__":
    print("Starting dummy telemetry dashboard...")
    http_thread = threading.Thread(target=run_http_server, daemon=True)
    http_thread.start()
    run_websocket_server()
