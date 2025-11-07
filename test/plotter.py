import serial
import re
import time
from collections import deque
from dash import Dash, dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objs as go
import threading

# ===================== CONFIG =====================
PORT = "COM10"
BAUDRATE = 115200
SAMPLE_TIME = 0.01
BUFFER_SIZE = 500
NUM_IR = 7

# Data buffers
time_buf = deque(maxlen=BUFFER_SIZE)
state_buf = deque(maxlen=BUFFER_SIZE)
irs_buf = [deque(maxlen=BUFFER_SIZE) for _ in range(NUM_IR)]
ultra_buf = deque(maxlen=BUFFER_SIZE)
servo_buf = deque(maxlen=BUFFER_SIZE)
dc_buf = deque(maxlen=BUFFER_SIZE)

pattern = re.compile(r"(\d+), (\d+), \[([^\]]+)\], ([\d\.]+), (\d+), (-?\d+)")

# ===================== SERIAL THREAD =====================
def serial_reader():
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    time.sleep(2)
    print(f"Serial {PORT} opened at {BAUDRATE} baud")

    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
            match = pattern.match(line)
            if not match:
                continue

            t = float(match.group(1))
            state = float(match.group(2))  # Capture the state value
            irs = [float(x.strip()) for x in match.group(3).split(",")]
            ultrasonic = float(match.group(4))
            servo = float(match.group(5))
            dc = float(match.group(6))

            time_buf.append(t)
            for i in range(NUM_IR):
                irs_buf[i].append(irs[i])
            ultra_buf.append(ultrasonic)
            servo_buf.append(servo)
            dc_buf.append(dc)

        except Exception as e:
            print("Serial read error:", e)
            time.sleep(0.1)

# ===================== DASH APP =====================
app = Dash(__name__)
app.layout = html.Div([
    html.H2("Real-time Sensor Data"),
    dcc.Checklist(
        id='channel-select',
        options=[
            {'label': 'IR sensors', 'value': 'ir'},
            {'label': 'Ultrasonic', 'value': 'ultra'},
            {'label': 'Servo', 'value': 'servo'},
            {'label': 'DC Motor', 'value': 'dc'}
        ],
        value=['ir', 'ultra', 'servo', 'dc'],
        inline=True
    ),
    dcc.Graph(id='live-graph', style={'height': '75vh'}),
    dcc.Interval(id='interval', interval=100, n_intervals=0)  # update every 100 ms
])

@app.callback(Output('live-graph', 'figure'),
              Input('interval', 'n_intervals'),
              Input('channel-select', 'value'))
def update_graph(_, selected):
    if not time_buf:
        return go.Figure()

    t = list(time_buf)
    fig = go.Figure()

    if 'ir' in selected:
        for i in range(NUM_IR):
            fig.add_trace(go.Scatter(
                x=t, y=list(irs_buf[i]),
                mode='lines', name=f'IR{i}'
            ))
    if 'ultra' in selected:
        fig.add_trace(go.Scatter(x=t, y=list(ultra_buf), mode='lines', name='Ultrasonic'))
    if 'servo' in selected:
        fig.add_trace(go.Scatter(x=t, y=list(servo_buf), mode='lines', name='Servo'))
    if 'dc' in selected:
        fig.add_trace(go.Scatter(x=t, y=list(dc_buf), mode='lines', name='DC Motor'))

    fig.update_layout(
        xaxis_title="Time",
        yaxis_title="Value",
        title="Real-time Sensor Data",
        margin=dict(l=40, r=20, t=40, b=40),
        template="plotly_dark"
    )
    return fig

# ===================== MAIN =====================
if __name__ == "__main__":
    threading.Thread(target=serial_reader, daemon=True).start()
    app.run(debug=False, use_reloader=False)
