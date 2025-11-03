import serial, time

port = "COM10"  # change this to your Mega2560 port
baud = 115200
filename = r".\others\step_response.txt"

with serial.Serial(port, baud, timeout=1) as ser, open(filename, "w") as f:
    print("Logging started... Press Ctrl+C to stop.")
    while True:
        try:
            line = ser.readline().decode("utf-8").strip()
            if line:
                print(line)
                f.write(line + "\n")
        except KeyboardInterrupt:
            print("Logging stopped.")
            break
