import serial
import time
import sys
import os
from datetime import datetime

def read_esp32_bluetooth(port='COM3', baudrate=115200):
    """
    Read messages from ESP32 via Bluetooth and log to file
    """
    ser = None
    messages = []  # Store all messages
    
    try:
        # Configure serial connection
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        
        print(f"🔗 Connected to {port}")
        print("📡 Waiting for messages from ESP32...")
        print("Press Ctrl+C to exit and save log")
        print("-" * 50)
        
        while True:
            if ser.in_waiting > 0:
                # Read line from ESP32
                message = ser.readline().decode('utf-8').strip()
                if message:
                    timestamp = time.strftime("%H:%M:%S")
                    output = f"[{timestamp}] ESP32: {message}"
                    print(output)
                    messages.append(output)  # Store for logging
            
            # time.sleep(0.1)
            
    except Exception as e:
        print(f"❌ Error: {e}")
    
    except KeyboardInterrupt:
        print("\n👋 Exiting and saving log...")
        
        # Save to file
        if messages:
            # Create directory and file path
            log_dir = os.path.join(".", "test", "log")
            os.makedirs(log_dir, exist_ok=True)
            # filename = os.path.join(log_dir, f"esp32_log_{datetime.now().strftime('%H%M%S')}.txt")
            filename = os.path.join(log_dir, f"esp32_log_curve_trial_99.txt")
            
            with open(filename, 'w') as f:
                f.write("\n".join(messages))
            print(f"💾 Log saved to: {filename} ({len(messages)} messages)")
    
    finally:
        if ser and ser.is_open:
            ser.close()
            print("🔒 Connection closed")

if __name__ == "__main__":
    com_port = 'COM15'
    if len(sys.argv) > 1:
        com_port = sys.argv[1]
    
    read_esp32_bluetooth(com_port)