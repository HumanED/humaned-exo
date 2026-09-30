# --- IMPORTS --------------------------------------------------------------------------------------
import serial
import time

# --- SERIAL ---------------------------------------------------------------------------------------
SERIAL_PORT = 'COM8' 
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to ESP32 on {SERIAL_PORT} at {BAUD_RATE} baud.")
    
    time.sleep(2) 

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').rstrip()
            print(f"ESP32: {line}")

except serial.SerialException as e:
    print(f"Error opening or using serial port: {e}")
except KeyboardInterrupt:
    print("\nExiting script...")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")