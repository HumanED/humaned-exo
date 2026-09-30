# HUMANED LEADER ARM V3 MUJOCO SIMULATION CONTROLLED BY HARDWARE ENCODERS

# --- IMPORTS --------------------------------------------------------------------------------------
import serial
import time
import mujoco
import mujoco.viewer
import threading
import numpy as np


# --- LOAD MODEL AS GLOBAL -------------------------------------------------------------------------
# path must be relative to location that script is being run from
model = mujoco.MjModel.from_xml_path("../leader-arm-v3_mujoco/scene.xml")
data = mujoco.MjData(model)

# --- SERIAL ---------------------------------------------------------------------------------------
SERIAL_PORT = 'COM8' 
BAUD_RATE = 115200

encoder_values = [0.0, 0.0, 0.0, 0.0]
data_lock = threading.Lock()
running = True

def serial_reader_thread():
    """Background thread that continuously reads data from the ESP32."""
    global encoder_values, running
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2) # let connection settle
        
        while running:
            if ser.in_waiting > 0:
                try:
                    # parse serial println
                    line = ser.readline().decode('utf-8', errors='ignore').rstrip()
                    values = line.split(",")
                    for i in range(0, len(values)):
                        values[i] = float(values[i])
                    
                    # safely update global variable using a lock
                    with data_lock:
                        encoder_values = values
                        
                except ValueError:
                    continue
                    
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
        print("Serial thread stopped.")


thread = threading.Thread(target=serial_reader_thread, daemon=True)
thread.start()


# --- MAIN -----------------------------------------------------------------------------------------
if __name__ == "__main__":
    # --- VISUALISER ----
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # start timer
            step_start = time.time()

            with data_lock:
                enc_values = encoder_values

            data.qpos[model.joint("shoulder-1").qposadr[0]] = -enc_values[0]
            data.qpos[model.joint("shoulder-2").qposadr[0]] = -enc_values[1]
            data.qpos[model.joint("shoulder-3").qposadr[0]] = np.pi - enc_values[2]
            data.qpos[model.joint("elbow").qposadr[0]] = np.pi - enc_values[3]

            # step physics forward & refresh viewer each step
            mujoco.mj_step(model, data)
            viewer.sync()

            # real-time sim
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

        running = False
        thread.join(timeout=1)