
#Server and controller 2DOF
import numpy as np
import time
import threading
import csv
import os
from flask import Flask, request, jsonify
from ServoPi import PWM

# Initialize Flask server
app = Flask(__name__)

# PWM class for servos
pwm = PWM(0x6F)  
pwm.set_pwm_freq(50)  

# Servo channels
SERVO_HIP1 = 0   # Right hip
SERVO_KNEE1 = 2  # Right knee
SERVO_HIP2 = 3   # Left hip
SERVO_KNEE2 = 4  # Left knee

# Default servo parameters
servo_params = {
    "hip1_min": 340, "hip1_max": 220, "hip1_phase": 0.0,
    "hip2_min": 340, "hip2_max": 220, "hip2_phase": 0.0,
    "knee1_min": 450, "knee1_max": 320, "knee1_phase": 0.25,
    "knee2_min": 450, "knee2_max": 320, "knee2_phase": 0.25,
    "speed": 0.0015
}

running = False  
params_received = False  
lock = threading.Lock()  

# Path to CSV log file for parameter logging
LOG_FILE_PATH = "/home/morten/Dokumenter/robot_params_log.csv"

def log_parameters(params):
    file_exists = os.path.isfile(LOG_FILE_PATH)
    with open(LOG_FILE_PATH, mode="a", newline="") as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow([
                "Timestamp", "Hip1 Min", "Hip1 Max", "Hip2 Min", "Hip2 Max",
                "Knee1 Min", "Knee1 Max", "Knee2 Min", "Knee2 Max",
                "Hip1 Phase", "Hip2 Phase", "Knee1 Phase", "Knee2 Phase",
                "Speed"
            ])
        writer.writerow([
            time.strftime("%Y-%m-%d %H:%M:%S"),
            params["hip1_min"], params["hip1_max"], params["hip2_min"], params["hip2_max"],
            params["knee1_min"], params["knee1_max"], params["knee2_min"], params["knee2_max"],
            params["hip1_phase"], params["hip2_phase"], params["knee1_phase"], params["knee2_phase"],
            params["speed"]
        ])

# Class for servo control using min-max sine wave
class MinMaxController:
    def __init__(self, servo_channel, min_val, max_val, phase_shift=0.0, invert=False):
        self.channel = servo_channel
        self.min = min_val
        self.max = max_val
        self.amp = max_val - min_val
        self.offset = (max_val + min_val) / 2
        self.phi = phase_shift
        self.invert = invert

    def compute_position(self, t):
        beta = (self.max + self.min) / 2
        alpha = (self.max - self.min) / 2
        if self.invert:
            return int(alpha + -np.sin(2 * np.pi * (t + self.phi)) * alpha + beta)
        else:
            return int(alpha + np.sin(2 * np.pi * (t + self.phi)) * alpha + beta)

    def set_servo_position(self):
        pos = self.compute_position(t)
        pwm.set_pwm(self.channel, 0, pos)

# Set all servos to idle (min) positions
def set_idle_position():
    pwm.set_pwm(SERVO_HIP1, 0, 340)
    pwm.set_pwm(SERVO_KNEE1, 0, 450)
    pwm.set_pwm(SERVO_HIP2, 0, 300)
    pwm.set_pwm(SERVO_KNEE2, 0, 450)
    print("Servos set to idle position (min values).")

@app.route("/set_params", methods=["POST"])
def set_params():
    global servo_params, params_received, running
    data = request.get_json()
    if not data:
        return jsonify({"error": "No data received"}), 400

    running = False
    set_idle_position()
    time.sleep(1)
    with lock:
        for key in servo_params.keys():
            if key in data and data[key] is not None:
                servo_params[key] = data[key]
    params_received = True
    log_parameters(servo_params)
    return jsonify({"Status": "OK", "Message": "Parameters updated"}), 200

@app.route("/start", methods=["POST"])
def start_robot():
    global running, params_received
    if not params_received:
        return jsonify({"Status": "Error", "Message": "Parameters not received yet"}), 400
    print("Starting robot with updated parameters.")
    running = True
    return jsonify({"Status": "OK", "Message": "Robot started"}), 200

@app.route("/stop", methods=["POST"])
def stop_robot():
    global running
    running = False
    set_idle_position()
    print("Robot stopped.")
    return jsonify({"Status": "OK", "Message": "Robot stopped, ready for new parameters"}), 200

@app.route("/", methods=["GET"])
def status():
    global running, params_received
    return jsonify({
        "Status": "OK",
        "Running": running,
        "Parameters Received": params_received
    }), 200

# Main robot loop running in a background thread
def robot_loop():
    global running
    t = 0.0
    set_idle_position()
    time.sleep(2)
    while True:
        if not running:
            time.sleep(0.1)
            continue
        with lock:
            hip1_controller = MinMaxController(SERVO_HIP1, servo_params["hip1_min"], servo_params["hip1_max"], servo_params["hip1_phase"])
            knee1_controller = MinMaxController(SERVO_KNEE1, servo_params["knee1_min"], servo_params["knee1_max"], servo_params["knee1_phase"])
            hip2_controller = MinMaxController(SERVO_HIP2, servo_params["hip2_min"], servo_params["hip2_max"], servo_params["hip2_phase"])
            knee2_controller = MinMaxController(SERVO_KNEE2, servo_params["knee2_min"], servo_params["knee2_max"], servo_params["knee2_phase"])
            hip1_controller.set_servo_position()
            knee1_controller.set_servo_position()
            hip2_controller.set_servo_position()
            knee2_controller.set_servo_position()
            t += servo_params["speed"]
        time.sleep(servo_params["speed"])

robot_thread = threading.Thread(target=robot_loop)
robot_thread.daemon = True
robot_thread.start()

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
    
    
'''
#Server and controller 1DOF
import numpy as np
import time
import threading
import os
import csv
from flask import Flask, request, jsonify
from ServoPi import PWM

#Initialize Flask server
app = Flask(__name__)

# === Initialize PWM for servos ===
pwm = PWM(0x6F)  # Address of PWM driver
pwm.set_pwm_freq(50)  # 50 Hz for servos

# Servo channels
SERVO_KNEE1 = 2  # Right knee
SERVO_KNEE2 = 4  # Left knee

# Default servo parameters
servo_params = {
    "knee1_min": 450, "knee1_max": 320, "knee1_phase": 0.0,
    "knee2_min": 450, "knee2_max": 320, "knee2_phase": 0.0,
    "speed": 0.0015
}

running = False  
params_received = False  
lock = threading.Lock()  

LOG_FILE_PATH = "/home/morten/Dokumenter/robot_params_log_knee_only.csv"

def log_parameters(params):
    file_exists = os.path.isfile(LOG_FILE_PATH)
    with open(LOG_FILE_PATH, mode="a", newline="") as file:
        writer = csv.writer(file)
        if not file_exists:
            writer.writerow(["Timestamp", "Knee1 Min", "Knee1 Max", "Knee2 Min", "Knee2 Max",
                             "Knee1 Phase", "Knee2 Phase", "Speed"])
        writer.writerow([time.strftime("%Y-%m-%d %H:%M:%S"),
                         params["knee1_min"], params["knee1_max"],
                         params["knee2_min"], params["knee2_max"],
                         params["knee1_phase"], params["knee2_phase"],
                         params["speed"]])

# MinMaxController class for knee movement
class MinMaxController:
    def __init__(self, servo_channel, min_val, max_val, phase_shift=0.0, invert=False):
        self.channel = servo_channel
        self.min = min_val
        self.max = max_val
        self.amp = max_val - min_val
        self.offset = (max_val + min_val) / 2
        self.phi = phase_shift
        self.invert = invert

    def compute_position(self, t):
        beta = (self.max + self.min) / 2
        alpha = (self.max - self.min) / 2
        if self.invert:
            return int(alpha + -np.sin(2 * np.pi * (t + self.phi)) * alpha + beta)
        else:
            return int(alpha + np.sin(2 * np.pi * (t + self.phi)) * alpha + beta)

    def set_servo_position(self, t):
        pos = self.compute_position(t)
        pwm.set_pwm(self.channel, 0, pos)

def set_idle_position():
    pwm.set_pwm(SERVO_KNEE1, 0, servo_params["knee1_min"])
    pwm.set_pwm(SERVO_KNEE2, 0, servo_params["knee2_min"])
    print("Knees set to idle position (min value).")

@app.route("/set_params", methods=["POST"])
def set_params():
    global servo_params, params_received, running
    data = request.get_json()
    if not data:
        return jsonify({"error": "No data received"}), 400

    running = False
    set_idle_position()
    time.sleep(1)
    with lock:
        for key in servo_params.keys():
            if key in data and data[key] is not None:
                servo_params[key] = data[key]
    params_received = True
    log_parameters(servo_params)
    return jsonify({"Status": "OK", "Message": "Parameters updated"}), 200

@app.route("/start", methods=["POST"])
def start_robot():
    global running, params_received
    if not params_received:
        print("Cannot start, parameters not received yet!")
        return jsonify({"Status": "Error", "Message": "Parameters not received yet"}), 400
    print("Starting robot with knee-only controller!")
    running = True
    return jsonify({"Status": "OK", "Message": "Robot started"}), 200

@app.route("/stop", methods=["POST"])
def stop_robot():
    global running
    running = False
    set_idle_position()
    print("Robot stopped.")
    return jsonify({"Status": "OK", "Message": "Robot stopped"}), 200

@app.route("/", methods=["GET"])
def status():
    global running, params_received
    return jsonify({
        "Status": "OK",
        "Running": running,
        "Parameters Received": params_received
    }), 200

def robot_loop():
    global running
    t = 0.0
    set_idle_position()
    time.sleep(2)
    while True:
        if not running:
            time.sleep(0.1)
            continue
        with lock:
            knee1 = MinMaxController(SERVO_KNEE1, servo_params["knee1_min"], servo_params["knee1_max"], servo_params["knee1_phase"])
            knee2 = MinMaxController(SERVO_KNEE2, servo_params["knee2_min"], servo_params["knee2_max"], servo_params["knee2_phase"])
            knee1.set_servo_position(t)
            knee2.set_servo_position(t)
            t += servo_params["speed"]
        time.sleep(servo_params["speed"])

robot_thread = threading.Thread(target=robot_loop)
robot_thread.daemon = True
robot_thread.start()

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
'''
