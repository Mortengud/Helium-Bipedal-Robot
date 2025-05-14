import asyncio
import xml.etree.ElementTree as ET
import qtm_rt 
import numpy as np
from scipy.interpolate import make_interp_spline
import time
import random
import requests
import csv
import matplotlib.pyplot as plt
from datetime import datetime

# === Configuration for Raspberry Pi Flask server ===
RASPBERRY_PI_IP = "192.168.50.177"
SET_PARAMS_URL = f"http://{RASPBERRY_PI_IP}:5000/set_params"
START_URL = f"http://{RASPBERRY_PI_IP}:5000/start"
STOP_URL = f"http://{RASPBERRY_PI_IP}:5000/stop"
STATUS_URL = f"http://{RASPBERRY_PI_IP}:5000/"

distances = []

# === Servo limits ===
SA_BOUNDS = {
    "hip1_min": 340, "hip1_max": 220,
    "hip2_min": 340, "hip2_max": 220,
    "knee1_min": 450, "knee1_max": 320,
    "knee2_min": 450, "knee2_max": 320,
    "speed": (0.0015, 0.0015),
    "hip1_phase": (0.0, 0.0), "hip2_phase": (0.0, 0.5),
    "knee1_phase": (0.0, 0.0), "knee2_phase": (0.0, 0.5)
}

# === Initial values ===
best_params = {
    "hip1_min": 340, "hip1_max": 220,
    "hip2_min": 340, "hip2_max": 220,
    "knee1_min": 450, "knee1_max": 320,
    "knee2_min": 450, "knee2_max": 320,
    "speed": 0.0015,
    "hip1_phase": 0.0, "hip2_phase": 0.5,
    "knee1_phase": 0.0, "knee2_phase": 0.5
}

# === Motion Capture connection ===
async def connect_mocap():
    connection = await qtm_rt.connect("192.168.50.50")
    if connection is None:
        print("Could not connect to motion capture system")
        return None
    return connection

def create_body_index(xml_string):
    xml = ET.fromstring(xml_string)
    body_to_index = {}
    for index, body in enumerate(xml.findall("*/Body/Name")):
        body_to_index[body.text.strip()] = index
    return body_to_index

async def measure_position(connection, wanted_body):
    try:
        xml_string = await connection.get_parameters(parameters=["6d"])
        body_index = create_body_index(xml_string)

        if wanted_body not in body_index:
            print(f"Body '{wanted_body}' not found in MoCap data!")
            return None

        position = None
        attempt = 0

        def on_packet(packet):
            nonlocal position
            _, bodies = packet.get_6d()
            wanted_index = body_index[wanted_body]
            pos, _ = bodies[wanted_index]
            position = np.array([pos.x, pos.y, pos.z])

        while position is None or np.isnan(position).any():
            attempt += 1
            print(f"Waiting for valid MoCap data... (Attempt {attempt})")

            await connection.stream_frames(components=["6d"], on_packet=on_packet)
            await asyncio.sleep(1)
            await connection.stream_frames_stop()

        print(f"Valid MoCap data received after {attempt} attempts: {position}")
        return position

    except Exception as e:
        print(f"Error in measure_position: {e}")
        return None

async def track_distance(connection, wanted_body="mortenrobot", params=None):
    print("Getting start position...")
    start_position = None
    while start_position is None:
        start_position = await measure_position(connection, wanted_body)

    print(f"Sending parameters: {params}")
    requests.post(SET_PARAMS_URL, json=params)

    print("Starting the robot")
    requests.post(START_URL)
    await asyncio.sleep(20)
    print("Stopping the robot")
    requests.post(STOP_URL)

    print("Getting end position...")
    end_position = None
    while end_position is None:
        end_position = await measure_position(connection, wanted_body)

    distance = abs(end_position[0] - start_position[0]) / 1000
    print(f"The robot moved {distance:.2f} meters.")
    return distance

async def optimize_gait():
    global best_params

    csv_data = []
    top_10_data = []
    log_file = f"Test {datetime.now().strftime('%d-%m-%Y %H-%M-%S')}.csv"
    top_10_file = f"Top_10_{datetime.now().strftime('%d-%m-%Y %H-%M-%S')}.csv"

    best_distance = -1
    T = 1.0
    max_iterations = 128
    iteration = 0

    connection = await connect_mocap()
    if connection is None:
        return

    while iteration < max_iterations:
        print(f"Iteration {iteration + 1}/{max_iterations}...")

        new_params = best_params.copy()

        exploration_factor = max(0.2, T)
        hip_scale = random.uniform(0.8 - exploration_factor, 1.2 + exploration_factor)
        knee_scale = random.uniform(0.8 - exploration_factor, 1.2 + exploration_factor)

        new_params["hip1_min"] = min(340, max(220, best_params["hip1_min"] * hip_scale))
        new_params["hip1_max"] = min(340, max(220, best_params["hip1_max"] * hip_scale))
        if new_params["hip1_max"] > new_params["hip1_min"]:
            new_params["hip1_max"] = new_params["hip1_min"]

        new_params["hip2_min"] = min(340, max(220, best_params["hip2_min"] * hip_scale))
        new_params["hip2_max"] = min(340, max(220, best_params["hip2_max"] * hip_scale))
        if new_params["hip2_max"] > new_params["hip2_min"]:
            new_params["hip2_max"] = new_params["hip2_min"]

        new_params["knee1_min"] = min(450, max(320, best_params["knee1_min"] * knee_scale))
        new_params["knee1_max"] = min(450, max(320, best_params["knee1_max"] * knee_scale))
        if new_params["knee1_max"] > new_params["knee1_min"]:
            new_params["knee1_max"] = new_params["knee1_min"]

        new_params["knee2_min"] = min(450, max(320, best_params["knee2_min"] * knee_scale))
        new_params["knee2_max"] = min(450, max(320, best_params["knee2_max"] * knee_scale))
        if new_params["knee2_max"] > new_params["knee2_min"]:
            new_params["knee2_max"] = new_params["knee2_min"]

        distance = await track_distance(connection, "mortenrobot", new_params)

        if distance > best_distance:
            best_distance = distance
            best_params = new_params.copy()

        result_row = [
            iteration + 1,
            new_params["hip1_min"], new_params["hip1_max"],
            new_params["hip2_min"], new_params["hip2_max"],
            new_params["knee1_min"], new_params["knee1_max"],
            new_params["knee2_min"], new_params["knee2_max"],
            new_params["hip1_phase"], new_params["hip2_phase"],
            new_params["knee1_phase"], new_params["knee2_phase"],
            new_params["speed"],
            distance
        ]

        csv_data.append(result_row)
        distances.append(distance)

        top_10_data.append(result_row)
        top_10_data.sort(key=lambda x: x[-1], reverse=True)
        top_10_data = top_10_data[:10]

        T *= 0.98
        iteration += 1

    with open(log_file, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            "Iteration", "Hip1 Min", "Hip1 Max", "Hip2 Min", "Hip2 Max",
            "Knee1 Min", "Knee1 Max", "Knee2 Min", "Knee2 Max",
            "Hip1 Phase", "Hip2 Phase", "Knee1 Phase", "Knee2 Phase",
            "Speed", "Distance"
        ])
        writer.writerows(csv_data)

    with open(top_10_file, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([
            "Iteration", "Hip1 Min", "Hip1 Max", "Hip2 Min", "Hip2 Max",
            "Knee1 Min", "Knee1 Max", "Knee2 Min", "Knee2 Max",
            "Hip1 Phase", "Hip2 Phase", "Knee1 Phase", "Knee2 Phase",
            "Speed", "Distance"
        ])
        writer.writerows(top_10_data)

    print(f"\nBest parameters: {best_params}, distance: {best_distance:.2f} meters.")
    plot_results(distances, log_file)

if __name__ == "__main__":
    start_params = {
        "hip1_min": 340, "hip1_max": 220,
        "hip2_min": 340, "hip2_max": 220,
        "knee1_min": 450, "knee1_max": 320,
        "knee2_min": 450, "knee2_max": 320,
        "speed": 0.0015,
        "hip1_phase": 0.0, "hip2_phase": 0.5,
        "knee1_phase": 0.0, "knee2_phase": 0.5
    }
    asyncio.run(optimize_gait())

def plot_results(distances, log_file):
    iterations = np.arange(1, len(distances) + 1)
    distances = np.array(distances)

    if len(distances) > 3:
        X_Y_Spline = make_interp_spline(iterations, distances, k=3)
        X_ = np.linspace(iterations.min(), iterations.max(), 500)
        Y_ = X_Y_Spline(X_)
    else:
        X_, Y_ = iterations, distances

    plt.figure(figsize=(8, 5))
    plt.plot(X_, Y_, label="Optimized distance", linewidth=2, color="blue")
    plt.scatter(iterations, distances, color="red", marker="o", label="Measurement points")
    plt.xlabel("Iteration")
    plt.ylabel("Walking distance (m)")
    plt.title("Optimization of robot gait")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.savefig(log_file.replace(".csv", ".png"))
    plt.show()