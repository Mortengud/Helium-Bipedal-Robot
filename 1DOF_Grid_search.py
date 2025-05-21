import asyncio
import xml.etree.ElementTree as ET
import qtm_rt 
import numpy as np
from scipy.interpolate import make_interp_spline
import time
import requests
import csv
import matplotlib.pyplot as plt
from datetime import datetime
import pandas as pd

# Configuration for Flask-server
RASPBERRY_PI_IP = "192.168.50.177"
SET_PARAMS_URL = f"http://{RASPBERRY_PI_IP}:5000/set_params"
START_URL = f"http://{RASPBERRY_PI_IP}:5000/start"
STOP_URL = f"http://{RASPBERRY_PI_IP}:5000/stop"

# MoCap
async def connect_mocap():
    connection = await qtm_rt.connect("192.168.50.50")
    if connection is None:
        print("Kunne ikke koble til motion capture-systemet")
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
            print(f"Kroppen '{wanted_body}' ikke funnet i MoCap-data!")
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
            print(f"Venter på gyldig MoCap-data... (Forsøk {attempt})")
            await connection.stream_frames(components=["6d"], on_packet=on_packet)
            await asyncio.sleep(1)
            await connection.stream_frames_stop()

        return position
    except Exception as e:
        print(f"Feil i measure_position: {e}")
        return None

async def track_distance(connection, wanted_body="mortenrobot", params=None):
    start_position = None
    while start_position is None:
        start_position = await measure_position(connection, wanted_body)

    requests.post(SET_PARAMS_URL, json=params)
    requests.post(START_URL)
    await asyncio.sleep(20)
    requests.post(STOP_URL)

    end_position = None
    while end_position is None:
        end_position = await measure_position(connection, wanted_body)

    distance = abs(end_position[0] - start_position[0]) / 1000
    print(f"Roboten beveget seg {distance:.2f} meter.")
    return distance

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
    plt.plot(X_, Y_, label="Optimalisert distanse", linewidth=2, color="blue")
    plt.scatter(iterations, distances, color="red", marker="o", label="Målepunkter")
    plt.xlabel("Iterasjon")
    plt.ylabel("Gangdistanse (m)")
    plt.title("Grid Search av kneledd")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.savefig(log_file.replace(".csv", ".png"))
    plt.show() 

# Read params from CSV
def load_knee_combinations_from_csv(path="grid_search.csv"):
    df = pd.read_csv(path)
    combos = df[["knee1_min", "knee1_max", "knee2_min", "knee2_max"]].values.tolist()
    return combos

# Optimize
async def optimize_gait():
    combos = load_knee_combinations_from_csv()[75:]
    csv_data = []
    distances = []

    log_file = f"GridSearch_{datetime.now().strftime('%d-%m-%Y %H-%M-%S')}.csv"

    connection = await connect_mocap()
    if connection is None:
        return

    for idx, (knee1_min, knee1_max, knee2_min, knee2_max) in enumerate(combos):
        print(f"\nIterasjon {idx + 1}/{len(combos)} - Knee1: ({knee1_min}, {knee1_max}), Knee2: ({knee2_min}, {knee2_max})")

        params = {
            "knee1_min": knee1_min, "knee1_max": knee1_max,
            "knee2_min": knee2_min, "knee2_max": knee2_max,
            "hip1_min": 340, "hip1_max": 340,
            "hip2_min": 340, "hip2_max": 340,
            "knee1_phase": 0.0, "knee2_phase": 0.5,
            "hip1_phase": 0.0, "hip2_phase": 0.5,
            "speed": 0.0015
        }

        distance = await track_distance(connection, "mortenrobot", params)

        csv_data.append([
            idx + 1,
            params["knee1_min"], params["knee1_max"],
            params["knee2_min"], params["knee2_max"],
            params["speed"],
            distance
        ])

        distances.append(distance)

    # Save to CSV
    with open(log_file, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["Iteration", "Knee1 Min", "Knee1 Max", "Knee2 Min", "Knee2 Max", "Speed", "Distance"])
        writer.writerows(csv_data)

    print(f"\n✅ Grid Search fullført!")
    plot_results(distances, log_file)

# Call main
if __name__ == "__main__":
    asyncio.run(optimize_gait())
