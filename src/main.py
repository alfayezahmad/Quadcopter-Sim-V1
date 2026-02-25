import numpy as np
import matplotlib.pyplot as plt
import os
from drone_sim import PID, KalmanFilter
from parser import MissionParser

# --- Physical Constants ---
DT = 0.1
MASS = 1.0
GRAVITY = 9.81

def run():
    # --- Cloud-Native Path Handling ---
    # Allow environment variables to override default paths for Docker volumes
    base_dir = os.path.dirname(os.path.abspath(__file__))
    default_mission = os.path.join(base_dir, "data", "mission.txt")
    default_output = os.path.join(base_dir, "data", "flight_plot.png")
    
    mission_path = os.environ.get("MISSION_PATH", default_mission)
    output_path = os.environ.get("OUTPUT_PATH", default_output)

    # Ensure output directory exists
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    # --- System Initialization ---
    try:
        mission = MissionParser(mission_path)
    except FileNotFoundError as e:
        print(f"[ERROR] {e}")
        print(f"[HINT] Ensure you have a valid mission file at {mission_path}")
        return

    brain = PID(kp=15.0, ki=0.5, kd=10.0, dt=DT)
    kf = KalmanFilter(q=0.1, r=0.5)

    # --- State Variables ---
    true_alt = 0.0
    velocity = 0.0
    history = {"true": [], "est": [], "time": []}

    # --- Simulation Loop ---
    for i in range(500):
        time_step = i * DT

        # 1. Perception & Logic (The "Brain")
        target = mission.get_logic(kf.estimate, time_step)
        if target is None:
            print("[INFO] Mission sequence completed successfully.")
            break

        # 2. Control (The "Actuator")
        # Added gravity feed-forward to assist the PID
        thrust = brain.compute(target, kf.estimate) + (MASS * GRAVITY)

        # 3. Physics (The "World")
        accel = (thrust - (MASS * GRAVITY)) / MASS
        velocity += accel * DT
        true_alt += velocity * DT

        # Ground collision logic
        if true_alt < 0:
            true_alt, velocity = 0.0, 0.0

        # 4. Sensing & Estimation (The "Filter")
        noisy_sensor = true_alt + np.random.normal(0, 0.5)
        kf.update(noisy_sensor)

        # Logging
        history["time"].append(time_step)
        history["true"].append(true_alt)
        history["est"].append(kf.estimate)

    # --- Visualization ---
    plt.figure(figsize=(10, 5))
    plt.plot(history["time"], history["true"], label="Ground Truth (Ideal)", color="black", linewidth=2)
    plt.plot(history["time"], history["est"], label="Kalman Estimate (Filtered)", linestyle="--", color="blue")
    plt.title("Quadcopter-Sim-V1: Altitude Tracking Performance")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Cloud-Native Tweak: Save instead of show
    plt.savefig(output_path)
    print(f"[SUCCESS] Flight telemetry plot saved to: {output_path}")

if __name__ == "__main__":
    run()
