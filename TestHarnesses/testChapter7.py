"""
Author : Anderson Compalas ( acompala@ucsc.edu )
This file contains a test harness for Beard Chapter 7 code. It generates a 4x4 plot with 16 subplots of the different sensors.
"""

import math
import random
import sys
import matplotlib.pyplot as plt

sys.path.append("..")

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Modeling.WindModel as WM
import ece163.Controls.VehicleTrim as VehicleTrim
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModel
import ece163.Sensors.SensorsModel as SensorsModel

def run_sensor_simulation():
    """Runs a 100-step sensor simulation and plots the results."""
    
    # Initialize vehicle dynamics model
    aero_model = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    sensors_model = SensorsModel.SensorsModel(aero_model)
    
    # Set initial conditions
    state = States.vehicleState()
    state.Va = 25.0  # Airspeed (m/s)
    state.u = 25.0 # Forward Speed
    state.pd = -100.0  # Altitude (positive up)
    state.yaw = math.radians(135.0)  # Convert course to radians
    aero_model.setVehicleState(state)
    
    # Store results
    timesteps = 1000
    time_series = [i * aero_model.vehicleDynamics.delta_t for i in range(timesteps)]
    true_data = {k: [] for k in vars(SensorsModel.SensorsModel().getSensorsTrue()).keys()}
    noisy_data = {k: [] for k in vars(SensorsModel.SensorsModel().getSensorsTrue()).keys()}

    for _ in range(timesteps):
        aero_model.Update(Inputs.controlInputs())
        sensors_model.update()
        true_sensors = sensors_model.getSensorsTrue()
        
        noisy_sensors = sensors_model.getSensorsNoisy()
        
        for key in true_data:
            true_data[key].append(getattr(true_sensors, key, 0))
            noisy_data[key].append(getattr(noisy_sensors, key, 0))
    
    # Define sensor labels
    sensor_labels = {
        "gyro_x": "Gyro X (deg/s)", "gyro_y": "Gyro Y (deg/s)", "gyro_z": "Gyro Z (deg/s)",
        "accel_x": "Accel X (m/s²)", "accel_y": "Accel Y (m/s²)", "accel_z": "Accel Z (m/s²)",
        "mag_x": "Mag X (nT)", "mag_y": "Mag Y (nT)", "mag_z": "Mag Z (nT)",
        "baro": "Barometric Pressure (N/m²)", "pitot": "Pitot Pressure (N/m²)",
        "gps_n": "GPS North (m)", "gps_e": "GPS East (m)", "gps_alt": "GPS Altitude (m)",
        "gps_sog": "GPS Speed Over Ground (m/s)", "gps_cog": "GPS Course Over Ground (rad)"
    }
    
    # Plot results
    fig, axes = plt.subplots(4, 4, figsize=(16, 12))
    fig.suptitle("True vs Noisy Sensor Data Over Time", fontsize=16)
    
    for idx, (key, label) in enumerate(sensor_labels.items()):
        ax = axes[idx // 4, idx % 4]
        ax.plot(time_series, true_data[key], label="True", linestyle="-", linewidth=1)
        ax.plot(time_series, noisy_data[key], label="Noisy", linestyle="--", linewidth=1)
        ax.set_title(label)
        ax.set_xlabel("Time (s)")
        ax.legend()
        ax.grid()
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()


run_sensor_simulation()
