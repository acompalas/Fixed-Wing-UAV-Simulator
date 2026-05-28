"""
Author : Anderson Compalas ( acompala@ucsc.edu )
This file contains a test harness for Beard Chapter 8 code. 
"""

import math
import sys
import matplotlib.pyplot as plt

sys.path.append("..")

import ece163.Utilities.MatrixMath as MM
import ece163.Modeling.VehicleAerodynamicsModel as VDM
import ece163.Sensors.SensorsModel as SensorsModel
import ece163.Containers.States as States
import ece163.Containers.Inputs as Inputs
import ece163.Controls.VehicleEstimator as VehicleEstimator


def plot_state_debug(time_series, true_data, estimated_data, labels, title="State Estimation Debugging"):
    """
    Helper function to plot true vs estimated values for state estimation debugging.
    """
    num_states = len(true_data)
    fig, axes = plt.subplots(num_states, 1, figsize=(12, 3 * num_states))

    if num_states == 1:
        axes = [axes]  # Ensure axes is iterable when there's only one plot.

    for i, (key, label) in enumerate(labels.items()):
        axes[i].plot(time_series, true_data[key], label=f"True {label}", linestyle="--", linewidth=1.5)
        axes[i].plot(time_series, estimated_data[key], label=f"Estimated {label}", linestyle='dotted', linewidth=2)
        axes[i].set_ylabel(label)
        axes[i].legend()
        axes[i].grid()

    plt.suptitle(title, fontsize=14)
    plt.xlabel("Time (s)")
    plt.show()


def debug_estimate_attitude_true():
    """Debug estimateAttitude() and estimateAirspeed() using true sensor values."""

    # Initialize models
    aero_model = VDM.VehicleAerodynamicsModel()
    sensors_model = SensorsModel.SensorsModel(aero_model)  # Sensor model
    estimator = VehicleEstimator.VehicleEstimator(sensorsModel=sensors_model)

    # Set initial conditions
    state = States.vehicleState()
    state.Va = 25.0  # Set true airspeed to 25 m/s
    state.u = 25.0  # Forward Speed
    state.yaw = math.radians(0.0)  # Initial yaw angle
    state.pitch = math.radians(0.0)  # Small pitch
    state.roll = math.radians(0.0)  # Small roll
    aero_model.setVehicleState(state)

    # Store results for plotting
    timesteps = 500
    time_series = [i * aero_model.vehicleDynamics.delta_t for i in range(timesteps)]
    true_data = {"gyro_x": [], "gyro_y": [], "gyro_z": []}
    estimated_data = {"gyro_x": [], "gyro_y": [], "gyro_z": []}

    for _ in range(timesteps):
        # Update the aerodynamics model (true state evolves)
        aero_model.Update(Inputs.controlInputs())

        # Update Sensors Model
        sensors_model.update()  

        # Get true sensor values 
        sensorData = sensors_model.sensorsTrue

        # Estimate attitude
        bias, angular_rates, R_est = estimator.estimateAttitude(sensorData, estimator.estimatedState)

        # Store results
        true_data["gyro_x"].append(sensorData.gyro_x)
        true_data["gyro_y"].append(sensorData.gyro_y)
        true_data["gyro_z"].append(sensorData.gyro_z)

        estimated_data["gyro_x"].append(angular_rates[0][0])
        estimated_data["gyro_y"].append(angular_rates[1][0])
        estimated_data["gyro_z"].append(angular_rates[2][0])

    # Define labels for plotting
    sensor_labels = {
        "gyro_x": "Angular Rate p (rad/s)",
        "gyro_y": "Angular Rate q (rad/s)",
        "gyro_z": "Angular Rate r (rad/s)",
    }

    # Call the helper function to plot results
    plot_state_debug(time_series, true_data, estimated_data, sensor_labels, "Debugging Attitude (True Sensors)")

def debug_estimate_attitude_noisy():
    """Debug estimateAttitude() using noisy sensor values while plotting true values for reference."""

    # Initialize models
    aero_model = VDM.VehicleAerodynamicsModel()
    sensors_model = SensorsModel.SensorsModel(aero_model)  # Sensor model
    estimator = VehicleEstimator.VehicleEstimator(sensorsModel=sensors_model)

    # Set initial conditions
    state = States.vehicleState()
    state.Va = 25.0  # Set true airspeed to 25 m/s
    state.u = 25.0  # Forward Speed
    state.yaw = math.radians(0.0)  # Initial yaw angle
    state.pitch = math.radians(0.0)  # Small pitch
    state.roll = math.radians(0.0)  # Small roll
    aero_model.setVehicleState(state)

    # Store results for plotting
    timesteps = 500
    time_series = [i * aero_model.vehicleDynamics.delta_t for i in range(timesteps)]
    
    # Data containers
    true_data = {"gyro_x": [], "gyro_y": [], "gyro_z": []}
    noisy_data = {"gyro_x": [], "gyro_y": [], "gyro_z": []}
    estimated_data = {"gyro_x": [], "gyro_y": [], "gyro_z": []}

    for _ in range(timesteps):
        # Update the aerodynamics model (true state evolves)
        aero_model.Update(Inputs.controlInputs())

        # Update Sensors Model (true and noisy values updated)
        sensors_model.update()  

        # Get true and noisy sensor values 
        trueSensorData = sensors_model.getSensorsTrue()
        noisySensorData = sensors_model.getSensorsNoisy()

        # Estimate attitude using NOISY sensor data
        bias, angular_rates, R_est = estimator.estimateAttitude(noisySensorData, estimator.estimatedState)

        # Store true values
        true_data["gyro_x"].append(trueSensorData.gyro_x)
        true_data["gyro_y"].append(trueSensorData.gyro_y)
        true_data["gyro_z"].append(trueSensorData.gyro_z)

        # Store noisy values
        noisy_data["gyro_x"].append(noisySensorData.gyro_x)
        noisy_data["gyro_y"].append(noisySensorData.gyro_y)
        noisy_data["gyro_z"].append(noisySensorData.gyro_z)

        # Store estimated values
        estimated_data["gyro_x"].append(angular_rates[0][0])
        estimated_data["gyro_y"].append(angular_rates[1][0])
        estimated_data["gyro_z"].append(angular_rates[2][0])

    # Define labels for plotting
    sensor_labels = {
        "gyro_x": "Angular Rate p (rad/s)",
        "gyro_y": "Angular Rate q (rad/s)",
        "gyro_z": "Angular Rate r (rad/s)",
    }

    # Plot true vs noisy vs estimated values
    fig, axes = plt.subplots(len(sensor_labels), 1, figsize=(12, 3 * len(sensor_labels)))

    for i, (key, label) in enumerate(sensor_labels.items()):
        ax = axes[i]
        ax.plot(time_series, true_data[key], label=f"True {label}", linestyle="-", linewidth=2)
        ax.plot(time_series, noisy_data[key], label=f"Noisy {label}", linestyle="--", linewidth=1.2)
        ax.plot(time_series, estimated_data[key], label=f"Estimated {label}", linestyle="dotted", linewidth=2)
        ax.set_ylabel(label)
        ax.legend()
        ax.grid()

    plt.suptitle("Debugging Attitude Estimation (Noisy vs True vs Estimated)", fontsize=14)
    plt.xlabel("Time (s)")
    plt.show()
    
def debug_estimate_attitude_bias():
    """Debug estimateAttitude() using noisy sensor values while tracking bias estimates."""
    
    # Initialize models
    aero_model = VDM.VehicleAerodynamicsModel()
    sensors_model = SensorsModel.SensorsModel(aero_model)  # Sensor model
    estimator = VehicleEstimator.VehicleEstimator(sensorsModel=sensors_model)

    # Set initial conditions
    state = States.vehicleState()
    state.Va = 25.0  # Set true airspeed to 25 m/s
    state.u = 25.0  # Forward Speed
    state.yaw = math.radians(45.0)  # Initial yaw angle
    state.pitch = math.radians(5.0)  # Small pitch
    state.roll = math.radians(10.0)  # Small roll
    aero_model.setVehicleState(state)

    # Store results for plotting
    timesteps = 500
    time_series = [i * aero_model.vehicleDynamics.delta_t for i in range(timesteps)]
    
    # Data containers
    true_data = {"gyro_x": [], "gyro_y": [], "gyro_z": []}
    noisy_data = {"gyro_x": [], "gyro_y": [], "gyro_z": []}
    estimated_data = {"gyro_x": [], "gyro_y": [], "gyro_z": []}
    bias_data = {"bias_p": [], "bias_q": [], "bias_r": []}

    for _ in range(timesteps):
        # Update the aerodynamics model (true state evolves)
        aero_model.Update(Inputs.controlInputs())

        # Update Sensors Model (true and noisy values updated)
        sensors_model.update()  

        # Get true and noisy sensor values 
        trueSensorData = sensors_model.getSensorsTrue()
        noisySensorData = sensors_model.getSensorsNoisy()

        # Estimate attitude using NOISY sensor data
        bias, angular_rates, R_est = estimator.estimateAttitude(noisySensorData, estimator.estimatedState)

        # Store true values
        true_data["gyro_x"].append(trueSensorData.gyro_x)
        true_data["gyro_y"].append(trueSensorData.gyro_y)
        true_data["gyro_z"].append(trueSensorData.gyro_z)

        # Store noisy values
        noisy_data["gyro_x"].append(noisySensorData.gyro_x)
        noisy_data["gyro_y"].append(noisySensorData.gyro_y)
        noisy_data["gyro_z"].append(noisySensorData.gyro_z)

        # Store estimated values
        estimated_data["gyro_x"].append(angular_rates[0][0])
        estimated_data["gyro_y"].append(angular_rates[1][0])
        estimated_data["gyro_z"].append(angular_rates[2][0])

        # Store bias estimates
        bias_data["bias_p"].append(bias[0][0])
        bias_data["bias_q"].append(bias[1][0])
        bias_data["bias_r"].append(bias[2][0])

    # Define labels for plotting
    sensor_labels = {
        "gyro_x": "Angular Rate p (rad/s)",
        "gyro_y": "Angular Rate q (rad/s)",
        "gyro_z": "Angular Rate r (rad/s)",
    }

    bias_labels = {
        "bias_p": "Estimated Bias p",
        "bias_q": "Estimated Bias q",
        "bias_r": "Estimated Bias r",
    }

    # Plot true vs noisy vs estimated values
    fig, axes = plt.subplots(len(sensor_labels), 1, figsize=(12, 3 * len(sensor_labels)))

    for i, (key, label) in enumerate(sensor_labels.items()):
        ax = axes[i]
        ax.plot(time_series, true_data[key], label=f"True {label}", linestyle="-", linewidth=2)
        ax.plot(time_series, noisy_data[key], label=f"Noisy {label}", linestyle="--", linewidth=1.2)
        ax.plot(time_series, estimated_data[key], label=f"Estimated {label}", linestyle="dotted", linewidth=2)
        ax.set_ylabel(label)
        ax.legend()
        ax.grid()

    plt.suptitle("Debugging Attitude Estimation (Noisy vs True vs Estimated)", fontsize=14)
    plt.xlabel("Time (s)")
    plt.show()

    # Plot Bias Estimation over Time
    fig, axes = plt.subplots(len(bias_labels), 1, figsize=(12, 3 * len(bias_labels)))

    for i, (key, label) in enumerate(bias_labels.items()):
        ax = axes[i]
        ax.plot(time_series, bias_data[key], label=f"{label}", linestyle="-", linewidth=2)
        ax.set_ylabel(label)
        ax.legend()
        ax.grid()

    plt.suptitle("Bias Estimation Over Time", fontsize=14)
    plt.xlabel("Time (s)")
    plt.show()
    
def debug_estimate_altitude():
    """
    Debug estimateAltitude() by plotting true vs estimated altitude and ascent rate.
    """
    
    # Initialize models
    aero_model = VDM.VehicleAerodynamicsModel()
    sensors_model = SensorsModel.SensorsModel(aero_model)  # Sensor model
    estimator = VehicleEstimator.VehicleEstimator(sensorsModel=sensors_model)

    # Set initial conditions
    state = States.vehicleState()
    state.pd = -100.0  # Initial altitude (negative NED convention)
    state.u = 30.0  # Forward speed in m/s
    state.w = 0.0  # No initial vertical speed
    aero_model.setVehicleState(state)
    
    # Store results for plotting
    timesteps =1000
    time_series = [i * aero_model.vehicleDynamics.delta_t for i in range(timesteps)]
    true_data = {"altitude": [], "ascent_rate": []}
    estimated_data = {"altitude": [], "ascent_rate": []}
    
    for _ in range(timesteps):
        # Update the aerodynamics model (true state evolves)
        aero_model.Update(Inputs.controlInputs())
        
        # Update Sensors Model
        sensors_model.update()
        
        # Get true sensor values
        sensorData = sensors_model.sensorsTrue
        
        # Estimate altitude
        h_est, h_dot_est, b_GPS = estimator.estimateAltitude(sensorData, estimator.estimatedState)
        
        # Store results
        true_data["altitude"].append(aero_model.vehicleDynamics.state.pd)  
        estimated_data["altitude"].append(-h_est)
        
        true_data["ascent_rate"].append(aero_model.vehicleDynamics.state.w)  
        estimated_data["ascent_rate"].append(h_dot_est)
    
    # Define labels for plotting
    sensor_labels = {"altitude": "Altitude (m)", "ascent_rate": "Ascent Rate (m/s)"}
    
    # Call the helper function to plot results
    plot_state_debug(time_series, true_data, estimated_data, sensor_labels, "Debugging Altitude & Ascent Rate Estimation")
    
def debug_estimate_airspeed():
    """
    Debug estimateAirspeed() by plotting true vs estimated airspeed.
    """
    aero_model = VDM.VehicleAerodynamicsModel()
    sensors_model = SensorsModel.SensorsModel(aero_model)
    estimator = VehicleEstimator.VehicleEstimator(sensorsModel=sensors_model)
    
    state = States.vehicleState()
    state.Va = 25.0  # True airspeed
    state.u = 25.0  # Forward speed
    aero_model.setVehicleState(state)
    
    timesteps = 1000
    time_series = [i * aero_model.vehicleDynamics.delta_t for i in range(timesteps)]
    true_data = {"airspeed": [], "bias": []}
    estimated_data = {"airspeed": [], "bias": []}
    
    for _ in range(timesteps):
        aero_model.Update(Inputs.controlInputs())
        sensors_model.update()
        sensorData = sensors_model.sensorsTrue
        b_Va, Va_est = estimator.estimateAirspeed(sensorData, estimator.estimatedState)
        
        true_data["airspeed"].append(aero_model.vehicleDynamics.state.Va)
        estimated_data["airspeed"].append(Va_est)
        true_data["bias"].append(0.0)  # Assuming no real bias in sim
        estimated_data["bias"].append(b_Va)
    
    labels = {"airspeed": "Airspeed (m/s)", "bias": "Airspeed Bias"}
    plot_state_debug(time_series, true_data, estimated_data, labels, "Debugging Airspeed Estimation")

def debug_estimate_course():
    """
    Debug estimateCourse() by plotting true vs estimated course and bias.
    """
    aero_model = VDM.VehicleAerodynamicsModel()
    sensors_model = SensorsModel.SensorsModel(aero_model)
    estimator = VehicleEstimator.VehicleEstimator(sensorsModel=sensors_model)
    
    state = States.vehicleState()
    state.yaw = math.radians(30.0)  # Initial true course (30 degrees in radians)
    aero_model.setVehicleState(state)
    
    timesteps = 1000
    time_series = [i * aero_model.vehicleDynamics.delta_t for i in range(timesteps)]
    true_data = {"course": [], "bias": []}
    estimated_data = {"course": [], "bias": []}
    
    for _ in range(timesteps):
        aero_model.Update(Inputs.controlInputs())
        sensors_model.update()
        sensorData = sensors_model.sensorsTrue
        b_chi, chi_est = estimator.estimateCourse(sensorData, estimator.estimatedState)
        
        true_data["course"].append(aero_model.vehicleDynamics.state.yaw)
        estimated_data["course"].append(chi_est)
        true_data["bias"].append(0.0)  # Assuming no real bias in sim
        estimated_data["bias"].append(b_chi)
    
    labels = {"course": "Course (rad)", "bias": "Course Bias"}
    plot_state_debug(time_series, true_data, estimated_data, labels, "Debugging Course Estimation")
    
def debug_update_step():
    """
    Debugs the VehicleEstimator.Update() function by printing the estimated state
    before and after the update step.
    """

    # Initialize models
    aero_model = VDM.VehicleAerodynamicsModel()
    sensors_model = SensorsModel.SensorsModel(aero_model)  # Sensor model
    estimator = VehicleEstimator.VehicleEstimator(sensorsModel=sensors_model)

    # Set initial conditions
    state = States.vehicleState()
    state.Va = 25.0  # Set true airspeed
    state.u = 25.0  # Forward Speed
    state.yaw = math.radians(10.0)  # Initial yaw
    state.pitch = math.radians(5.0)  # Small pitch
    state.roll = math.radians(2.0)  # Small roll
    aero_model.setVehicleState(state)

    print("\n================= DEBUGGING Update() =================")
    print(f"Initial Estimated State:\n{estimator.getEstimatedState()}")

    # Run a few update steps to check state evolution
    for i in range(5):
        print(f"\n--- Iteration {i+1} ---")

        # Print estimated state before update
        print("Before Update:")
        print(estimator.getEstimatedState())

        # Update the aerodynamics model (true state evolves)
        aero_model.Update(Inputs.controlInputs())

        # Update Sensors Model
        sensors_model.update()

        # Call Update() to process the noisy sensor data
        estimator.Update()

        # Print estimated state after update
        print("After Update:")
        print(estimator.getEstimatedState())

        print("===============================================")
  
def plot_full_state_debug(time_series, true_data, estimated_data):
    """
    Plots the estimated state vs. the true state over time in a 4x4 panel layout.
    """
    state_labels = {
        "pn": "North Position (m)", "pe": "East Position (m)", "pd": "Down Position (m)", "u": "Body Velocity u (m/s)",
        "v": "Body Velocity v (m/s)", "w": "Body Velocity w (m/s)", "yaw": "Yaw (rad)", "pitch": "Pitch (rad)",
        "roll": "Roll (rad)", "p": "Angular Rate p (rad/s)", "q": "Angular Rate q (rad/s)", "r": "Angular Rate r (rad/s)",
        "Va": "Airspeed (m/s)", "alpha": "Angle of Attack (rad)", "beta": "Sideslip Angle (rad)", "chi": "Course Angle (rad)"
    }
    
    fig, axes = plt.subplots(4, 4, figsize=(16, 12))
    fig.suptitle("Full State Estimation Debugging", fontsize=16)
    axes = axes.flatten()
    
    for i, (key, label) in enumerate(state_labels.items()):
        axes[i].plot(time_series, true_data[key], label=f"True {label}", linestyle="-", linewidth=1)
        axes[i].plot(time_series, estimated_data[key], label=f"Estimated {label}", linestyle="dotted", linewidth=1)
        axes[i].set_title(label)
        axes[i].legend()
        axes[i].grid()
    
    # Remove any unused subplots
    for j in range(i + 1, len(axes)):
        fig.delaxes(axes[j])
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

def run_estimator_simulation():
    """
    Runs a 1000-step estimator simulation and plots the results.
    """
    # Initialize vehicle models
    aero_model = VDM.VehicleAerodynamicsModel()
    sensors_model = SensorsModel.SensorsModel(aero_model)
    estimator = VehicleEstimator.VehicleEstimator(sensorsModel=sensors_model)
    
    # Set initial state
    state = States.vehicleState()
    state.Va = 25.0  # Airspeed
    state.u = 25.0   # Forward velocity
    state.pd = -100.0  # Altitude (negative down)
    state.yaw = math.radians(10.0)  # Initial yaw
    state.pitch = math.radians(5.0)  # Initial pitch
    state.roll = math.radians(2.0)  # Initial roll
    aero_model.setVehicleState(state)
    
    # Data storage
    timesteps = 1000
    time_series = [i * aero_model.vehicleDynamics.delta_t for i in range(timesteps)]
    true_data = {key: [] for key in state.__dict__.keys() if key != "R"}
    estimated_data = {key: [] for key in true_data.keys()}
    
    # Run simulation
    for _ in range(timesteps):
        aero_model.Update(Inputs.controlInputs())  # Update true state
        sensors_model.update()  # Update sensors
        estimator.Update()  # Update estimator
        
        # Store true and estimated states
        current_state = aero_model.vehicleDynamics.state
        estimated_state = estimator.getEstimatedState()
        
        for key in true_data.keys():
            true_data[key].append(getattr(current_state, key))
            estimated_data[key].append(getattr(estimated_state, key))
    
    # Plot results
    plot_full_state_debug(time_series, true_data, estimated_data)
          
# Run the test
debug_estimate_attitude_true()
debug_estimate_attitude_noisy()
debug_estimate_attitude_bias()
debug_estimate_altitude()
debug_estimate_airspeed()
debug_estimate_course()
debug_update_step()
run_estimator_simulation()