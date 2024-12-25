import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from input_module import SimulationInput
from target_simulation_module import TargetSimulation
from radar_processing_module import RadarProcessing
from interception_planning_module import InterceptionPlanner
from uav_dynamics_module import UAVDynamics
from optical_guidance_module import OpticalGuidance

# Initialize modules
input_data = SimulationInput()
target_sim = TargetSimulation(
    position=input_data.target_position,
    speed=input_data.target_speed,
    mode=input_data.target_mode,
    time_step=input_data.time_step
)
radar = RadarProcessing()
planner = InterceptionPlanner()
uav_dynamics = UAVDynamics()
optical_guidance = OpticalGuidance()

# UAV initial state
uav_position = np.array(input_data.uav_position)
uav_velocity = np.array([input_data.uav_speed, 0, 0])
uav_orientation = np.array([0, 0, 0])  # Roll, pitch, yaw in degrees

# Initialize data recording
positions_uav = [uav_position.tolist()]
positions_target = [target_sim.get_position().tolist()]
velocities_uav = [uav_velocity.tolist()]
overloads = []

# GUI setup
fig, ax = plt.subplots()
ax.set_xlim(-500, 5000)
ax.set_ylim(-2000, 2000)
ax.set_title("UAV Interception Simulation")
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")

uav_dot, = ax.plot([], [], 'ro', label="UAV (Red)")
target_dot, = ax.plot([], [], 'bo', label="Target (Blue)")
legend = ax.legend()

def update(frame):
    global uav_position, uav_velocity, uav_orientation

    # Step 1: Update target position
    target_position = target_sim.update_position()
    positions_target.append(target_position.tolist())

    # Step 2: Use radar or optical guidance for target data
    radar_position = target_position  # For simplicity, use the same position
    target_data, source = radar.get_target_position(
        uav_position, uav_orientation, target_position, radar_position
    )

    # Step 3: Plan interception path
    acceleration = optical_guidance.compute_guidance_command(
        uav_position, target_data, uav_velocity
    )

    # Step 4: Update UAV state
    uav_position, uav_velocity, uav_orientation, angular_velocity, overload = uav_dynamics.update_state(
        uav_position, uav_velocity, acceleration, uav_orientation
    )
    positions_uav.append(uav_position.tolist())
    velocities_uav.append(uav_velocity.tolist())
    overloads.append(overload)

    # Step 5: Update GUI
    uav_dot.set_data(uav_position[0], uav_position[1])
    target_dot.set_data(target_position[0], target_position[1])

    # Check if target is hit
    if optical_guidance.is_target_hit(uav_position, target_position):
        print("Target Hit!")
        ani.event_source.stop()

    return uav_dot, target_dot

# Animation
ani = FuncAnimation(fig, update, frames=int(input_data.simulation_time / input_data.time_step), interval=40)

# Display animation
plt.show()

# Save trajectory plot
def save_trajectory_plot():
    fig, ax = plt.subplots()
    ax.set_title("UAV and Target Trajectory")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.plot(
        [pos[0] for pos in positions_uav], [pos[1] for pos in positions_uav], 'r-', label="UAV Trajectory"
    )
    ax.plot(
        [pos[0] for pos in positions_target], [pos[1] for pos in positions_target], 'b-', label="Target Trajectory"
    )
    ax.legend()
    fig.savefig("trajectory_plot.png")
    print("Trajectory plot saved as 'trajectory_plot.png'.")

save_trajectory_plot()

# Save detailed simulation report
def save_simulation_report():
    filename = "simulation_report.csv"
    headers = ["Step", "UAV_X", "UAV_Y", "UAV_Z", "UAV_VX", "UAV_VY", "UAV_VZ", "Overload (g)"]
    with open(filename, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(headers)
        for i, (pos, vel, overload) in enumerate(zip(positions_uav, velocities_uav, overloads)):
            writer.writerow([i] + pos + vel + [overload])
    print(f"Simulation report saved as '{filename}'.")

save_simulation_report()
