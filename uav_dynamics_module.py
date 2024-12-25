# uav_dynamics_module.py
import numpy as np

class UAVDynamics:
    def __init__(self, mass=25, time_step=0.04):
        """
        Initialize the UAV dynamics model.
        :param mass: UAV mass in kilograms (default: 25 kg).
        :param time_step: Simulation time step in seconds.
        """
        self.mass = mass  # UAV mass in kg
        self.time_step = time_step  # Time step in seconds

    def update_state(self, position, velocity, acceleration, orientation):
        """
        Update the UAV's state based on the applied acceleration and current state.
        :param position: Current UAV position [x, y, z] in meters.
        :param velocity: Current UAV velocity [vx, vy, vz] in m/s.
        :param acceleration: Applied acceleration [ax, ay, az] in m/s².
        :param orientation: Current UAV orientation [roll, pitch, yaw] in degrees.
        :return: Updated position, velocity, orientation, angular velocity, and overload.
        """
        # Update position and velocity
        new_velocity = np.array(velocity) + np.array(acceleration) * self.time_step
        new_position = np.array(position) + new_velocity * self.time_step

        # Calculate angular adjustments (simplified for demonstration purposes)
        pitch_angle = np.degrees(np.arctan2(acceleration[2], np.linalg.norm(acceleration[:2])))
        yaw_angle = np.degrees(np.arctan2(acceleration[1], acceleration[0]))
        roll_angle = 0  # For simplicity, assume no rolling in this model

        # Update orientation
        new_orientation = [roll_angle, pitch_angle, yaw_angle]

        # Calculate angular velocities (change rates)
        angular_velocity = (np.array(new_orientation) - np.array(orientation)) / self.time_step

        # Calculate overload
        overload = np.linalg.norm(acceleration) / 9.81  # Overload in g

        return new_position, new_velocity, new_orientation, angular_velocity, overload

# Example usage
if __name__ == "__main__":
    uav = UAVDynamics()

    # Initial UAV state
    position = [0, 0, 700]
    velocity = [100, 0, 0]
    acceleration = [2, 0.5, -1]  # Example acceleration vector
    orientation = [0, 0, 0]  # Roll, pitch, yaw in degrees

    # Simulate for 5 steps
    for step in range(5):
        position, velocity, orientation, angular_velocity, overload = uav.update_state(
            position, velocity, acceleration, orientation
        )
        print(f"Step {step + 1}:")
        print(f"  Position: {position}")
        print(f"  Velocity: {velocity}")
        print(f"  Orientation: {orientation}")
        print(f"  Angular Velocity: {angular_velocity}")
        print(f"  Overload: {overload:.2f} g")
