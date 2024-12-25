# interception_planning_module.py
import numpy as np

class InterceptionPlanner:
    def __init__(self, max_acceleration=40, time_step=0.04):
        """
        Initialize the interception planner.
        :param max_acceleration: Maximum allowable acceleration in m/s² (default: 40 m/s², ~4g).
        :param time_step: Time step in seconds.
        """
        self.max_acceleration = max_acceleration  # Maximum acceleration (4g)
        self.time_step = time_step  # Time step for updates

    def predict_target_position(self, target_position, target_velocity, prediction_time):
        """
        Predict the future position of the target.
        :param target_position: Current target position [x, y, z] in meters.
        :param target_velocity: Current target velocity [vx, vy, vz] in m/s.
        :param prediction_time: Time into the future to predict in seconds.
        :return: Predicted [x, y, z] position of the target.
        """
        predicted_position = np.array(target_position) + np.array(target_velocity) * prediction_time
        return predicted_position

    def calculate_interception_point(self, uav_position, uav_velocity, target_position, target_velocity):
        """
        Calculate the optimal interception point based on the current states.
        :param uav_position: UAV current position [x, y, z] in meters.
        :param uav_velocity: UAV current velocity [vx, vy, vz] in m/s.
        :param target_position: Target current position [x, y, z] in meters.
        :param target_velocity: Target current velocity [vx, vy, vz] in m/s.
        :return: Interception point [x, y, z] in meters.
        """
        # Time prediction for interception (simple approximation)
        relative_position = np.array(target_position) - np.array(uav_position)
        relative_velocity = np.linalg.norm(np.array(uav_velocity) - np.array(target_velocity))
        if relative_velocity == 0:
            return target_position  # Avoid division by zero
        time_to_intercept = np.linalg.norm(relative_position) / relative_velocity

        # Predict target's future position
        interception_point = self.predict_target_position(target_position, target_velocity, time_to_intercept)
        return interception_point

    def compute_next_step(self, uav_position, uav_velocity, target_position, target_velocity, strategy="pursuit"):
        """
        Compute the UAV's next movement step based on the interception strategy.
        :param uav_position: UAV current position [x, y, z] in meters.
        :param uav_velocity: UAV current velocity [vx, vy, vz] in m/s.
        :param target_position: Target current position [x, y, z] in meters.
        :param target_velocity: Target current velocity [vx, vy, vz] in m/s.
        :param strategy: Interception strategy: "pursuit" (追击) or "intercept" (迎击).
        :return: Updated UAV position, velocity, and acceleration vector.
        """
        if strategy == "pursuit":
            # Pursuit strategy: Directly pursue the current target position
            aim_point = np.array(target_position)
        elif strategy == "intercept":
            # Intercept strategy: Predict and pursue the interception point
            aim_point = self.calculate_interception_point(uav_position, uav_velocity, target_position, target_velocity)
        else:
            raise ValueError("Invalid strategy. Use 'pursuit' or 'intercept'.")

        # Calculate direction and acceleration needed to reach the aim point
        direction = aim_point - np.array(uav_position)
        distance = np.linalg.norm(direction)
        direction_unit_vector = direction / distance if distance != 0 else np.zeros_like(direction)

        # Compute acceleration limited by max acceleration
        desired_acceleration = direction_unit_vector * min(distance / self.time_step**2, self.max_acceleration)

        # Update velocity and position
        new_velocity = np.array(uav_velocity) + desired_acceleration * self.time_step
        new_position = np.array(uav_position) + new_velocity * self.time_step

        return new_position, new_velocity, desired_acceleration

# Example usage
if __name__ == "__main__":
    planner = InterceptionPlanner()

    # Initial conditions
    uav_position = [0, 0, 700]
    uav_velocity = [100, 0, 0]
    target_position = [4000, 0, 700]
    target_velocity = [50, 0, 0]

    # Simulate for 10 seconds
    simulation_time = 10
    time_step = planner.time_step
    steps = int(simulation_time / time_step)

    for step in range(steps):
        uav_position, uav_velocity, acceleration = planner.compute_next_step(
            uav_position, uav_velocity, target_position, target_velocity, strategy="intercept"
        )
        print(f"Step {step + 1}: UAV Position: {uav_position}, UAV Velocity: {uav_velocity}, Acceleration: {acceleration}")
