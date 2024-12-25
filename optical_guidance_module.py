# optical_guidance_module.py
import numpy as np

class OpticalGuidance:
    def __init__(self, fov=20, lock_range=800, interception_threshold=1):
        """
        Initialize the optical guidance module.
        :param fov: Field of view in degrees (default: 20°).
        :param lock_range: Maximum lock-on range in meters (default: 800m).
        :param interception_threshold: Threshold for interception in meters (default: 1m).
        """
        self.fov = np.radians(fov)  # Convert FOV to radians
        self.lock_range = lock_range
        self.interception_threshold = interception_threshold

    def is_target_in_fov(self, uav_position, uav_orientation, target_position):
        """
        Check if the target is within the field of view of the UAV.
        :param uav_position: UAV position [x, y, z] in meters.
        :param uav_orientation: UAV yaw angle in degrees.
        :param target_position: Target position [x, y, z] in meters.
        :return: Boolean indicating whether the target is within the field of view.
        """
        # Calculate relative position vector
        relative_position = np.array(target_position) - np.array(uav_position)
        distance = np.linalg.norm(relative_position)

        # Check distance
        if distance > self.lock_range:
            return False

        # Check field of view (based on yaw only for simplicity)
        yaw = np.radians(uav_orientation[2])  # Convert yaw to radians
        forward_vector = np.array([np.cos(yaw), np.sin(yaw), 0])  # UAV forward vector in 2D
        relative_vector_2d = relative_position[:2]  # Target position in 2D plane

        # Normalize vectors
        forward_unit_vector = forward_vector[:2] / np.linalg.norm(forward_vector[:2])
        relative_unit_vector = relative_vector_2d / np.linalg.norm(relative_vector_2d)

        # Calculate angle between vectors
        cos_angle = np.dot(forward_unit_vector, relative_unit_vector)
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # Clip for numerical stability

        return angle <= self.fov / 2

    def compute_guidance(self, uav_position, target_position):
        """
        Compute the guidance vector to steer the UAV toward the target.
        :param uav_position: UAV position [x, y, z] in meters.
        :param target_position: Target position [x, y, z] in meters.
        :return: Guidance vector [gx, gy, gz].
        """
        guidance_vector = np.array(target_position) - np.array(uav_position)
        return guidance_vector / np.linalg.norm(guidance_vector)  # Normalize guidance vector

    def execute_guidance(self, uav_position, uav_velocity, target_position):
        """
        Execute the optical guidance to steer the UAV toward the target.
        :param uav_position: UAV position [x, y, z] in meters.
        :param uav_velocity: UAV velocity [vx, vy, vz] in m/s.
        :param target_position: Target position [x, y, z] in meters.
        :return: Updated UAV position, velocity, and interception status.
        """
        # Check if within interception range
        distance_to_target = np.linalg.norm(np.array(target_position) - np.array(uav_position))
        if distance_to_target <= self.interception_threshold:
            return uav_position, uav_velocity, True  # Target intercepted

        # Compute guidance vector
        guidance_vector = self.compute_guidance(uav_position, target_position)

        # Update UAV velocity (simple proportional control)
        new_velocity = np.array(uav_velocity) + guidance_vector * 10 * self.time_step
        new_position = np.array(uav_position) + new_velocity * self.time_step

        return new_position, new_velocity, False  # Target not yet intercepted

# Example usage
if __name__ == "__main__":
    optical = OpticalGuidance()

    # Initial UAV and target states
    uav_position = [0, 0, 700]
    uav_velocity = [100, 0, 0]
    target_position = [100, 100, 700]
    uav_orientation = [0, 0, 0]  # Roll, pitch, yaw in degrees

    # Simulate optical guidance for 5 steps
    for step in range(5):
        if optical.is_target_in_fov(uav_position, uav_orientation, target_position):
            uav_position, uav_velocity, intercepted = optical.execute_guidance(
                uav_position, uav_velocity, target_position
            )
            print(f"Step {step + 1}: Position: {uav_position}, Velocity: {uav_velocity}, Intercepted: {intercepted}")
            if intercepted:
                print("Target successfully intercepted!")
                break
        else:
            print(f"Step {step + 1}: Target not in FOV.")
