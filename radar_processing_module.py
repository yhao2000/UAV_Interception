# radar_processing_module.py
import numpy as np

class RadarProcessing:
    def __init__(self, recognition_range=800, fov_angle=20):
        """
        Initialize the radar processing module.
        :param recognition_range: Recognition range in meters (default: 800m).
        :param fov_angle: Field of view angle in degrees (default: 20°).
        """
        self.recognition_range = recognition_range
        self.fov_angle = np.radians(fov_angle)  # Convert to radians

    def is_target_in_recognition_range(self, uav_position, uav_orientation, target_position):
        """
        Check if the target is within the recognition range and field of view of the UAV.
        :param uav_position: UAV position [x, y, z] in meters.
        :param uav_orientation: UAV orientation [roll, pitch, yaw] in degrees.
        :param target_position: Target position [x, y, z] in meters.
        :return: Boolean indicating whether the target is in recognition range.
        """
        # Calculate relative position vector
        relative_position = np.array(target_position) - np.array(uav_position)
        distance = np.linalg.norm(relative_position)

        # Check distance
        if distance > self.recognition_range:
            return False

        # Check field of view
        # UAV yaw is used to define the forward-facing direction in 2D plane
        yaw = np.radians(uav_orientation[2])  # Convert yaw to radians
        forward_vector = np.array([np.cos(yaw), np.sin(yaw), 0])  # UAV forward vector in 2D
        relative_vector_2d = relative_position[:2]  # Target position in 2D plane

        # Normalize vectors
        forward_unit_vector = forward_vector[:2] / np.linalg.norm(forward_vector[:2])
        relative_unit_vector = relative_vector_2d / np.linalg.norm(relative_vector_2d)

        # Calculate angle between vectors
        cos_angle = np.dot(forward_unit_vector, relative_unit_vector)
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))  # Clip for numerical stability

        return angle <= self.fov_angle / 2

    def get_target_position(self, uav_position, uav_orientation, target_position, radar_position):
        """
        Determine the target position based on recognition range and radar data.
        :param uav_position: UAV position [x, y, z] in meters.
        :param uav_orientation: UAV orientation [roll, pitch, yaw] in degrees.
        :param target_position: True target position [x, y, z] in meters.
        :param radar_position: Radar-provided target position [x, y, z] in meters.
        :return: Target position [x, y, z] and source ('radar' or 'uav').
        """
        if self.is_target_in_recognition_range(uav_position, uav_orientation, target_position):
            return target_position, "uav"
        else:
            return radar_position, "radar"

# Example usage
if __name__ == "__main__":
    radar_processor = RadarProcessing()

    # Example positions
    uav_position = [0, 0, 700]
    uav_orientation = [0, 0, 0]  # UAV facing along +x axis
    target_position = [600, 0, 700]  # Target is within range and directly ahead
    radar_position = [620, 10, 700]  # Radar
