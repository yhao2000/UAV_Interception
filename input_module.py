class SimulationInput:
    def __init__(self):
        """
        Initialize default simulation input parameters.
        """
        # Target UAV parameters
        self.target_position = [4000, 0, 700]  # [x, y, z] in meters
        self.target_speed = 50  # Speed in m/s
        self.target_mode = "linear"  # Motion type: "linear", "circular", "ascending", "descending"

        # Interceptor UAV parameters
        self.uav_position = [0, 0, 700]  # [x, y, z] in meters
        self.uav_speed = 100  # Speed in m/s
        self.uav_orientation = [0, 0, 0]  # [roll, pitch, yaw] in degrees

        # Simulation settings
        self.simulation_time = 60  # Duration in seconds
        self.time_step = 0.04  # Time step in seconds

    def set_target_parameters(self, position, speed, mode):
        """
        Set the initial parameters of the target UAV.
        :param position: List of [x, y, z] position in meters.
        :param speed: Speed in m/s.
        :param mode: Motion type, "linear", "circular", "ascending", or "descending".
        """
        self.target_position = position
        self.target_speed = speed
        self.target_mode = mode

    def set_uav_parameters(self, position, speed, orientation):
        """
        Set the initial parameters of the interceptor UAV.
        :param position: List of [x, y, z] position in meters.
        :param speed: Speed in m/s.
        :param orientation: List of [roll, pitch, yaw] in degrees.
        """
        self.uav_position = position
        self.uav_speed = speed
        self.uav_orientation = orientation

    def set_simulation_settings(self, duration, time_step):
        """
        Set the simulation settings.
        :param duration: Total simulation time in seconds.
        :param time_step: Time step in seconds.
        """
        self.simulation_time = duration
        self.time_step = time_step

    def get_simulation_inputs(self):
        """
        Return a dictionary of all simulation input parameters.
        :return: Dictionary of input parameters.
        """
        return {
            "target": {
                "position": self.target_position,
                "speed": self.target_speed,
                "mode": self.target_mode,
            },
            "uav": {
                "position": self.uav_position,
                "speed": self.uav_speed,
                "orientation": self.uav_orientation,
            },
            "settings": {
                "simulation_time": self.simulation_time,
                "time_step": self.time_step,
            },
        }
