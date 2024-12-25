# target_simulation_module.py
import numpy as np

class TargetSimulation:
    def __init__(self, position, speed, mode, time_step=0.04):
        """
        Initialize the target simulation.
        :param position: Initial [x, y, z] position of the target in meters.
        :param speed: Speed of the target in m/s.
        :param mode: Motion type: "linear", "circular", "climb", "dive".
        :param time_step: Simulation time step in seconds.
        """
        self.position = np.array(position, dtype=float)
        self.speed = speed
        self.mode = mode
        self.time_step = time_step

        # Internal variables for circular motion
        self.angle = 0  # Angle in radians, used for circular motion
        self.radius = 500  # Default radius for circular motion in meters

        # Vertical speed for climb or dive
        self.vertical_speed = 0
        if mode == "climb":
            self.vertical_speed = 4  # Maximum climb rate: 4 m/s
        elif mode == "dive":
            self.vertical_speed = -10  # Maximum dive rate: -10 m/s

    def update_position(self):
        """
        Update the target's position based on its motion mode.
        :return: Updated [x, y, z] position in meters.
        """
        if self.mode == "linear":
            # Linear motion in x direction
            self.position[0] += self.speed * self.time_step

        elif self.mode == "circular":
            # Circular motion in the xy-plane
            self.angle += (self.speed / self.radius) * self.time_step  # Update angle
            self.position[0] = self.radius * np.cos(self.angle)
            self.position[1] = self.radius * np.sin(self.angle)

        elif self.mode in ["climb", "dive"]:
            # Linear motion with vertical speed adjustment
            self.position[0] += self.speed * self.time_step  # Horizontal motion
            self.position[2] += self.vertical_speed * self.time_step  # Vertical motion

        return self.position

    def get_position(self):
        """
        Get the current position of the target.
        :return: Current [x, y, z] position in meters.
        """
        return self.position

# Example usage
if __name__ == "__main__":
    # Initialize target simulation
    target = TargetSimulation(position=[4000, 0, 700], speed=50, mode="climb", time_step=0.04)

    # Simulate for 10 seconds
    simulation_time = 10  # seconds
    steps = int(simulation_time / target.time_step)

    for step in range(steps):
        position = target.update_position()
        print(f"Time: {step * target.time_step:.2f}s, Position: {position}")
