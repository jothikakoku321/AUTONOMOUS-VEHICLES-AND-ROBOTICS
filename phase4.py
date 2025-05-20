import math
import time

# Differential drive robot class
class DiffDriveRobot:
    def _init_(self, x=0, y=0, theta=0):
        self.x = x          # Robot x position
        self.y = y          # Robot y position
        self.theta = theta  # Robot orientation (radians)

    def move(self, v, w, dt):
        # v: linear velocity, w: angular velocity, dt: time step
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt
        self.theta = self.theta % (2 * math.pi)

    def get_position(self):
        return (self.x, self.y, self.theta)

# Simple proportional controller to follow waypoints
def follow_path(robot, waypoints, dt=0.1):
    K_linear = 1.0
    K_angular = 4.0
    for wp in waypoints:
        while True:
            x, y, theta = robot.get_position()
            dx = wp[0] - x
            dy = wp[1] - y
            distance = math.hypot(dx, dy)
            if distance < 0.05:  # close enough to waypoint
                break
            path_angle = math.atan2(dy, dx)
            angle_diff = path_angle - theta
            # Normalize angle_diff to [-pi, pi]
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            v = K_linear * distance
            w = K_angular * angle_diff

            # Limit speeds
            v = min(v, 0.5)
            w = max(min(w, 1.0), -1.0)

            robot.move(v, w, dt)
            print(f"Robot position: x={robot.x:.2f}, y={robot.y:.2f}, theta={math.degrees(robot.theta):.1f}°")
            time.sleep(dt)

# Example usage
if _name_ == "_main_":
    robot = DiffDriveRobot()
    # Define waypoints for the robot to follow
    waypoints = [(1, 0), (1, 1), (0, 1), (0, 0)]
    follow_path(robot, waypoints)
