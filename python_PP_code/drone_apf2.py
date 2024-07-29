import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
from apf_code import APFEnv
import numpy as np
import subprocess


class PositionSubscriber(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('position_subscriber')
        self.drone_id = 2  # Set the drone ID here
        self.force_magnitude = 1
        self.height = 6
        self.limit = 3
        self.goal_threshold = 1
        self.timer = 2
        
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'drone_info',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.positions = {}
        self.obstacles = []
        self.goal = [goal_x, goal_y]

        self.current_process = None
        self.obstacles_copy = None
        self.obstacles_radius = None

        self.create_timer(self.timer, self.apf)

    def listener_callback(self, msg):
        drone_id = int(msg.data[0])
        x = msg.data[1]
        y = msg.data[2]

        if drone_id == self.drone_id:
            self.positions['drone_position'] = [x, y]
        else:
            self.positions[drone_id] = [x, y]

        self.obstacles = [pos for key, pos in self.positions.items() if key != 'drone_position']

    def print_positions(self):
        drone_position = self.positions.get('drone_position', None)
        if drone_position:
            drone_data = [[drone_position[0], drone_position[1]], self.obstacles, self.goal]
            print(drone_data)

    def apf(self):
        drone_position = self.positions.get('drone_position', None)
        if drone_position:
            self.obstacles_copy = [point[:] for point in self.obstacles]
            self.obstacles_radius = self.add_radius(self.obstacles_copy)
            drone_data = [[drone_position[0], drone_position[1]], self.obstacles_radius, self.goal]

            pos = np.array(drone_data[0])

            # landing
            if np.linalg.norm(pos - self.goal) < self.goal_threshold:
                self.land()
                self.timer.cancel()
                rclpy.shutdown()
                return

            # calculate and activate
            env = APFEnv(drone_data[0])
            force = env.apf(drone_data[2], drone_data[1])
            new_pos = pos + force * self.force_magnitude
            self.goto_client(new_pos[0], new_pos[1])

    def add_radius(self, points):
        for point in points:
            if len(point) == 2:
                point.append(self.limit)
        return points

    def goto_client(self, x, y):
        if self.current_process is not None:
            self.current_process.terminate()
        command = ['ros2', 'run', 'drone_package', f'goto_client{self.drone_id}', str(x), str(y), str(self.height)]
        self.current_process = subprocess.Popen(command)

    def land(self):
        if self.current_process is not None:
            self.current_process.terminate()
        command = ['ros2', 'run', 'drone_package', f'land_client{self.drone_id}']
        subprocess.run(command)


def main(args=None):
    rclpy.init(args=args)

    # Parse command line arguments
    args_without_ros = rclpy.utilities.remove_ros_args(args)
    if len(args_without_ros) != 3:
        print("Usage: python3 drone_apf.py <goal_x> <goal_y>")
        return

    goal_x = float(args_without_ros[1])
    goal_y = float(args_without_ros[2])

    position_subscriber = PositionSubscriber(goal_x, goal_y)
    try:
        rclpy.spin(position_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        position_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
