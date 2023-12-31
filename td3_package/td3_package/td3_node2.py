import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from msgs.srv import TD3
import numpy as np
import math
from stable_baselines3 import TD3 as TD3_model

model = TD3_model.load("TD3")


def td3(start, goal, obstacles, velocity=3):
    start = np.array(start)
    goal = np.array(goal) - start
    obstacle0 = obstacles[0] - start
    obstacle1 = obstacles[0] - start
    obstacle2 = obstacles[0] - start

    observation = np.concatenate((goal, obstacle0, obstacle1, obstacle2, [0, 0], [0, 0], [0, 0]), axis=0)

    observation = np.array(observation) * 0.01
    action, _ = model.predict(observation)

    return start + action * velocity


class TD3Node2(Node):
    def __init__(self):
        super().__init__('td3_node2')
        self.srv = self.create_service(TD3, 'td32', self.td3_callback)
        self.subscription = self.create_subscription(Float64MultiArray, 'drone_info', self.listener_callback, 10)
        self.drone_id = 2.0  # setting value !!!
        self.current_positions = {}
        self.start = None

    def listener_callback(self, msg):
        data = np.array(msg.data)
        n, x, y, _ = data
        if n == self.drone_id:
            self.start = np.array([x, y])
            self.get_logger().info(f'Start{self.drone_id}: {self.start}')
        else:
            self.current_positions[n] = np.array([x, y])
            self.get_logger().info(f'Obstacle: {self.current_positions[n]} for drone {n}')

    def td3_callback(self, request, response):
        obstacles = list(self.current_positions.values())
        # obstacles.append([3, 5])
        if self.start is None:
            self.get_logger().warn('Waiting for drone start position...')
            return response
        goal = np.array([request.goal[0], request.goal[1]])

        next_position = td3(self.start, goal, obstacles)
        response.path = []
        for value in next_position:
            response.path.append(float(value))
        return response


def main(args=None):
    rclpy.init(args=args)
    td3_node2 = TD3Node2()
    rclpy.spin(td3_node2)
    td3_node2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
