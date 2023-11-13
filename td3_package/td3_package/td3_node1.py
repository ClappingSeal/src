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

    observation = np.array(observation) * 0.04530495
    action, _ = model.predict(observation)

    action_array=[0, 0]
    action_array[0] = (action * velocity)[0]
    action_array[1] = (action * velocity)[1]
    
    return action_array


class TD3Node1(Node):
    def __init__(self):
        super().__init__('td3_node1')
        self.srv = self.create_service(TD3, 'td31', self.td3_callback)
        self.subscription = self.create_subscription(Float64MultiArray, 'drone_info', self.listener_callback, 10)
        self.drone_id = 1.0  # setting value !!!
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

        print(next_position)
        response.path = next_position.tolist()
        return response


def main(args=None):
    rclpy.init(args=args)
    td3_node1 = TD3Node1()
    rclpy.spin(td3_node1)
    td3_node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
