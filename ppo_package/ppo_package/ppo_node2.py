import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from msgs.srv import PPO
import numpy as np
import math
from stable_baselines3 import PPO as PPO_model

model = PPO_model.load("PPO")


def ppo(start, goal, obstacles, velocity=3):
    def change_action(num):
        sq = math.sqrt(2) / 2
        if num == 0:
            return np.array([0, velocity])
        elif num == 1:
            return np.array([velocity * sq, velocity * sq])
        elif num == 2:
            return np.array([velocity, 0])
        elif num == 3:
            return np.array([velocity * sq, -velocity * sq])
        elif num == 4:
            return np.array([0, -velocity])
        elif num == 5:
            return np.array([-velocity * sq, -velocity * sq])
        elif num == 6:
            return np.array([-velocity, 0])
        elif num == 7:
            return np.array([-velocity * sq, velocity * sq])

    start = np.array(start)
    goal = np.array(goal) - start
    scale = 0.02

    obstacle0 = (obstacles[0] - start) * scale
    obstacle1 = (obstacles[0] - start) * scale
    obstacle2 = (obstacles[0] - start) * scale

    observation = np.concatenate((goal, obstacle0, obstacle1, obstacle2, [0, 0], [0, 0], [0, 0]), axis=0)
    observation = np.array(observation)
    action, _ = model.predict(observation)

    print(obstacle0)

    return start + change_action(action) * velocity


class PPONode2(Node):
    def __init__(self):
        super().__init__('ppo_node2')
        self.srv = self.create_service(PPO, 'ppo2', self.ppo_callback)
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

    def ppo_callback(self, request, response):
        obstacles = list(self.current_positions.values())
        # obstacles.append([3, 5])
        if self.start is None:
            self.get_logger().warn('Waiting for drone start position...')
            return response
        goal = np.array([request.goal[0], request.goal[1]])
        next_position = ppo(self.start, goal, obstacles)
        response.path = []
        for value in next_position:
            response.path.append(float(value))
        return response


def main(args=None):
    rclpy.init(args=args)
    ppo_node2 = PPONode2()
    rclpy.spin(ppo_node2)
    ppo_node2.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
