import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from msgs.srv import APF
import numpy as np


def apf(start, goal, obstacles, velocity=3, distract_rate=8):
    q = np.array(start)
    att_force = velocity * (goal - q) / np.linalg.norm(goal - q)
    rep_force = np.zeros(2)

    for obs in obstacles:
        rho = np.linalg.norm(q - obs)
        if rho < distract_rate:
            rep_force += velocity * (1.0 / rho - 1.0 / distract_rate) * ((q - obs) / rho) * 10

    total_force = att_force + rep_force
    q = q + total_force
    return q


class APFNode1(Node):
    def __init__(self):
        super().__init__('apf_node1')
        self.srv = self.create_service(APF, 'apf1', self.apf_callback)
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

    def apf_callback(self, request, response):
        obstacles = []

        # LSTM 사용
        new_data = np.array([
            self.recent_positions[self.drone_id][0],
            self.recent_positions[self.drone_id][1],
            self.recent_positions[self.drone_id][2],
            [None, None]
        ])
        predicted_position = LSTM_prediction(new_data)

        # 예측된 위치를 obstacles에 추가
        if predicted_position is not None:
            obstacles.append(predicted_position)

        # 가장 최근 위치 추가
        if len(self.recent_positions[self.drone_id]) >= 1:
            obstacles.append(self.recent_positions[self.drone_id][-1])

        if self.start is None:
            self.get_logger().warn('Waiting for drone start position...')
            return response

        goal = np.array([request.goal[0], request.goal[1]])
        next_position = apf(self.start, goal, obstacles)
        response.path = list(np.array(next_position).flatten())
        return response


def main(args=None):
    rclpy.init(args=args)
    apf_node1 = APFNode1()
    rclpy.spin(apf_node1)
    apf_node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
