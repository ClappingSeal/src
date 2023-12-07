import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from msgs.srv import APF
import numpy as np
from tensorflow.keras.models import load_model

model_LSTM = load_model('lstm_drone_positions_model.keras')


def LSTM_prediction(array, time_steps=3):
    def inverse_min_max_scale(scaled_data, min_val, max_val):
        return scaled_data * (max_val - min_val) + min_val

    def preprocess_data(new_data, time_steps=1):
        def min_max_scale(data):
            min_val = np.min(data, axis=0)
            max_val = np.max(data, axis=0)
            scaled_data = (data - min_val) / (max_val - min_val)
            return scaled_data, min_val, max_val

        scaled_data, min_val, max_val = min_max_scale(new_data)
        X_new = []
        for i in range(len(scaled_data) - time_steps):
            X_new.append(scaled_data[i:(i + time_steps), :])
        return np.array(X_new), min_val, max_val

    array = np.nan_to_num(array)
    X_new, min_val, max_val = preprocess_data(array, time_steps)
    predictions = model_LSTM.predict(X_new)
    predictions_rescaled = inverse_min_max_scale(predictions, min_val, max_val)
    return predictions_rescaled


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

print("1112121412")


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
        obstacles = list(self.current_positions.values())
        # obstacles.append([3, 5])
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
