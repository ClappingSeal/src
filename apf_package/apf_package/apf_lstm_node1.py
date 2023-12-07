import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from msgs.srv import APF
import tensorflow as tf
import numpy as np


interpreter = tf.lite.Interpreter(model_path='lstm_drone_positions_model.tflite')
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()


# LSTM
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

    array = np.array(array, dtype=np.float32)
    X_new, min_val, max_val = preprocess_data(array, time_steps)

    # TensorFlow Lite 모델로 예측
    interpreter.set_tensor(input_details[0]['index'], X_new)
    interpreter.invoke()
    predictions = interpreter.get_tensor(output_details[0]['index'])

    # 예측값 역변환
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


class APFNode1(Node):
    def __init__(self):
        super().__init__('apf_lstm_node1')
        self.srv = self.create_service(APF, 'apf1', self.apf_callback)
        self.subscription = self.create_subscription(Float64MultiArray, 'drone_info', self.listener_callback, 10)
        self.drone_id = 1.0  # setting value !!!
        self.current_positions = {}
        self.recent_positions = {}
        self.start = None

    def listener_callback(self, msg):
        data = np.array(msg.data)
        n, x, y, _ = data
        if n == self.drone_id:
            self.start = np.array([x, y])
            self.get_logger().info(f'Start{self.drone_id}: {self.start}')
        else:
            position = np.array([x, y])
            self.current_positions[n] = position
            self.update_recent_positions(n, position)
            self.get_logger().info(f'Obstacle: {position} for drone {n}')

    def update_recent_positions(self, drone_id, position):
        if drone_id not in self.recent_positions:
            self.recent_positions[drone_id] = []

        self.recent_positions[drone_id].append(position)

        # 리스트의 길이가 3보다 클 경우, 가장 오래된 위치를 제거
        while len(self.recent_positions[drone_id]) > 3:
            self.recent_positions[drone_id].pop(0)

        # 리스트의 길이가 3보다 짧을 경우, 가장 최근의 위치로 나머지 부분을 채움
        while len(self.recent_positions[drone_id]) < 3:
            self.recent_positions[drone_id].append(position)

    def apf_callback(self, request, response):
        obstacles = []

        # LSTM
        for drone_id, positions in self.recent_positions.items():
            if drone_id != self.drone_id and len(positions) >= 3:
                new_data = np.array([
                    positions[0],
                    positions[1],
                    positions[2],
                    [0, 0]
                ])
                predicted_position = LSTM_prediction(new_data)

                # 예측된 위치
                if predicted_position is not None:
                    obstacles.append(predicted_position)

                # 가장 최근 위치
                obstacles.append(positions[2])

        if self.start is None:
            self.get_logger().warn('Waiting for drone start position...')
            return response

        goal = np.array([request.goal[0], request.goal[1]])
        next_position = apf(self.start, goal, obstacles)
        response.path = list(np.array(next_position).flatten())
        print(obstacles)
        return response


def main(args=None):
    rclpy.init(args=args)
    apf_node1 = APFNode1()
    rclpy.spin(apf_node1)
    apf_node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
