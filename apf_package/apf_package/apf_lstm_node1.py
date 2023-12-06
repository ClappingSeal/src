import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from msgs.srv import APF
import numpy as np
from tensorflow.keras.models import load_model
from sklearn.preprocessing import MinMaxScaler

# 저장된 모델 로드
model_LSTM = load_model('lstm_drone_positions_model.keras')


def LSTM_prediction(array):
    def preprocess_data(new_data, scaler, time_steps):
        # 데이터 정규화
        new_scaled_data = scaler.transform(new_data)

        # 시퀀스 생성
        X_new = []
        for i in range(len(new_scaled_data) - time_steps):
            X_new.append(new_scaled_data[i:(i + time_steps)])
        X_new = np.array(X_new)

        return X_new

    scaler = MinMaxScaler()
    scaler.fit(array)
    X_new = preprocess_data(array, scaler, time_steps=3)

    predictions = model_LSTM.predict(X_new)
    predictions_rescaled = scaler.inverse_transform(predictions)
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
        if len(self.recent_positions[drone_id]) > 3:
            self.recent_positions[drone_id].pop(0)

    def apf_callback(self, request, response):
        obstacles = []

        # LSTM
        for drone_id, positions in self.recent_positions.items():
            if drone_id != self.drone_id and len(positions) >= 3:
                new_data = np.array([
                    positions[0],
                    positions[1],
                    positions[2],
                    [None, None]
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
        return response


def main(args=None):
    rclpy.init(args=args)
    apf_node1 = APFNode1()
    rclpy.spin(apf_node1)
    apf_node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()