import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from msgs.srv import PPO
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense
import numpy as np
import math
from stable_baselines3 import PPO as PPO_model


def create_model():
    model = Sequential()
    model.add(LSTM(20, activation='tanh', input_shape=(3, 2), return_sequences=True))
    model.add(LSTM(20, activation='tanh'))
    model.add(Dense(2))
    return model


model = PPO_model.load("PPO")
model_LSTM = create_model()
model_LSTM.load_weights('lstm_drone_positions_model.keras')


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

    array = np.nan_to_num(array)
    X_new, min_val, max_val = preprocess_data(array, time_steps)
    predictions = model_LSTM.predict(X_new)
    predictions_rescaled = inverse_min_max_scale(predictions, min_val, max_val)
    return predictions_rescaled


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
    obstacle1 = (obstacles[1] - start) * scale
    obstacle2 = (obstacles[0] - start) * scale

    observation = np.concatenate((goal, obstacle0, obstacle1, obstacle2, [0, 0], [0, 0], [0, 0]), axis=0)
    observation = np.array(observation)
    action, _ = model.predict(observation)

    return start + change_action(action) * velocity


class PPONode1(Node):
    def __init__(self):
        super().__init__('ppo_lstm_node1')
        self.srv = self.create_service(PPO, 'ppo1', self.ppo_callback)
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

    def ppo_callback(self, request, response):
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
        next_position = ppo(self.start, goal, obstacles)
        response.path = list(np.array(next_position).flatten())
        print(obstacles)
        return response


def main(args=None):
    rclpy.init(args=args)
    ppo_node1 = PPONode1()
    rclpy.spin(ppo_node1)
    ppo_node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
