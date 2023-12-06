import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from msgs.srv import APF
import numpy as np
from tensorflow.keras.models import load_model
from sklearn.preprocessing import MinMaxScaler

# 저장된 모델 로드
model_LSTM = load_model('lstm_drone_positions_model.keras')

