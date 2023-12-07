import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from msgs.srv import APF
import numpy as np
from tensorflow.keras.models import load_model

model_LSTM = load_model('lstm_drone_positions_model.keras')

