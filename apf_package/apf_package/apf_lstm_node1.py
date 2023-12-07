import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from msgs.srv import APF
import numpy as np
from tensorflow.keras.models import load_model

# TensorFlow와 Keras 버전 확인
import tensorflow as tf
print("TensorFlow version:", tf.__version__)
import keras
print("Keras version:", keras.__version__)

# 모델 로드
try:
    model_LSTM = load_model('lstm_drone_positions_model.keras')
except Exception as e:
    print("모델 로드 중 오류 발생:", e)

def main():
    # ROS2 초기화
    rclpy.init(args=None)

    # 여기에 노드 생성 및 실행 로직을 작성하세요
    # 예: node = MyNode()

    try:
        # ROS2 스피닝 - 노드가 메시지를 받고 처리할 수 있도록 합니다.
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('노드 종료')
    finally:
        # ROS2 정리
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
