import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from msgs.srv import APF
import numpy as np
from tensorflow.keras.models import load_model

# TensorFlow와 Keras 버전 확인
# 라즈베리 파이에서 호환 가능한 버전을 사용하고 있는지 확인하세요
import tensorflow as tf
print("TensorFlow version:", tf.__version__)
import keras
print("Keras version:", keras.__version__)

# 모델을 로드하기 전에 필요한 설정을 추가할 수 있습니다
# 예를 들어, 메모리 사용량을 제한하거나, 모델 최적화를 위한 설정을 할 수 있습니다
# 이 부분은 라즈베리 파이의 하드웨어에 따라 조정해야 할 수 있습니다

# 모델 로드
try:
    model_LSTM = load_model('lstm_drone_positions_model.keras')
except Exception as e:
    print("모델 로드 중 오류 발생:", e)
    # 여기에서 추가적인 오류 처리 또는 로깅을 할 수 있습니다

def main():
    # 여기에 실행 로직을 작성하세요
    pass

if __name__ == "__main__":
    main()
