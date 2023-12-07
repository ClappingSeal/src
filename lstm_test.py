import numpy as np
import tensorflow as tf

# TensorFlow Lite 모델 로드
interpreter = tf.lite.Interpreter(model_path='lstm_drone_positions_model.tflite')
interpreter.allocate_tensors()

# 입력 및 출력 텐서 정보 가져오기
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()


# LSTM 예측 함수 (TensorFlow Lite 사용)
def LSTM_prediction_tflite(array, time_steps=3):
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


# 예측 실행
new_data = np.array([
    [1, 5],
    [1, 5],
    [1, 5],
    [0, 0]  # None 대신 0 사용
])

print(LSTM_prediction_tflite(new_data))
