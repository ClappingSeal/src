import pandas as pd
from tensorflow.keras.models import load_model
import numpy as np

# LSTM 모델 로드
model_LSTM = load_model('lstm_drone_positions_model.keras')


# LSTM 예측 함수
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


# 예측 실행
new_data = np.array([
    [1, 5],
    [1, 5],
    [1, 5],
    [0, 0]  # None 대신 0 사용
])

print(LSTM_prediction(new_data))
