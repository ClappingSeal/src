import numpy as np
import time
from stable_baselines3 import PPO, TD3
from APF_Settings import APFEnv

def get_ab(model, pos, obstacles, goal):
    env = APFEnv(pos)
    get_state = env.apf_rev_rotate(goal, obstacles)
    state = np.concatenate((
        get_state[0],
        get_state[1],
        get_state[2],
        np.array([np.linalg.norm(env.heuristic(goal))])
    ))
    action, _states = model.predict(state, deterministic=True)
    a = action[0]
    b = [action[1], action[2]]
    if np.linalg.norm(b) > 0.8:
        b = np.array(b) / np.linalg.norm(b)

    b = env.apf_inverse_rotate(goal, obstacles, b)

    return a, b

def measure_time(model, start_positions, goal_positions, obstacles, num_steps=100):
    start_time = time.time()
    for _ in range(num_steps):
        for pos, goal in zip(start_positions, goal_positions):
            get_ab(model, pos, obstacles, goal)
    end_time = time.time()
    total_time = end_time - start_time
    average_time_per_step = total_time / num_steps
    return total_time, average_time_per_step

# 모델 로드
model_ppo = PPO.load("logs_ppo/best_model.zip")
model_td3 = TD3.load("logs_td3/best_model.zip")

# 로봇들의 출발점과 목표점 설정
start_positions = [
    np.array([0, 0], dtype=float),
    np.array([20, 20.1], dtype=float),
]

goal_positions = [
    np.array([21, 21], dtype=float),
    np.array([-1, -1], dtype=float),
]

# 고정 장애물 배열
obstacles = np.array([
    [10, 10, 1],
], dtype=float)

# PPO 모델 시간 측정
ppo_total_time, ppo_avg_time_per_step = measure_time(model_ppo, start_positions, goal_positions, obstacles)
print(f"PPO 모델 100 스텝 총 시간: {ppo_total_time:.2f} 초")
print(f"PPO 모델 스텝당 평균 시간: {ppo_avg_time_per_step:.4f} 초")

# TD3 모델 시간 측정
td3_total_time, td3_avg_time_per_step = measure_time(model_td3, start_positions, goal_positions, obstacles)
print(f"TD3 모델 100 스텝 총 시간: {td3_total_time:.2f} 초")
print(f"TD3 모델 스텝당 평균 시간: {td3_avg_time_per_step:.4f} 초")
