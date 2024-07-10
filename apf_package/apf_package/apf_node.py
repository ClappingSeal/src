import rclpy
from rclpy.node import Node
from msgs.srv import STATE
import numpy as np


class APFEnvironment:
    def __init__(self, pos, r=10):
        self.pos = np.array(pos, dtype=float)
        self.r = r
        self.limit = 3

    def within_obs(self, obs_pos):
        obs_pos = [np.array(pos) for pos in obs_pos]
        within_radius = [pos for pos in obs_pos if np.linalg.norm(pos - self.pos) <= self.r]

        if not within_radius:
            within_radius = [np.array([self.pos[0], self.pos[1]])]  # 로봇의 현재 위치를 포함하는 더미 항목 추가

        return within_radius

    def att_force(self, goal, att_gain=1):
        goal_vector = np.array(goal - self.pos)
        norm = np.linalg.norm(goal_vector)
        if norm == 0:
            return np.zeros_like(goal_vector)
        return goal_vector / norm * att_gain

    def rep_force(self, obs_pos, rep_gain=0.1):
        obs_within_radius = obs_pos
        force = np.zeros(2)

        for obs in obs_within_radius:
            distance_vector = self.pos - obs
            distance = np.linalg.norm(distance_vector)
            if distance == 0:
                continue
            if (self.limit < distance) and (distance < self.r):
                repulsive_force_magnitude = rep_gain * ((1 / pow((distance - self.limit), 0.5)) + 1)
                repulsive_force_direction = distance_vector / distance
                force += repulsive_force_magnitude * repulsive_force_direction
            elif distance <= self.limit:
                repulsive_force_magnitude = 10
                repulsive_force_direction = distance_vector / distance
                force += repulsive_force_magnitude * repulsive_force_direction
        return force

    def apf(self, goal, obs_pos):
        goal = np.array(goal)
        total_force = self.att_force(goal) + self.rep_force(obs_pos)
        return total_force / np.linalg.norm(total_force) * 0.1


class APFNode(Node):
    def __init__(self):
        super().__init__('apf_node')
        self.srv = self.create_service(STATE, 'apf', self.apf_callback)

    def apf_callback(self, request, response):
        start = request.start
        goal = request.goal
        x = request.x
        y = request.y
        obstacles = list(zip(x, y))

        force_vector = self.apf(start, goal, obstacles)
        response.force = force_vector
        print("start : ", start)
        print("goal : ", goal)
        print("obs : ", obstacles)
        return response

    def apf(self, start, goal, obstacles):
        env = APFEnvironment(start)
        force = np.array(env.apf(goal, obstacles)).astype(float)
        return force


def main(args=None):
    rclpy.init(args=args)
    apf_node = APFNode()
    rclpy.spin(apf_node)
    apf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
