import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point

class APFNode(Node):
    def __init__(self):
        super().__init__('apf_node1')
        self.srv = self.create_service(Float64MultiArray, 'calculate_force_vector', self.calculate_force_callback)

    def calculate_force_callback(self, request, response):
        data = request.data
        start = data[0:2]
        goal = data[2:4]
        obstacles = [data[i:i+2] for i in range(4, len(data), 2)]

        force_vector = self.calculate_force(start, goal, obstacles)
        response.data = force_vector
        return response

    def calculate_force(self, start, goal, obstacles):
        # 인공 잠재장 알고리즘을 통해 힘 벡터를 계산하는 로직을 구현
        force = [0.0, 0.0]
        # 계산 예시 (구체적인 APF 알고리즘을 구현)
        for obstacle in obstacles:
            # 예제: 장애물로 인한 repulsive force 계산 (실제 로직은 다를 수 있음)
            ox, oy = obstacle
            dx, dy = start[0] - ox, start[1] - oy
            distance = (dx**2 + dy**2)**0.5
            if distance < 1.0:  # 임계 거리
                repulsive_force = [dx / distance, dy / distance]
                force[0] += repulsive_force[0]
                force[1] += repulsive_force[1]
        
        attractive_force = [goal[0] - start[0], goal[1] - start[1]]
        force[0] += attractive_force[0]
        force[1] += attractive_force[1]

        return force

def main(args=None):
    rclpy.init(args=args)

    apf_node = APFNode()

    rclpy.spin(apf_node)

    apf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
