import rclpy
from rclpy.node import Node
from msgs.srv import STATE

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
        start = [123, 321]
        goal= [123, 123]
        obstacles = [[1, 2], [2, 5]]
        
        force = [123.0, 123.0]
        return force

def main(args=None):
    rclpy.init(args=args)
    apf_node = APFNode()
    rclpy.spin(apf_node)
    apf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
