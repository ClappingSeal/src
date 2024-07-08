import rclpy
from rclpy.node import Node
from msgs.srv import APF

class APFNode(Node):
    def __init__(self):
        super().__init__('apf_node')
        self.srv = self.create_service(APF, 'apf1', self.apf_callback)

    def apf_callback(self, request, response):
        start = request.start
        goal = request.goal
        x = request.x
        y = request.y
        obstacles = list(zip(x, y))

        force_vector = self.apf(start, goal, obstacles)
        response.force = force_vector
        return response

    def apf(self, start, goal, obstacles):
        start = [123, 321]
        goal= [123, 123]
        obstacles = [[1, 2], [2, 5]]
        
        force = [123, 123]
        return force

def main(args=None):
    rclpy.init(args=args)
    apf_node = APFNode()
    rclpy.spin(apf_node)
    apf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
