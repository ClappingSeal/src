import rclpy
from rclpy.node import Node
from msgs.srv import APF
import sys

class APFClientNode(Node):
    def __init__(self):
        super().__init__('apf_client')
        self.client = self.create_client(CalculateForce, 'calculate_force_vector')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = CalculateForce.Request()

    def send_request(self, goal):
        self.request.start = start
        self.request.goal = goal
        self.request.obstacles = obstacles
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().force

def main(args=None):
    rclpy.init(args=args)
    apf_client = APFClient()
    start = [float(x) for x in sys.argv[1].strip('[]').split(',')]
    goal = [float(x) for x in sys.argv[2].strip('[]').split(',')]
    obstacles = [[float(y) for y in x.strip('[]').split(',')] for x in sys.argv[3:]]

    response = apf_client.send_request(start, goal, obstacles)
    print(f'Response: {response}')

    apf_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
