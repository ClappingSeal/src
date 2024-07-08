import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import sys

class APFClient(Node):
    def __init__(self):
        super().__init__('apf_client1')
        self.client = self.create_client(Float64MultiArray, 'calculate_force_vector')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = Float64MultiArray()

    def send_request(self, start, goal, obstacles):
        self.request.data = start + goal + [item for sublist in obstacles for item in sublist]
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().data

def main(args=None):
    rclpy.init(args=args)

    apf_client = APFClient()

    start = [float(x) for x in sys.argv[1].strip('[]').split(',')]
    goal = [float(x) for x in sys.argv[2].strip('[]').split(',')]
    obstacles = [list(map(float, ob.strip('[]').split(','))) for ob in sys.argv[3:]]

    response = apf_client.send_request(start, goal, obstacles)
    print(f'Response: {response}')

    apf_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
