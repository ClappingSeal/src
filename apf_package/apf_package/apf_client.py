import rclpy
from rclpy.node import Node
from msgs.srv import STATE
import sys

class APFClient(Node):
    def __init__(self):
        super().__init__('apf_client')
        self.client = self.create_client(STATE, 'apf')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = STATE.Request()

    def send_request(self, start, goal, x, y):
        self.request.start = start
        self.request.goal = goal
        self.request.x = x
        self.request.y = y
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().force

def main(args=None):
    rclpy.init(args=args)

    apf_client = APFClient()

    start = [float(coord) for coord in sys.argv[1].strip('[]').split(',')]
    goal = [float(coord) for coord in sys.argv[2].strip('[]').split(',')]
    obstacles = [[float(coord) for coord in ob.strip('[]').split(',')] for ob in sys.argv[3:]]

    x = [ob[0] for ob in obstacles]
    y = [ob[1] for ob in obstacles]

    response = apf_client.send_request(start, goal, x, y)
    print(f'Response: {response}')

    apf_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
