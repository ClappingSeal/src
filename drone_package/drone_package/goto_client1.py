import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from msgs.action import GOTO

class Goto_client1(Node):

    def __init__(self):
        super().__init__('goto_client1')
        self.cli = self.create_client(GOTO, 'goto1')
        self.current_future = None
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
    
    def cancel_current_request(self):
        if self.current_future:
            self.current_future.cancel()
            self.current_future = None
            self.get_logger().info('Cancelled the current request.')

    def send_request(self, x, y, z):
        self.cancel_current_request()  # 기존 경로 취소

        request = GOTO.Request()
        request.x = x
        request.y = y
        request.z = z
        self.current_future = self.cli.call_async(request)

    def check_futures(self):
        if self.current_future and self.current_future.done():
            if self.current_future.result() is not None:
                self.get_logger().info(f'Result: {self.current_future.result().message}')
            else:
                self.get_logger().error('Service call failed')
            self.current_future = None

def main(args=None):
    rclpy.init(args=args)

    args_without_ros = rclpy.utilities.remove_ros_args(args)

    if len(args_without_ros) != 4:
        print("Usage: ros2 run arm_pos_package goto_client <x> <y> <z>")
        return

    x = float(args_without_ros[1])
    y = float(args_without_ros[2])
    z = float(args_without_ros[3])

    goto1 = Goto_client1()
    goto1.send_request(x, y, z)

    while rclpy.ok():
        goto1.check_futures()
        rclpy.spin_once(goto1, timeout_sec=0.1)

    goto1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
