import rclpy
from rclpy.node import Node
from msgs.srv import TD3
from msgs.srv import GOTO
from msgs.srv import LAND
import argparse
import math
import time


class TD3ClientNode(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('td3_client2')
        self.cli_td3 = self.create_client(TD3, 'td32')
        while not self.cli_td3.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for TD3 service to become available...')

        self.cli_goto = self.create_client(GOTO, 'goto2')
        while not self.cli_goto.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for GOTO service to become available...')

        self.cli_land = self.create_client(LAND, 'land2')
        while not self.cli_land.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for LAND service to become available...')

        self.req = TD3.Request()
        self.send_request([goal_x, goal_y])

    def send_request(self, goal):
        self.req.goal = goal
        future = self.cli_td3.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            path = list(future.result().path)
            print(path)

            goto_request = GOTO.Request()
            goto_request.x = float(path[0])
            goto_request.y = float(path[1])
            goto_request.z = float(5) # 높이 세팅 값
            future_goto = self.cli_goto.call_async(goto_request)
            rclpy.spin_until_future_complete(self, future_goto)

            if future_goto.result() is not None:
                self.get_logger().info(f'Result of GOTO: {future_goto.result().message}')

                # Check distance to target
                distance = math.sqrt((goal[0] - path[0]) ** 2 + (goal[1] - path[1]) ** 2)
                threshold_distance = 0.5  # 골 세팅 값

                if distance <= threshold_distance:
                    time.sleep(1)
                    land_request = LAND.Request()
                    future_land = self.cli_land.call_async(land_request)
                    rclpy.spin_until_future_complete(self, future_land)
                    time.sleep(1)

                    if future_land.result() is not None:
                        self.get_logger().info(f'Result of LAND: {future_land.result().message}')

                    while True:
                        self.get_logger().info('Mission Complete')
                        time.sleep(5000)

            else:
                self.get_logger().error('Exception while calling GOTO service')

        else:
            self.get_logger().error('Exception while calling TD3 service')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('goal_x', type=float, help='X coordinate of goal')
    parser.add_argument('goal_y', type=float, help='Y coordinate of goal')
    args = parser.parse_args()

    rclpy.init(args=None)
    client_node = TD3ClientNode(args.goal_x, args.goal_y)
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
