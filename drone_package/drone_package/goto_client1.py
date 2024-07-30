import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from msgs.action import GOTO

class GotoClient1(Node):

    def __init__(self):
        super().__init__('goto_client1')
        self._action_client = ActionClient(self, GOTO, 'goto1')

    def send_goal(self, x, y, z):
        goal_msg = GOTO.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.z = z

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback}')

def main(args=None):
    rclpy.init(args=args)

    args_without_ros = rclpy.utilities.remove_ros_args(args)

    if len(args_without_ros) != 4:
        print("Usage: ros2 run arm_pos_package goto_client1 <x> <y> <z>")
        return

    x = float(args_without_ros[1])
    y = float(args_without_ros[2])
    z = float(args_without_ros[3])

    goto_client1 = GotoClient1()
    goto_client1.send_goal(x, y, z)

    rclpy.spin(goto_client1)

    goto_client1.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
