import rclpy
from rclpy.node import Node
from msgs.msg import POS
from std_msgs.msg import Float64MultiArray
from datetime import datetime
import csv


class PosSubscriber(Node):
    def __init__(self):
        super().__init__('pos_sub')

        self.pub = self.create_publisher(Float64MultiArray, 'drone_info', 10)
        self.current_positions = {}  # 현재 위치 저장
        self.recent_positions = {1: [], 2: []}  # 최근 3개 위치 저장 (드론 추가 시 여기에 추가)

        self.subscription1 = self.create_subscription(
            POS,
            'position_topic1',
            lambda msg: self.listener_callback(msg, 1),
            10)
        self.subscription2 = self.create_subscription(
            POS,
            'position_topic2',
            lambda msg: self.listener_callback(msg, 2),
            10)
        # 드론 추가 시 여기에 subscription 추가

    def listener_callback(self, msg, drone_id):
        LATITUDE_CONVERSION = 111000
        LONGITUDE_CONVERSION = 88.649 * 1000

        base_lat = 35.2266470
        base_lon = 126.8405244

        x = (msg.x - base_lat) * LATITUDE_CONVERSION
        y = (base_lon - msg.y) * LONGITUDE_CONVERSION
        z = msg.z

        # 현재 위치 업데이트
        self.current_positions[drone_id] = (x, y, z)

        # 최근 위치 업데이트
        self.update_recent_positions(drone_id, (x, y, z))

        # 데이터 출력
        print(f"Drone {drone_id} Current Position: {x}, {y}, {z}")

    def update_recent_positions(self, drone_id, position):
        if drone_id in self.recent_positions:
            self.recent_positions[drone_id].append(position)
            # 최근 3개 위치만 유지
            if len(self.recent_positions[drone_id]) > 3:
                self.recent_positions[drone_id].pop(0)

    def print_recent_positions(self):
        for drone_id, positions in self.recent_positions.items():
            print(f"Drone {drone_id} Recent Positions:")
            for pos in positions:
                print(pos)


def main(args=None):
    rclpy.init(args=args)
    pos_node = PosSubscriber()
    try:
        rclpy.spin(pos_node)
    except KeyboardInterrupt:
        # 드론 위치 데이터 저장 (현재 위치만)
        keys = ['Drone_ID', 'X', 'Y', 'Z']
        with open('current_drone_positions.csv', 'w', newline='') as output_file:
            dict_writer = csv.DictWriter(output_file, fieldnames=keys)
            dict_writer.writeheader()
            for drone_id, position in pos_node.current_positions.items():
                dict_writer.writerow({'Drone_ID': drone_id, 'X': position[0], 'Y': position[1], 'Z': position[2]})
        print("Current positions saved to 'current_drone_positions.csv'")
        # 최근 위치 출력
        pos_node.print_recent_positions()
    finally:
        pos_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
