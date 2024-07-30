import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import logging

from msgs.msg import POS
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from msgs.srv import ARM, TAKEOFF, LAND
from msgs.action import MOVE

logging.getLogger('dronekit').setLevel(logging.CRITICAL)

class DroneNode1(Node):

    def __init__(self):
        super().__init__('drone_node1')

        # Service for takeoff
        self.takeoff_service = self.create_service(TAKEOFF, 'takeoff1', self.takeoff_callback)

        # Service for landing
        self.land_service = self.create_service(LAND, 'land1', self.land_callback)

        # Publisher for position
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        self.publisher = self.create_publisher(POS, 'position_topic1', qos_profile)

        # Action server for moving
        self._action_server = ActionServer(
            self,
            MOVE,
            'move1',  # Use the specific action name
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        # Connect to vehicle
        self.vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=115200, timeout=60)

        # Timer for publishing position
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_position)

        # Position message
        self.position = POS()
        self.drone_num = 1
        self.init_lat = self.vehicle.location.global_relative_frame.lat
        self.init_lon = self.vehicle.location.global_relative_frame.lon

        self.base_lat = 35.2266470
        self.base_lon = 126.8405244

    def takeoff(self, h):
        self.vehicle.mode = VehicleMode("STABILIZE")
        time.sleep(0.5)

        cmds = self.vehicle.commands
        cmds.download()
        cmds.wait_ready()
        cmds.clear()
        takeoff_cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                              mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, h)
        cmds.add(takeoff_cmd)
        cmds.upload()
        time.sleep(0.5)  # upload wait

        self.vehicle._master.mav.command_long_send(
            self.vehicle._master.target_system,
            self.vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0)

        time.sleep(3)
        print("ARMED : ", self.vehicle.armed)

        self.vehicle._master.mav.command_long_send(
            self.vehicle._master.target_system, self.vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_MISSION_START, 0,
            0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(2)

        print("Mission started")

        while True:
            print(f"Altitude: {self.vehicle.location.global_relative_frame.alt}")
            if self.vehicle.location.global_relative_frame.alt >= h * 0.8:
                print("Reached target altitude!!!!!!!!!!!!!!!!!!!!")
                break
            time.sleep(1)

        self.vehicle.mode = VehicleMode("GUIDED")

    def goto(self, x, y, z, speed=2.3):
        LATITUDE_CONVERSION = 111000
        LONGITUDE_CONVERSION = 88.649 * 1000

        target_lat = self.base_lat + (x / LATITUDE_CONVERSION)
        target_lon = self.base_lon - (y / LONGITUDE_CONVERSION)
        target_alt = z

        if self.vehicle.mode != VehicleMode("GUIDED"):
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(0.01)

        target_location = LocationGlobalRelative(target_lat, target_lon, target_alt)

        self.vehicle.groundspeed = speed
        self.vehicle.simple_goto(target_location)

    def land(self):
        print("Initiating landing sequence")
        self.vehicle._master.mav.command_long_send(
            self.vehicle._master.target_system, self.vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0, 0)

        while self.vehicle.location.global_relative_frame.alt > 0.3:
            print(f"Altitude: {self.vehicle.location.global_relative_frame.alt}")
            time.sleep(1)
        print("Landed successfully!!!!!!!!!!!!!!!!!!!!")

    def takeoff_callback(self, request, response):
        try:
            h = request.altitude
            self.takeoff(h)
            response.success = True
            response.message = f"Vehicle has taken off to {h} altitude"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def land_callback(self, request, response):
        try:
            self.land()
            response.success = True
            response.message = "Vehicle has landed successfully"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def goal_callback(self, goal_handle):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        feedback_msg = MOVE.Feedback()
        self.goto(goal_handle.request.x, goal_handle.request.y, goal_handle.request.z)
        goal_handle.succeed()
        result = MOVE.Result()
        result.success = True
        result.message = f"Vehicle moved to position ({goal_handle.request.x}, {goal_handle.request.y}, {goal_handle.request.z})"
        return result

    def publish_position(self):
        location = self.vehicle.location.global_relative_frame

        num = self.drone_num
        self.position.n = num
        self.position.x = location.lat
        self.position.y = location.lon
        self.position.z = location.alt
        self.publisher.publish(self.position)

    def close_connection(self):
        if self.vehicle:
            self.vehicle.close()
            self.vehicle = None
            print("Vehicle connection closed.")

def main(args=None):
    rclpy.init(args=args)
    drone1 = DroneNode1()
    try:
        rclpy.spin(drone1)
    finally:
        drone1.close_connection()
        drone1.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
