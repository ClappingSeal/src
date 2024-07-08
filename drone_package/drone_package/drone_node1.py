import rclpy
from rclpy.node import Node
from dronekit import connect, VehicleMode, APIException, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math
import logging

from msgs.msg import POS
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from msgs.srv import ARM, TAKEOFF, LAND, GOTO

logging.getLogger('dronekit').setLevel(logging.CRITICAL)

class Drone_node1(Node):

    def __init__(self):
        super().__init__('drone_node1')

        # Service for takeoff
        self.takeoff_service = self.create_service(TAKEOFF, 'takeoff1', self.takeoff_callback)

        # Service for landing
        self.land_service = self.create_service(LAND, 'land1', self.land_callback)

        # Service for moving 1
        self.goto_service = self.create_service(GOTO, 'goto1', self.goto_callback)
        # self.goto_service = self.create_service(GOTO, 'goto1', self.goto_block_callback)

        # Publisher for position
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
        self.publisher = self.create_publisher(POS, 'position_topic1', qos_profile)

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

        # # Check position
        # if self.init_lat is None or self.init_lon is None:
        #     raise ValueError("Latitude or Longitude value is None. Class initialization aborted.")
        # print("Drone current location : ", self.init_lat, "lat, ", self.init_lon, "lon")
        
        # if self.init_lat == 0 or self.init_lon == 0:
        #     raise ValueError("Cannot get Location. Class initialization aborted.")

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

    def goto(self, x, y, z, speed=10):
        LATITUDE_CONVERSION = 111000
        LONGITUDE_CONVERSION = 88.649 * 1000

        target_lat = self.init_lat + (x / LATITUDE_CONVERSION)
        target_lon = self.init_lon - (y / LONGITUDE_CONVERSION)
        target_alt = z

        if self.vehicle.mode != VehicleMode("GUIDED"):
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(0.1)

        target_location = LocationGlobalRelative(target_lat, target_lon, target_alt)

        self.vehicle.groundspeed = speed
        self.vehicle.simple_goto(target_location)

        # print(f"Moving to: Lat: {target_lat}, Lon: {target_lon}, Alt: {target_alt} at {speed} m/s")

    def goto_block(self, x, y, z):
        LATITUDE_CONVERSION = 111000
        LONGITUDE_CONVERSION = 88.649 * 1000

        print(self.init_lat, y, LONGITUDE_CONVERSION)

        target_lat = self.init_lat + (x / LATITUDE_CONVERSION)
        target_lon = self.init_lon - (y / LONGITUDE_CONVERSION)
        target_alt = z

        def get_distance(lat1, lon1, lat2, lon2):
            import math
            R = 6371000  # Earth radius in meters

            d_lat = math.radians(lat2 - lat1)
            d_lon = math.radians(lon2 - lon1)

            a = (math.sin(d_lat / 2) * math.sin(d_lat / 2) +
                 math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
                 math.sin(d_lon / 2) * math.sin(d_lon / 2))
            c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

            return R * c

        if self.vehicle.mode != VehicleMode("GUIDED"):
            self.vehicle.mode = VehicleMode("GUIDED")
            time.sleep(0.1)

        target_location = LocationGlobalRelative(target_lat, target_lon, target_alt)
        self.vehicle.simple_goto(target_location)
        print(f"Moving to: Lat: {target_lat}, Lon: {target_lon}, Alt: {target_alt}")

        while True:
            current_location = self.vehicle.location.global_relative_frame
            distance_to_target = get_distance(current_location.lat, current_location.lon, target_lat, target_lon)
            alt_diff = abs(current_location.alt - target_alt)
            print("current pos : ", self.get_pos())

            if distance_to_target < 1 and alt_diff < 1:
                print("Arrived at target location!!!!!!!!!!!!!!!!!!!!!")
                break
            time.sleep(0.5)

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

    def goto_callback(self, request, response):
        try:
            self.goto(request.x, request.y, request.z)
            response.success = True
            response.message = f"Vehicle is moving to position ({request.x}, {request.y}, {request.z})"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def goto_block_callback(self, request, response):
        try:
            self.goto_block(request.x, request.y, request.z)
            response.success = True
            response.message = f"Vehicle is moving to position ({request.x}, {request.y}, {request.z})"
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
    drone1 = Drone_node1()
    try:
        rclpy.spin(drone1)
    finally:
        drone1.close_connection()
        drone1.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
