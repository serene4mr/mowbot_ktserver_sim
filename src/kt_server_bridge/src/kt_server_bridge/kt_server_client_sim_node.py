import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import NavSatFix, Imu, NavSatStatus
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from enum import Enum
from scipy.spatial.transform import Rotation as R

from kt_server_bridge.kt_server_client import KTServerClient

from kt_server_bridge.schemas.robot_status_schema import (
    DrivingStatus, 
    AutonomousStatus,
    RTKStatus
)
from kt_server_bridge.schemas.common_schema import TaskStatus   


# DEMO_PHASES = {
#     "pre_manual": {
#         "latitude": 36.114005953987785,
#         "longitude": 128.41844551680896
#     },
#     "manual_started": {
#         "latitude": 36.11400615610248,
#         "longitude": 128.41844355130064
#     },
#     "manual_stopped": {
#         "latitude": 36.11403369504778,
#         "longitude": 128.41834785466844
#     },
#     "auto_started": {
#         "latitude": 36.11403524490441,
#         "longitude": 128.4183341161192
#     },
#     "auto_stopped": {
#         "latitude": 36.114037052546394,
#         "longitude": 128.41826355116552
#     }
# }

# DEMO_FIELD_BOUNDARY = [
#     {"lat": 36.114035673055845, "lon": 128.41830360303496},
#     {"lat": 36.1140370525463946, "lon": 128.41826355116552},
# ]

class DemoPhases(Enum):
    PRE_MANUAL = "pre_manual"
    MANUAL_STARTED = "manual_started"
    MANUAL_STOPPED = "manual_stopped"
    AUTO_STARTED = "auto_started"
    AUTO_STOPPED = "auto_stopped"

class KtServerClientSimNode(Node):
    
    def __init__(self):
        super().__init__('kt_server_client_sim_node')
        
        # Declare parameters
        self.declare_parameter("robot_serial", "RSN800-0000")
        self.declare_parameter("client_id", "snsolutions")
        self.declare_parameter("client_secret", "QgvVztMwFggaC3Ds1nDCgIXec9P5Ca84")
        self.declare_parameter("report_hz", 1.0)
        self.declare_parameter("verbose", True)
        
        
        # Field boundary
        self.declare_parameter("fb_min_lat", 36.114035673055845)
        self.declare_parameter("fb_min_lon", 128.41830360303496)
        self.declare_parameter("fb_max_lat", 36.1140370525463946)
        self.declare_parameter("fb_max_lon", 128.41826355116552)
        
        
        # Get parameters
        self.robot_serial = self.get_parameter("robot_serial").get_parameter_value().string_value
        self.client_id = self.get_parameter("client_id").get_parameter_value().string_value
        self.client_secret = self.get_parameter("client_secret").get_parameter_value().string_value
        self.report_hz = self.get_parameter("report_hz").get_parameter_value().double_value
        self.verbose = self.get_parameter("verbose").get_parameter_value().bool_value

        self.field_boundary = [
            {"lat": self.get_parameter("fb_min_lat").get_parameter_value().double_value, "lon": self.get_parameter("fb_min_lon").get_parameter_value().double_value},
            {"lat": self.get_parameter("fb_max_lat").get_parameter_value().double_value, "lon": self.get_parameter("fb_max_lon").get_parameter_value().double_value}
        ]
        
        self.get_logger().info(f"field_boundary: {self.field_boundary}")

        # Create kt_server_client
        self.kt_server_client = KTServerClient(
            robot_serial=self.robot_serial,
            client_id=self.client_id,
            client_secret=self.client_secret,
            logger=self.get_logger()
        )
        
        # Report timer
        self.report_timer = self.create_timer(
            1.0 / float(self.report_hz), 
            self.on_report_timer_callback
        )

        # Subscriptions
        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/gps/fix", self.on_gps_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom", self.on_odom_callback, 10
        )
        self.phase_sub = self.create_subscription(
            String,
            "/demo_phase", self.on_phase_callback, 10
        )
        
        self._last_gps = None
        self._last_heading = None
        self._last_speed = None
        self._last_rtk_status: RTKStatus = RTKStatus.NORMAL
        
        self._last_drive_status: DrivingStatus = DrivingStatus.STANDBY
        self._last_autonomous_status: AutonomousStatus = AutonomousStatus.STOPPED
        
        self._last_task_status: TaskStatus = TaskStatus.STANDBY
        
        self._last_field_boundary = []
        self.current_phase: DemoPhases = None
        
        # assume mission and task already created
        self._last_field_boundary = self.field_boundary
        self.kt_server_client.demo_start_new_mission_and_task()
        
    def on_report_timer_callback(self):
        # if self.verbose:
        #     self.get_logger().info("Sending robot status...")
        
        # Send robot status
        self._send_robot_status()
        self._send_mission_info()

    def on_gps_callback(self, msg: NavSatFix):
        # if self.verbose:
        #     self.get_logger().info(f"Received GPS: {msg.latitude}, {msg.longitude}, {msg.altitude}")
        
        self._last_gps = {"lat": msg.latitude, "lon": msg.longitude}
        
    
    def on_phase_callback(self, msg: String):
        if self.verbose:
            self.get_logger().info(f"Received phase: {msg.data}")
        
        if self.current_phase == msg.data:
            # No change in phase
            return
        else:
            self.current_phase = msg.data
        
        if self.current_phase == DemoPhases.PRE_MANUAL.value:
            
            self.get_logger().info("Demo phase: pre-manual")
            
            # initial state when starting demo
            self._last_drive_status = DrivingStatus.STANDBY
            self._last_task_status = TaskStatus.STANDBY
            self._last_autonomous_status = AutonomousStatus.STANDBY    
            
            self._send_robot_status()
            self._send_mission_info()

            
        elif self.current_phase == DemoPhases.MANUAL_STARTED.value:
            
            self.get_logger().info("Demo phase: manual-started")
            
            # start manual driving
            self._last_drive_status = DrivingStatus.MANUAL_DRIVING
            self._last_task_status = TaskStatus.STANDBY
            self._last_autonomous_status = AutonomousStatus.STOPPED
            self._send_robot_status()
            
        elif self.current_phase == DemoPhases.MANUAL_STOPPED.value:
            
            self.get_logger().info("Demo phase: manual-stopped")
            
            # conclude manual driving back to previous driving status and task status
            self._last_drive_status = DrivingStatus.STANDBY
            self._last_task_status = TaskStatus.STANDBY
            self._last_autonomous_status = AutonomousStatus.STANDBY
            self._send_robot_status()
            self._send_mission_info()
            
            
        elif self.current_phase == DemoPhases.AUTO_STARTED.value:
            
            self.get_logger().info("Demo phase: auto-started")
            
            # start autonomous driving
            self._last_drive_status = DrivingStatus.STANDBY
            self._last_task_status = TaskStatus.STARTED
            self._last_autonomous_status = AutonomousStatus.STANDBY
            self._send_robot_status()
            self._send_mission_info()
            
            # on-going autonomous driving
            self._last_drive_status = DrivingStatus.NORMAL_DRIVING
            self._last_task_status = TaskStatus.ON_PROGRESS
            self._last_autonomous_status = AutonomousStatus.ACTIVE
            self._send_robot_status()
            self._send_mission_info()
            
            
        elif self.current_phase == DemoPhases.AUTO_STOPPED.value:
            
            self.get_logger().info("Demo phase: auto-stopped")
            
            # reach destination of autonomous driving
            self._last_drive_status = DrivingStatus.NORMAL_DRIVING_COMPLETION
            self._last_task_status = TaskStatus.DEST_ARRIVED
            self._last_autonomous_status = AutonomousStatus.ACTIVE
            self._send_robot_status()
            self._send_mission_info()
            
            # conclude task
            self._last_drive_status = DrivingStatus.NORMAL_DRIVING_COMPLETION
            self._last_task_status = TaskStatus.END
            self._last_autonomous_status = AutonomousStatus.STOPPED
            self._send_robot_status()
            self._send_mission_info()
            
            # reset to end demo
            self._last_drive_status = DrivingStatus.STANDBY
            self._last_task_status = TaskStatus.STANDBY
            self._last_autonomous_status = AutonomousStatus.STANDBY
            
            # assume new mission and task created again at the end of demo
            self._last_field_boundary = self.field_boundary
            self.kt_server_client.demo_start_new_mission_and_task()
            
            self._send_robot_status()
            self._send_mission_info()
            
            
    def _check_data_validity(self):
        if self._last_gps is None or \
           self._last_heading is None or \
           self._last_speed is None:
            if self.verbose:
                self.get_logger().warn("Data not valid, skipping sending status.")
            return False
        return True
            
    def _send_robot_status(self):
        
        if not self._check_data_validity():
            return
        
        self.kt_server_client.send_robot_status(
            task_id = "01",
            gps_location = {"lat": self._last_gps["lat"], "lon": self._last_gps["lon"]},
            speed = self._last_speed,
            heading = self._last_heading,
            rtk_status = self._last_rtk_status,
            autonomous_status = self._last_autonomous_status,
            drive_status = self._last_drive_status,
            task_status= self._last_task_status
        )
        
    def _send_mission_info(self):
        if not self._check_data_validity():
            return
        
        self.kt_server_client.send_mission_info(
            task_id="01",
            task_status = self._last_task_status,
            field_boundary = self._last_field_boundary
        )
    
    
            
            
    def on_odom_callback(self, msg: Odometry):
        self._last_speed = msg.twist.twist.linear.x
        self._last_heading = R.from_quat([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]).as_euler('xyz', degrees=True)[2]
        # self.get_logger().info(
        #     f"Received Odometry: Speed={round(self._last_speed, 2)}, Heading={round(self._last_heading, 2)}"
        # )
        
        
        
            
def main(args=None):
    rclpy.init(args=args)
    
    node = KtServerClientSimNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

    