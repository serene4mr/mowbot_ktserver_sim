import rclpy
from rclpy.node import Node

from kt_server_bridge.kt_server_client import KTServerClient, DrivingStatus

from sensor_msgs.msg import NavSatFix, Imu, NavSatStatus
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R

class KTServerClientNode(Node):
    def __init__(self):
        super().__init__('kt_server_client')
        
        # Declare parameters
        self.declare_parameter("robot_serial", "")
        self.declare_parameter("client_id", "")
        self.declare_parameter("client_secret", "")
        self.declare_parameter("robot_status_report_hz", 1.0)
        self.declare_parameter("verbose", False)
        
        # Get parameters
        self.robot_serial = self.get_parameter("robot_serial").get_parameter_value().string_value
        self.client_id = self.get_parameter("client_id").get_parameter_value().string_value
        self.client_secret = self.get_parameter("client_secret").get_parameter_value().string_value
        self.robot_status_report_hz = self.get_parameter("robot_status_report_hz").get_parameter_value().double_value
        self.verbose = self.get_parameter("verbose").get_parameter_value().bool_value
        
        if self.robot_status_report_hz <= 0:
            self.get_logger().warn("robot_status_report_hz must be > 0. Setting to 1.")
            self.robot_status_report_hz = 1
        
        if self.verbose:
            self.get_logger().info(
                f"robot_serial: {self.robot_serial}\n"
                f"client_id: {self.client_id}\n"
                f"client_secret: {self.client_secret}"
            )
        
        # Create kt_server_client
        self.kt_server_client = KTServerClient(
            robot_serial=self.robot_serial,
            client_id=self.client_id,
            client_secret=self.client_secret,
            logger=self.get_logger()
        )
        
        # Report timer
        self.robot_status_report_timer = self.create_timer(
            1.0 / float(self.robot_status_report_hz), 
            self.on_robot_status_report_timer_callback
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
        self.heading_sub = self.create_subscription(
            Imu,
            "/heading", self.on_heading_callback, 10
        )
        
        # Timer for checking autonomous status
        self.autonomous_status_timer = self.create_timer(
            1.0, self.on_autonomous_status_check_timer_callback
        )
        
        # Timer for checking drive status
        self.drive_status_timer = self.create_timer(
            1.0, self.on_drive_status_check_timer_callback
        )
        
        self.last_gps = None
        self.last_speed = None
        self.last_heading = None
        self.last_rtk_status = None
        
        self.last_autonomous_status = "Stopped"
        self.drive_status = DrivingStatus.STANDBY
    
    def _is_node_available(self, target_node_name):
        # Get list of node names (without namespaces)
        node_names = [name for name, _ in self.get_node_names_and_namespaces()]
        return target_node_name in node_names
    
    def on_autonomous_status_check_timer_callback(self):
        # Check for required nodes
        if (self._is_node_available("ekf_filter_node_map") and
            self._is_node_available("ekf_filter_node_odom") and
            self._is_node_available("navsat_transform")):
            self.last_autonomous_status = "Standby"
            if self._is_node_available("gui_gps_waypoint_follower"):
                self.last_autonomous_status = "Active"
        else:
            self.last_autonomous_status = "Stopped"
        
        if self.verbose:
            self.get_logger().info(f"Autonomous status: {self.last_autonomous_status}")
    
    def on_drive_status_check_timer_callback(self):
        # check for required nodes
        # TODO: fix later
        if (self._is_node_available("ekf_filter_node_map") and
            self._is_node_available("ekf_filter_node_odom") and
            self._is_node_available("navsat_transform")):
            if self._is_node_available("gui_gps_waypoint_follower"):
                self.drive_status = DrivingStatus.NORMAL_DRIVING
            else:
                self.drive_status = DrivingStatus.STANDBY 
        else:
            self.drive_status = DrivingStatus.MANUAL_DRIVING
        
        if self.verbose:
            self.get_logger().info(f"Drive status: {self.drive_status}")
        
    
    def on_gps_callback(self, msg: NavSatFix):
        self.last_gps = {
            "lat": msg.latitude,
            "lon": msg.longitude,
        }
        # Get RTK status
        if msg.status.status == NavSatStatus.STATUS_GBAS_FIX:
            self.last_rtk_status = "normal"
        else:
            self.last_rtk_status = "error"
            
        if self.verbose:
            self.get_logger().info(f"GPS Location: {self.last_gps}")
    
    def on_odom_callback(self, msg):
        self.last_speed = msg.twist.twist.linear.x
        if self.verbose:
            self.get_logger().info(f"Speed: {self.last_speed}")
    
    def on_heading_callback(self, msg):
        quat = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        euler = R.from_quat(quat).as_euler('xyz', degrees=True)
        yaw_enu = euler[2]
        yaw_ned = (90 - yaw_enu) % 360
        self.last_heading = yaw_ned
        if self.verbose:
            self.get_logger().info(f"Heading: {self.last_heading}")
    
    def on_robot_status_report_timer_callback(self):
        if (self.last_gps is None or
            self.last_speed is None or
            self.last_heading is None or
            self.last_rtk_status is None):
            self.get_logger().warn("Robot status not ready yet.")
            return
        
        self.kt_server_client.send_robot_status(
            gps_location=self.last_gps,
            speed=self.last_speed,
            heading=self.last_heading,
            rtk_status=self.last_rtk_status,
            autonomous_status=self.last_autonomous_status,
        )
    
        # Reset last status
        self.last_gps = None
        self.last_speed = None
        self.last_heading = None
        self.last_rtk_status = None
        
def main(args=None):
    rclpy.init(args=args)
    node = KTServerClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()
