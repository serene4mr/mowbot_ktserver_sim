import time
from datetime import datetime, timedelta, timezone
import requests
import pyproj
from typing import Literal
from enum import Enum

KT_SERVER_TOKEN_URL = "https://biz-dev.ktraas.kt.co.kr/keycloak/realms/openrm/protocol/openid-connect/token"
KT_SERVER_API_BASE_URL = "https://biz-dev.ktraas.kt.co.kr/kt-pa-open-api/api"

class DrivingStatus(Enum):
    STANDBY = 0
    NORMAL_DRIVING = 1
    NORMAL_DRIVING_COMPLETION = 2
    CANCELLED = 3
    OBSTACLE_DETECTED = 4
    DRIVING_FAILURE = 5
    ERROR_STOP = 12
    MANUAL_DRIVING = 16
    

def get_utm_coordinates(latitude, longitude):
    # Define the source (WGS84) and target (UTM) coordinate systems
    wgs84 = pyproj.CRS("EPSG:4326")  # WGS84 coordinate system
    crs = pyproj.CRS("EPSG:32652")  # UTM Zone 52N coordinate system
    # Create a transformer for the conversion
    transformer = pyproj.Transformer.from_crs(wgs84, crs, always_xy=True)
    utm_easting, utm_northing = transformer.transform(longitude, latitude)
    return utm_easting, utm_northing

class KTServerClient:
    def __init__(
        self,
        robot_serial: str,
        client_id: str,
        client_secret: str,
        logger=None,  # Optionally pass a logger (e.g., ROS2 node's logger)
        verbose: bool = False
    ):
        if not robot_serial:
            raise ValueError("robot_serial is required")
        if not client_id:
            raise ValueError("client_id is required")
        if not client_secret:
            raise ValueError("client_secret is required")
        
        self.robot_serial = robot_serial
        self.client_id = client_id
        self.client_secret = client_secret
        self.logger = logger
        
        self.verbose = verbose  

        self.access_token = None
        self.token_expiry_time = 0  # Unix timestamp
        
        self.robot_status_endpoint = f"{KT_SERVER_API_BASE_URL}/robots/{self.robot_serial}/robot-status"
        self.service_status_endpoint = f"{KT_SERVER_API_BASE_URL}/robots/{self.robot_serial}/service-status"
        self.error_status_endpoint = f"{KT_SERVER_API_BASE_URL}/robots/{self.robot_serial}/error-status"
        

    def log_info(self, msg):
        if self.logger:
            self.logger.info(msg)
        else:
            print(f"[INFO] {msg}")

    def log_error(self, msg):
        if self.logger:
            self.logger.error(msg)
        else:
            print(f"[ERROR] {msg}")

    def get_access_token(self):
        try:
            response = requests.post(
                url=KT_SERVER_TOKEN_URL,
                data={
                    "grant_type": "client_credentials",
                    "client_id": self.client_id,
                    "client_secret": self.client_secret
                },
                headers={
                    "Content-Type": "application/x-www-form-urlencoded"
                }
            )
            response.raise_for_status()
            token_data = response.json()
            self.access_token = token_data.get("access_token")
            expires_in = token_data.get("expires_in", 0)
            self.token_expiry_time = time.time() + expires_in
            self.log_info(f"Obtained access token (expires in {expires_in} seconds).")
        except Exception as e:
            self.log_error(f"Error getting token: {e}")
            # Optionally, set expiry to now to retry soon
            self.token_expiry_time = time.time() + 1

    def is_token_valid(self, buffer_seconds: int = 30) -> bool:
        """Check if the token is still valid for at least buffer_seconds."""
        return (
            self.access_token is not None and
            (self.token_expiry_time - time.time()) > buffer_seconds
        )

    def check_and_refresh_token(self):
        """Refresh the access token if it's about to expire."""
        if not self.is_token_valid():
            self.get_access_token()
            
    def get_current_time_kst(self):
        """Get the current time in KST format."""
        # Get current UTC time as a timezone-aware object
        now_utc = datetime.now(timezone.utc)
        kst_timezone = timezone(timedelta(hours=9)) # Define KST timezone
        now_kst = now_utc.astimezone(kst_timezone) # Convert to KST
        return now_kst.strftime("%y%m%d%H%M%S%f")[:15] # Format as yyMMddHHmmssfff
            
    def send_robot_status(
        self,
        gps_location: dict,
        speed: float,
        heading: float,
        rtk_status: Literal["normal", "error"] = "normal",
        autonomous_status: Literal["Active", "Stopped", "Standby"] = "Stopped",
        drive_status: Literal[0, 1, 2, 3, 4, 5, 12, 16] = 0,
    ):  
        self.check_and_refresh_token()  # Ensure token is valid before sending status
        
        utm_easting, utm_northing = get_utm_coordinates(gps_location["lat"], gps_location["lon"])
        time_now = self.get_current_time_kst()
        try: 
            response = requests.post(
                self.robot_status_endpoint,  # Fixed URL with robot serial
                headers= {
                    "Authorization": f"Bearer {self.access_token}",
                    "Content-Type": "application/json"
                },
                json={
                    "robot_serial": self.robot_serial,
                    "create_time": time_now,  # Current time in the required (KST)format
                    "x": utm_easting,
                    "y": utm_northing,
                    "battery": 10, # fixed
                    "drive_status": drive_status, # Please refer to Excel and update it.(0,1, 2, 3, 4, 5, 12, 16)
                    "speed": int(speed), # Exclude decimal point in the result value.
                    "heading": int(heading), # Exclude decimal point in the result value.
                    "charge": False,
                    "charge_type": "None",
                    "is_indoor": False,
                    "coord_code": "WGS84",
                    "service_mode": "mowing",
                    "service": {
                        "mowing": {
                            "fuel": 10, # Add fuel sensors later.
                            "autonomous_status": autonomous_status, # need an update on these 3 driving conditions.("Stopped": 사용불가(정지)"Standby": 준비완료(대기)"Active": 자율작업중(작업중))
                            "cutter": {
                                "type": "rotary",
                                "width": 80
                            },
                            "lift": {
                                "status": "down",
                                "height": 5
                            },
                            "rtk": {
                                "status": rtk_status, # Change "error" when there is no rtk signal
                                "error_range": 3
                            },
                            "rollangle": 30, # Enter the angle data of the IMU in real time.
                            "rollover_status": "normal" # Change the "normal" to "error" with printable imu data when the robot is overturned
                        }
                    },
                    "task": {
                        "task_id": f"{self.robot_serial}-{time_now}0101",
                        "task_code": "mowing",
                        "task_status": "OnProgress" # Please refer to Excel and update it.(Standby, Started, DestArrived, OnProgress, End, Cancelled, Failed)
                    }
                }
            )
            
            if response.status_code == 200:
                if self.verbose:
                    self.log_info(f"Robot status sent successfully: {response.json()}")
            else:
                if self.verbose:
                    self.log_error(f"Failed to send robot status: {response.status_code} - {response.text}")
                return False
        
        except Exception as e:  
            if self.verbose:
                self.log_error(f"Error sending robot status: {e}")
            return False
        
        return True


def main():
    # Example usage
    robot_serial = "your_robot_serial"
    client_id = "your_client_id"
    client_secret = "your_client_secret"

    kt_client = KTServerClient(robot_serial, client_id, client_secret)
    
    # Check and refresh token if needed
    kt_client.check_and_refresh_token()
    
    # Now you can use the access token for API calls
    print(f"Access Token: {kt_client.access_token}")
    
if __name__ == "__main__":
    main()