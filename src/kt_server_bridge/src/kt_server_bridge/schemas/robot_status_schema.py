from pydantic import BaseModel
from typing import Optional, Literal
from enum import Enum

from .common_schema import Cutter, Lift, LiftStatus, TaskStatus

class DrivingStatus(Enum):
    STANDBY = 0
    NORMAL_DRIVING = 1
    NORMAL_DRIVING_COMPLETION = 2
    CANCELLED = 3
    OBSTACLE_DETECTED = 4
    DRIVING_FAILURE = 5
    ERROR_STOP = 12
    MANUAL_DRIVING = 16
    
class AutonomousStatus(Enum):
    STOPPED = "Stopped"
    STANDBY = "Standby"
    ACTIVE = "Active"

class RTKStatus(Enum):
    NORMAL = "normal"
    ERROR = "error"
    
#########################################################

class Lift(BaseModel):
    status: LiftStatus
    height: float

class RTK(BaseModel):
    status: RTKStatus
    error_range: float

class MowingService(BaseModel):
    fuel: float
    autonomous_status: AutonomousStatus
    cutter: Cutter
    lift: Lift
    rtk: RTK
    rollangle: float
    rollover_status: str

class Service(BaseModel):
    mowing: MowingService

class Task(BaseModel):
    task_id: str
    task_code: str
    task_status: TaskStatus

class RobotStatus(BaseModel):
    robot_serial: str
    create_time: str
    x: float
    y: float
    battery: float
    drive_status: DrivingStatus
    speed: int
    heading: int
    charge: bool
    charge_type: str
    is_indoor: bool
    coord_code: str = "WGS84"
    service_mode: str = "mowing"
    service: Service
    task: Task
