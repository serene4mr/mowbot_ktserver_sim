from pydantic import BaseModel
from typing import List, Literal
from enum import Enum

from .common_schema import Cutter, Lift, LiftStatus, TaskStatus

##################
    
class Point(BaseModel):
    x: float
    y: float

class Cutter(BaseModel):
    type: str
    width: float

class Attachment(BaseModel):
    cutter: Cutter
    lift: Lift

class Report(BaseModel):
    time: int
    distance: float
    area: float

class TaskData(BaseModel):
    map_id: int
    report: Report
    attachment: Attachment
    field_boundary: List[Point]

class Task(BaseModel):
    task_id: str
    task_code: str
    status: TaskStatus
    seq: int
    task_data: TaskData

class RobotMission(BaseModel):
    robot_serial: str
    create_time: str
    mission_code: str
    mission_id: str
    owner: str
    task: List[Task]
