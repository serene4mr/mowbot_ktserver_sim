from pydantic import BaseModel
from typing import Literal
from enum import Enum   

class LiftStatus(Enum):
    UP = "up"
    DOWN = "down"
    
class TaskStatus(Enum):
    STANDBY = "Standby"
    STARTED = "Started"
    DEST_ARRIVED = "DestArrived"
    ON_PROGRESS = "OnProgress"
    END = "End"
    CANCELLED = "Cancelled"
    FAILED = "Failed"
    
    
##############################
    
class Cutter(BaseModel):
    type: Literal['rotary', 'flail']
    width: float

class Lift(BaseModel):
    status: LiftStatus
    height: float
    