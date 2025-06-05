from enum import Enum


class ErrorStatus(Enum):
    OCCURRED = "Occurred"
    FIXED = "Fixed"
    
class ErrorCode(Enum):
    CODE_01 = "01"  # H/W 고장(예시)
    CODE_02 = "02"  # 연료 부족(예시)
    CODE_03 = "03"  # 경고등 점등(예시)
    CODE_04 = "04"  # 시스템 로그 발생(예시)
    CODE_100 = "100"  # GPS 오류
    CODE_101 = "101"  # 센서 오류
    CODE_102 = "102"  # IMU 오류
    CODE_103 = "103"  # 경로 진폭
    

