from datetime import datetime
from zoneinfo import ZoneInfo

def get_current_time_kst():
    """Get the current time in KST format: yyMMddHHmmssfff."""
    kst_timezone = ZoneInfo("Asia/Seoul")
    now_kst = datetime.now(kst_timezone)
    return now_kst.strftime("%y%m%d%H%M%S%f")[:15]

kst_timestamp = get_current_time_kst()
print("KST Timestamp:", kst_timestamp)