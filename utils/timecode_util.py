from datetime import datetime
from pathlib import Path


def str2int(string: str):
    if string.startswith("0"):
        return int(string[1:])
    
    else:
        return int(string)


def mp4_get_start_datetime(mp4_path: Path) -> datetime:
    """
    Return: datetime (Y, MON, D, H, MIN, S)
    """
    start_date = mp4_path.name[4: -4]
    year = str2int(start_date[0: 4])
    month = str2int(start_date[4: 6])
    day = str2int(start_date[6: 8])
    hour = str2int(start_date[9: 11])
    min = str2int(start_date[11: 13])
    sec = str2int(start_date[13: ])
    # TODO take microseconds into consideration
    mp4_start_datetime = datetime(year, month, day, hour, min, sec)
    
    return mp4_start_datetime
