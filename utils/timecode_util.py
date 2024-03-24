from datetime import datetime
from pathlib import Path
import numpy as np


def str2int(string: str):
    if string.startswith("0"):
        return int(string[1:])
    
    else:
        return int(string)


def get_timestamp_increment(timestamp: np.ndarray):
    ### get delta time (seconds) between neighbor timestamps
    timestamp[1: ] = (timestamp[1: ] - timestamp[: -1]) / 1e6
    timestamp[0] = 0.0
    ### get timestamp increments (seconds) w.r.t the starting time
    timestamp_inc = np.array([
        timestamp[ :(i+1)].sum()
        for i in range(timestamp.shape[0])
    ])

    return timestamp_inc


def convert_datetime(date: str) -> datetime:
    """
    Params:
        date: PilotGuru date, directory name (Y_MON_D-H_MIN_SEC)
    
    Returns:
        Python datetime (Y, MON, D, H, MIN, S)

    """
    year = str2int(date[0: 4])
    month = str2int(date[5: 7])
    day = str2int(date[8: 10])
    hour = str2int(date[11: 13])
    min = str2int(date[14: 16])
    sec = str2int(date[17: ])
    """
    Remarks: Microseconds are not taken into account
    """
    date_time = datetime(year, month, day, hour, min, sec)

    return date_time


def mp4_get_start_datetime(mp4_path: Path) -> datetime:
    start_date = mp4_path.name[0: -4]
    
    return convert_datetime(start_date)


# def mp4_get_start_datetime(mp4_path: Path) -> datetime:
#     """
#     Return: datetime (Y, MON, D, H, MIN, S)
#     """
#     start_date = mp4_path.name[4: -4]
#     year = str2int(start_date[0: 4])
#     month = str2int(start_date[4: 6])
#     day = str2int(start_date[6: 8])
#     hour = str2int(start_date[9: 11])
#     min = str2int(start_date[11: 13])
#     sec = str2int(start_date[13: ])
#     """
#     Remarks: Microseconds are not taken into account
#     """
#     mp4_start_datetime = datetime(year, month, day, hour, min, sec)
    
#     return mp4_start_datetime
