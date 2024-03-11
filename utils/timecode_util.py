import datetime
import numpy as np
from pathlib import Path


def str2int(string: str):
    if string.startswith("0"):
        return int(string[1:])
    
    else:
        return int(string)


def mp4_get_start_datetime(load_dir: Path, start_date: str) -> datetime.datetime:
    """
    start_date: {year}{month}{day}_{hour}{minute}{second}, such as 20240307_184717
    """
    timestamp_path = list(load_dir.glob(f"**/*{start_date}_imu_timestamps.csv"))[0]
    start_timestamp = np.loadtxt(open(str(timestamp_path), 'rb'), delimiter=",")[0]
    
    year = str2int(start_date[0: 4])
    month = str2int(start_date[4: 6])
    day = str2int(start_date[6: 8])
    hour = str2int(start_date[9: 11])
    min = str2int(start_date[11: 13])
    sec = str2int(start_date[13: ])
    microsec =  int(start_timestamp / 10e11)  # TODO convert to microseconds
    mp4_start_datetime = datetime.datetime(year, month, day, hour, min, sec, microsec)
    
    return mp4_start_datetime


def imu_convert_format(imu_data: np.ndarray, mp4_start_datetime: datetime): 
    start_datetime = mp4_start_datetime.replace(microsecond=0)
    imu_file = {
        "1": {
            "streams": {
                "ACCL": {
                    "samples": [],
                    "name": "Accelerometer",
                    "units": "m/s2"
                },
                "GYRO": {
                    "samples": [],
                    "name": "Gyroscope",
                    "units": "rad/s"
                }
            },
            "device name": "Mobile Phone"
        },
        "frames/second": 100 # TODO
    }
    for i in range(imu_data.shape[0]):
        start_timestamp = imu_data[i, -1]
        microsec = int(start_timestamp / 10e8)  # TODO convert to microseconds
        delta_time = datetime.timedelta(microseconds=microsec)
        imu_datetime = start_datetime + delta_time
        imu_utc_datatime = imu_datetime - datetime.timedelta(hours=8)
        imu_utc = imu_utc_datatime.strftime(r"%Y-%m-%dT%H:%M:%S.%fZ")  # TODO we want milliseconds after dot, not microseconds
        gyro = list(imu_data[i, 0: 3])
        accel = list(imu_data[i, 3: 6])
        gyro_data = {
            "value": gyro,
            "cts": 0,
            "date": imu_utc,
            "temperature [°C]": 53.400390625
        }
        accel_data = {
            "value": accel,
            "cts": 0,
            "date": imu_utc,
            "temperature [°C]": 53.400390625
        }
        imu_file["1"]["streams"]["GYRO"]["samples"].append(gyro_data)
        imu_file["1"]["streams"]["ACCL"]["samples"].append(accel_data)

    return imu_file
    

if __name__ == "__main__":
    load_dir = Path("/home/wangjl/project")
    mp4_path = "20240307_184717"
    time = mp4_get_start_datetime(load_dir, mp4_path)
    print(time.strftime(r"%Y-%m-%dT%H:%M:%S.%fZ"))
