import cv2
import json
import numpy as np
from pathlib import Path
from datetime import datetime
from datetime import timedelta

from .timecode_util import (
    get_timestamp_increment,
    date2datetime,
    datetime2timestamp
    )


def imu_synchronize(
    vel_data: list[dict],
    accel_data: list[dict]
    ):
    """Align velocity data with acceleration data"""
    imu_data = np.array([
        [
            vel_data[i]["time_usec"],
            vel_data[i]["x"],
            vel_data[i]["y"],
            vel_data[i]["z"],
            accel_data[i]["x"],
            accel_data[i]["y"],
            accel_data[i]["z"]
        ]
        for i in range(len(vel_data))
    ])

    return imu_data


def imu_convert_kalibr_format(
    load_dir: Path,
    save_dir: Path    
    ):
    ### get start datetime
    start_datetime = date2datetime(load_dir.name)
    ### load camera frame timestamps
    frame_ts_path = list(load_dir.glob("frames.json"))[0]
    frame_ts = json.load(open(str(frame_ts_path), 'rb'))["frames"]
    frame_timestamp = np.array(
        [
            frame_ts[i]["time_usec"]
            for i in range(len(frame_ts))
        ],
        dtype=np.float32
    )
    ### get frame timestamp increments
    frame_ts_inc = get_timestamp_increment(frame_timestamp)
    ### save video frames
    video_path = list(load_dir.glob("video.mp4"))[0]
    video_cp = cv2.VideoCapture(str(video_path))
    img_save_dir = save_dir.joinpath("cam0")
    if not img_save_dir.is_dir():
        img_save_dir.mkdir()

    for i in range(frame_ts_inc.shape[0]):
        success, frame = video_cp.read()
        if not success:
            break

        delta_time = timedelta(seconds=float(frame_ts_inc[i]))
        frame_datetime = start_datetime + delta_time
        fname = datetime2timestamp(frame_datetime)
        img_save_path = img_save_dir.joinpath(f"{fname}.png")
        cv2.imwrite(str(img_save_path), frame)
    ### load raw imu data
    vel_path = list(load_dir.glob("rotations.json"))[0]
    accel_path = list(load_dir.glob("accelerations.json"))[0]
    vel_data = json.load(open(str(vel_path), 'rb'))["rotations"]
    accel_data = json.load(open(str(accel_path), 'rb'))["accelerations"]
    ### synchronize gyro and accel data
    imu_data = imu_synchronize(vel_data, accel_data)
    ### get timestamp increments
    imu_timestamp = imu_data[:, 0]
    imu_ts_inc = get_timestamp_increment(imu_timestamp)
    ### save imu data
    for i in range(imu_data.shape[0]):
        delta_time = timedelta(seconds=float(imu_ts_inc[i]))
        imu_datetime = start_datetime + delta_time
        imu_timestamp = datetime2timestamp(imu_datetime)
        imu_data[i, 0] = float(imu_timestamp)

    imu_save_path = save_dir.joinpath("imu0.csv")
    np.savetxt(str(imu_save_path), imu_data, delimiter=",", fmt="%f")


def imu_convert_umi_format(
    load_dir: Path,
    save_dir: Path,
    fps: float
    ): 
    """
    convert raw imu data to the formats supported by UMI

    Params:
        load_dir: the directory used for loading imu data

        save_dir: the directory used for saving imu data

        start_datetime: see the directory name of "load_dir" 

        fps: camera frames per second, required if mode is "slam"
    """
    ### get start datetime
    start_datetime = date2datetime(load_dir.name)
    ### load raw imu data
    vel_path = list(load_dir.glob("rotations.json"))[0]
    accel_path = list(load_dir.glob("accelerations.json"))[0]
    vel_data = json.load(open(str(vel_path), 'rb'))["rotations"]
    accel_data = json.load(open(str(accel_path), 'rb'))["accelerations"]
    ### synchronize gyro and accel data
    imu_data = imu_synchronize(vel_data, accel_data)
    ### get timestamp increments
    timestamp = imu_data[:, 0]
    timestamp_inc = get_timestamp_increment(timestamp)
    ### convert formats
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
                },
                "CORI": {
                    "samples": [],
                    "name": "CameraOrientation"
                }
            },
            "device name": "Mobile Phone"
        },
        "frames/second": fps
    }
    for i in range(imu_data.shape[0]):
        ### get imu's datetime
        delta_time = timedelta(seconds=float(timestamp_inc[i]))
        imu_datetime = start_datetime + delta_time
        ### convert to UTC time
        imu_utc = imu_datetime - timedelta(hours=8)
        imu_utc = imu_utc.strftime(r"%Y-%m-%dT%H:%M:%S.%fZ")
        imu_utc = imu_utc[: -4] + "Z"  # convert microsecond to millisecond
        ### dump imu data
        cts = float(timestamp_inc[i] * 1e3)  # millisecond timestamps
        vel = imu_data[i, 0: 3].tolist()
        accel = imu_data[i, 3: 6].tolist()
        vel_data = {
            "value": vel,
            "cts": cts,
            "date": imu_utc,
            "temperature [\u2103]": 0.0
        }
        accel_data = {
            "value": accel,
            "cts": cts,
            "date": imu_utc,
            "temperature [\u2103]": 0.0
        }
        # CORI: the quaternion pose of camera (set to zeros)
        cori_data = {
            "value": [0, 0, 0, 0],
            "cts": cts,
            "date": imu_utc
        }
        imu_file["1"]["streams"]["GYRO"]["samples"].append(vel_data)
        imu_file["1"]["streams"]["ACCL"]["samples"].append(accel_data)
        imu_file["1"]["streams"]["CORI"]["samples"].append(cori_data)

    imu_save_path = save_dir.joinpath("imu_data.json")
    json.dump(imu_file, open(str(imu_save_path), "w"))


# def imu_synchronize(
#     vel_data: np.ndarray,
#     accel_data: np.ndarray
#     ):
#     """Align velocity data with acceleration data"""
#     num_vel = vel_data.shape[0]
#     num_accel = accel_data.shape[0]
#     imu_data = np.zeros((num_vel, 7), dtype=np.float32)
#     if num_vel >= num_accel:
#         imu_data[:, 0] = accel_data[:, 3]
#         imu_data[:, 4: 7] = accel_data[:, 0: 3]
#         imu_data[:, 1: 4] = vel_data[0: num_accel, 0: 3]

#     else:
#         imu_data[:, 0] = vel_data[:, 3]
#         imu_data[:, 4: 7] = accel_data[0: num_vel, 0: 3]
#         imu_data[:, 1: 4] = vel_data[:, 0: 3]

#     return imu_data


# def imu_convert_format(
#     load_dir: Path,
#     save_dir: Path,
#     start_datetime: datetime,
#     fps: float
#     ): 
#     """
#     convert raw imu data to the formats supported by UMI

#     Params:
#         load_dir: the directory used for loading imu data

#         save_dir: the directory used for saving imu data

#         start_datetime: see the directory name of "load_dir" 

#         fps: camera frames per second
#     """
#     ### load raw imu data
#     vel_path = list(load_dir.glob("*gyro.csv"))[0]
#     accel_path = list(load_dir.glob("*accel.csv"))[0]
#     vel_data = np.loadtxt(open(str(vel_path), 'rb'), delimiter=",")
#     accel_data = np.loadtxt(open(str(accel_path), 'rb'), delimiter=",")
#     ### synchronize gyro and accel data
#     imu_data = imu_synchronize(vel_data, accel_data)
#     ### get delta time (seconds) between neighbor timestamps
#     timestamp = imu_data[ :, 0]
#     timestamp[1: ] = (timestamp[1: ] - timestamp[: -1]) / 1e9
#     timestamp[0] = 0.0
#     ### get timestamp increments (seconds) w.r.t the starting time
#     timestamp_inc = np.array([
#         timestamp[: (i+1)].sum()
#         for i in range(timestamp.shape[0])
#     ])
#     ### convert formats
#     imu_file = {
#         "1": {
#             "streams": {
#                 "ACCL": {
#                     "samples": [],
#                     "name": "Accelerometer",
#                     "units": "m/s2"
#                 },
#                 "GYRO": {
#                     "samples": [],
#                     "name": "Gyroscope",
#                     "units": "rad/s"
#                 },
#                 "CORI": {
#                     "samples": [],
#                     "name": "CameraOrientation"
#                 }
#             },
#             "device name": "Mobile Phone"
#         },
#         "frames/second": fps
#     }
#     for i in range(imu_data.shape[0]):
#         ### get imu's datetime
#         delta_time = timedelta(seconds=float(timestamp_inc[i]))
#         imu_datetime = start_datetime + delta_time
#         ### convert to UTC time
#         imu_utc = imu_datetime - timedelta(hours=8)
#         imu_utc = imu_utc.strftime(r"%Y-%m-%dT%H:%M:%S.%fZ")
#         imu_utc = imu_utc[: -4] + "Z"  # convert microsecond to millisecond
#         ### dump imu data
#         cts = float(timestamp_inc[i] * 1e3)  # millisecond timestamps
#         gyro = imu_data[i, 0: 3].tolist()
#         accel = imu_data[i, 3: 6].tolist()
#         vel_data = {
#             "value": gyro,
#             "cts": cts,
#             "date": imu_utc,
#             "temperature [\u2103]": 0.0
#         }
#         accel_data = {
#             "value": accel,
#             "cts": cts,
#             "date": imu_utc,
#             "temperature [\u2103]": 0.0
#         }
#         # CORI: the quaternion pose of camera (set to zeros)
#         cori_data = {
#             "value": [0, 0, 0, 0],
#             "cts": cts,
#             "date": imu_utc
#         }
#         imu_file["1"]["streams"]["GYRO"]["samples"].append(vel_data)
#         imu_file["1"]["streams"]["ACCL"]["samples"].append(accel_data)
#         imu_file["1"]["streams"]["CORI"]["samples"].append(cori_data)

#     imu_save_path = save_dir.joinpath("imu_data.json")
#     json.dump(imu_file, open(str(imu_save_path), "w"))
    

# if __name__ == "__main__":
#     vel_data = json.load(open("/home/wangjl/project/universal_manipulation/PilotGuru/2024_03_19-12_19_31/rotations.json", 'rb'))["rotations"]
#     accel_data = json.load(open("/home/wangjl/project/universal_manipulation/PilotGuru/2024_03_19-12_19_31/accelerations.json", 'rb'))["accelerations"]
#     imu_data = imu_synchronize(vel_data, accel_data)
#     print(imu_data[0, 1])
