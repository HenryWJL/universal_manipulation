import json
import numpy as np
from pathlib import Path
from datetime import datetime
from datetime import timedelta


def imu_synchronize(
    gyro_data: np.ndarray,
    accel_data: np.ndarray
    ):
    """
    align gyro data with accel data
    """
    num_gyro = gyro_data.shape[0]
    num_accel = accel_data.shape[0]
    imu_data = np.zeros((num_gyro, 7), dtype=np.float32)
    if num_gyro >= num_accel:
        imu_data[:, 0] = accel_data[:, 3]
        imu_data[:, 4: 7] = accel_data[:, 0: 3]
        imu_data[:, 1: 4] = gyro_data[0: num_accel, 0: 3]

    else:
        imu_data[:, 0] = gyro_data[:, 3]
        imu_data[:, 4: 7] = accel_data[0: num_gyro, 0: 3]
        imu_data[:, 1: 4] = gyro_data[:, 0: 3]

    return imu_data
    

# def imu_synchronize(
#     gyro_data: np.ndarray,
#     accel_data: np.ndarray
#     ):
#     """
#     Synchronize gyro and accel data 

#     Returns:
#         gyro + accel + timestamp
#     """
#     idx_accel = 0
#     idx_gyro = 0
#     idx_imu = 0
#     num_gyro = gyro_data.shape[0]
#     num_accel = accel_data.shape[0]
#     imu_data = np.zeros((num_gyro, 7), dtype=np.float32)
#     while (accel_data[0, -1] > gyro_data[idx_gyro, -1]):
#         idx_gyro += 1

#     while (idx_accel + 1 < num_accel and idx_gyro < num_gyro):
#         # compute timestamp and acceleration difference
#         delta_time_accel = accel_data[(idx_accel + 1), -1] - accel_data[idx_accel, -1]
#         delta_accel = accel_data[(idx_accel + 1), 0: 3] - accel_data[idx_accel, 0: 3]
#         # combine imu data
#         while (idx_gyro < num_gyro and accel_data[idx_accel + 1, -1] >= gyro_data[idx_gyro, -1]):
#             imu_data[idx_imu, -1] = gyro_data[idx_gyro, -1]
#             # interpolate acceleration
#             imu_data[idx_imu, 3: 6] = accel_data[idx_accel, 0: 3] + \
#                 (gyro_data[idx_gyro, -1] - accel_data[idx_accel, -1]) * delta_accel / delta_time_accel
#             # dump gyro data
#             imu_data[idx_imu, 0: 3] = gyro_data[idx_gyro, 0: 3]

#             idx_gyro += 1
#             idx_imu += 1

#         idx_accel += 1

#     imu_data = np.delete(imu_data, range(idx_imu, num_gyro), axis=0)


def imu_convert_format(
    load_dir: Path,
    save_dir: Path,
    start_datetime: datetime
    ): 
    """
    convert raw imu data to the formats supported by UMI

    Params:
        load_dir: the directory used for loading imu data

        save_dir: the directory used for saving imu data

        start_datetime: see the directory name of "load_dir" 
    """
    ### load raw imu data
    gyro_path = list(load_dir.glob("*gyro.csv"))[0]
    accel_path = list(load_dir.glob("*accel.csv"))[0]
    gyro_data = np.loadtxt(open(str(gyro_path), 'rb'), delimiter=",")
    accel_data = np.loadtxt(open(str(accel_path), 'rb'), delimiter=",")
    ### synchronize gyro and accel data
    imu_data = imu_synchronize(gyro_data, accel_data)
    ### get delta time (seconds) between neighbor timestamps
    timestamp = imu_data[ :, 0]
    """
    Remark: the unit of timestamp difference is not nanoseconds
    """
    timestamp[1: ] = (timestamp[1: ] - timestamp[: -1]) / 10e8
    timestamp[0] = 0.0
    ### get timestamp increments w.r.t the starting time
    timestamp_inc = np.array([
        timestamp[: (i+1)].sum()
        for i in range(timestamp.shape[0])
    ])
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
                }
            },
            "device name": "Mobile Phone"
        },
        "frames/second": 100 # TODO
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
        gyro = imu_data[i, 0: 3].tolist()
        accel = imu_data[i, 3: 6].tolist()
        # TODO find the meanings of "cts" and "temperature"
        gyro_data = {
            "value": gyro,
            "cts": 0.0,
            "date": imu_utc,
            "temperature [\u2103]": 0.0
        }
        accel_data = {
            "value": accel,
            "cts": 0.0,
            "date": imu_utc,
            "temperature [\u2103]": 0.0
        }
        imu_file["1"]["streams"]["GYRO"]["samples"].append(gyro_data)
        imu_file["1"]["streams"]["ACCL"]["samples"].append(accel_data)

    imu_save_path = save_dir.joinpath("imu_data.json")
    json.dump(imu_file, open(str(imu_save_path), "w"))
    