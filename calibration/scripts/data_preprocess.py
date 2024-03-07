import os
import sys
import cv2
import argparse
import numpy as np
from pathlib import Path


def make_parser():
    parser = argparse.ArgumentParser(
        description="Preprocess recorded data that are then used for calibration.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--root",
        default="C:/Users/86136/python_project/videos",
        help="Root directory where videos and IMU data are stored."
    )
    parser.add_argument(
        "--save_dir",
        default="dataset_dir",
        help="Directory used for saving prepared data."
    )
    return parser


class Dataset:
    """Calibration dataset"""
    
    def __init__(self, args):
        ROOT_DIR = Path(args.root)
        self.video_path = list(ROOT_DIR.glob("VID*.mp4"))[0]
        self.imu_dir = ROOT_DIR.joinpath(os.path.split(str(self.video_path))[1][4: -4]) 
            
        self.save_dir = ROOT_DIR.joinpath(Path(args.save_dir))
        if not self.save_dir.is_dir():
            self.save_dir.mkdir()
            
        self.image_save_dir = self.save_dir.joinpath("cam0")
        if not self.image_save_dir.is_dir():
            self.image_save_dir.mkdir()
            
    
    def prepare(self):
        # extract gyro data
        gyro_path = list(self.imu_dir.glob("*gyro.csv"))[0]
        gyro_data = np.loadtxt(open(str(gyro_path), 'rb'), delimiter=",")
        gyro_data = gyro_data[ :, [3, 0, 1, 2]]  # place column_3 (timestamp) at the beginning 
        # extract acceleration data
        accel_path = list(self.imu_dir.glob("*accel.csv"))[0]
        accel_data = np.loadtxt(open(str(accel_path), 'rb'), delimiter=",")
        accel_data = accel_data[ :, [3, 0, 1, 2]]  # place column_3 (timestamp) at the beginning
        # transform mp4 videos to image frames
        video_timestamp_path = list(self.imu_dir.glob("*timestamps.csv"))[0]
        video_timestamp = np.loadtxt(open(str(video_timestamp_path), 'rb'), delimiter=",")
        video_cp = cv2.VideoCapture(str(self.video_path))
        for time in range(video_timestamp.shape[0]):
            success, frame = video_cp.read()
            if not success:
                continue
            cv2.imwrite(str(self.image_save_dir.joinpath(f"{int(video_timestamp[time])}.png")), frame)
            
        video_cp.release()
        # interpolate acceleration, making it synchronize with gyro
        idx_accel = 0
        idx_gyro = 0
        idx_imu = 0
        num_gyro = gyro_data.shape[0]
        num_accel = accel_data.shape[0]
        imu_data = np.zeros((num_gyro, 7), dtype=np.float32)
        while (accel_data[0, 0] > gyro_data[idx_gyro, 0]):
            idx_gyro += 1

        while (idx_accel + 1 < num_accel and idx_gyro < num_gyro):
            # compute timestamp and acceleration difference
            delta_time_accel = accel_data[(idx_accel + 1), 0] - accel_data[idx_accel, 0]
            delta_accel = accel_data[(idx_accel + 1), 1: 4] - accel_data[idx_accel, 1: 4]
            # combine imu data
            while (idx_gyro < num_gyro and accel_data[idx_accel + 1, 0] >= gyro_data[idx_gyro, 0]):
                imu_data[idx_imu, 0] = gyro_data[idx_gyro, 0]
                # interpolate acceleration
                imu_data[idx_imu, 4: 7] = accel_data[idx_accel, 1: 4] + \
                    (gyro_data[idx_gyro, 0] - accel_data[idx_accel, 0]) * delta_accel / delta_time_accel
                # dump gyro data
                imu_data[idx_imu, 1: 4] = gyro_data[idx_gyro, 1: 4]

                idx_gyro += 1
                idx_imu += 1

            idx_accel += 1

        imu_data = np.delete(imu_data, range(idx_imu, num_gyro), axis=0)
        np.savetxt(str(self.save_dir.joinpath("imu0.csv")), imu_data, delimiter=",")
            
    
def main(argv=sys.argv[1:]):
    parser = make_parser()
    args = parser.parse_args(argv)
    dataset = Dataset(args)
    dataset.prepare()
    
    
if __name__ == "__main__":
    main()
