import os
import sys
import click
from pathlib import Path

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from utils.imu_util import imu_convert_kalibr_format

@click.command(help="Convert raw videos and IMU data to Kalibr supported formats.")
@click.option("-l", "--load_dir", type=str, required=True, help="Directory where videos and raw IMU data are stored.")

def main(load_dir):
    load_dir = Path(os.path.expanduser(load_dir)).absolute()
    save_dir = load_dir.joinpath("dataset")
    if not save_dir.is_dir():
        save_dir.mkdir()

    imu_convert_kalibr_format(load_dir, save_dir)


# def make_parser():
#     parser = argparse.ArgumentParser(
#         description="Preprocess recorded data that are then used for calibration.",
#         formatter_class=argparse.ArgumentDefaultsHelpFormatter,
#     )
#     parser.add_argument(
#         "--root",
#         default="",
#         help="Root directory where videos and IMU data are stored."
#     )
#     parser.add_argument(
#         "--save_dir",
#         default="",
#         help="Directory used for saving prepared data."
#     )
#     return parser


# class Dataset:
#     """Calibration dataset"""
    
#     def __init__(self, args):
#         ROOT_DIR = Path(args.root)
#         self.video_path = list(ROOT_DIR.glob("VID*.mp4"))[0]
#         self.imu_dir = ROOT_DIR.joinpath(os.path.split(str(self.video_path))[1][4: -4]) 
            
#         self.save_dir = ROOT_DIR.joinpath(Path(args.save_dir))
#         if not self.save_dir.is_dir():
#             self.save_dir.mkdir()
            
#         self.image_save_dir = self.save_dir.joinpath("cam0")
#         if not self.image_save_dir.is_dir():
#             self.image_save_dir.mkdir()
            
    
#     def prepare(self):
#         # extract gyro data
#         gyro_path = list(self.imu_dir.glob("*gyro.csv"))[0]
#         gyro_data = np.loadtxt(open(str(gyro_path), 'rb'), delimiter=",") 
#         # extract acceleration data
#         accel_path = list(self.imu_dir.glob("*accel.csv"))[0]
#         accel_data = np.loadtxt(open(str(accel_path), 'rb'), delimiter=",")
#         # transform mp4 videos to image frames
#         video_timestamp_path = list(self.imu_dir.glob("*timestamps.csv"))[0]
#         video_timestamp = np.loadtxt(open(str(video_timestamp_path), 'rb'), delimiter=",")
#         video_cp = cv2.VideoCapture(str(self.video_path))
#         for time in range(video_timestamp.shape[0]):
#             success, frame = video_cp.read()
#             if not success:
#                 continue
#             cv2.imwrite(str(self.image_save_dir.joinpath(f"{int(video_timestamp[time])}.png")), frame)
            
#         video_cp.release()
        

#         num_gyro = gyro_data.shape[0]
#         num_accel = accel_data.shape[0]
#         imu_data = np.zeros((num_gyro, 7), dtype=np.float32)
#         if num_gyro >= num_accel:
#             imu_data[:, 0] = accel_data[:, 3]
#             imu_data[:, 4: 7] = accel_data[:, 0: 3]
#             imu_data[:, 1: 4] = gyro_data[0: num_accel, 0: 3]
    
#         else:
#             imu_data[:, 0] = gyro_data[:, 3]
#             imu_data[:, 4: 7] = accel_data[0: num_gyro, 0: 3]
#             imu_data[:, 1: 4] = gyro_data[:, 0: 3]
        
#         np.savetxt(str(self.save_dir.joinpath("imu0.csv")), imu_data, delimiter=",", fmt="%f")
            
    
# def main(argv=sys.argv[1:]):
#     parser = make_parser()
#     args = parser.parse_args(argv)
#     dataset = Dataset(args)
#     dataset.prepare()
    
    
if __name__ == "__main__":
    main()
