import os
import sys
import cv2
import yaml
import pathlib
import click
import numpy as np
from scipy.spatial.transform import Rotation

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

@click.command()
@click.option('-d', '--dataset_dir', required=True, help='Directory for loading ')

def main(dataset_dir):
    load_dir = pathlib.Path(os.path.expanduser(dataset_dir)).absolute()
    ### load camera intrinsics
    cam_path = load_dir.joinpath("cam/cam_calib_video_ph_1.0.json")
    cam_intr = yaml.load(open(str(cam_path), "rb").read(), Loader=yaml.FullLoader)
    fps = cam_intr["fps"]
    height = cam_intr["image_height"]
    width = cam_intr["image_width"]
    cam_type = cam_intr["intrinsic_type"]
    f = cam_intr["intrinsics"]["focal_length"]
    cx = cam_intr["intrinsics"]["principal_pt_x"]
    cy = cam_intr["intrinsics"]["principal_pt_y"]
    ### load camera-imu extrinsics (transformation matrix)
    cam_imu_path = load_dir.joinpath("cam_imu/cam_imu_calib_result_video.json")
    cam_imu_extr = yaml.load(open(str(cam_imu_path), "rb").read(), Loader=yaml.FullLoader)
    quat = list(cam_imu_extr["q_i_c"].values())
    trans = np.array(list(cam_imu_extr["t_i_c"].values()))
    rot = Rotation.from_quat(quat).as_matrix()
    mat = np.column_stack((rot, trans))
    mat = np.row_stack((mat, np.array([0, 0, 0, 1])))

    ### save
    # save_path = pathlib.Path(ROOT_DIR).joinpath("config/setting_copy.yaml")
    # fs = cv2.FileStorage(str(save_path), cv2.FileStorage_WRITE)
    # fs.write("File", 1.0)
    # fs.release()


if __name__ == "__main__":
    main()