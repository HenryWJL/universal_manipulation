import os
import sys
import click
import subprocess
from pathlib import Path

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from utils.imu_util import imu_convert_kalibr_format

@click.command(help="Convert raw videos and IMU data to Kalibr supported formats.")
@click.option("-l", "--load_dir", type=str, required=True, help="Directory where videos and raw IMU data are stored.")

def main(load_dir):
    load_dir = os.path.join(ROOT_DIR, load_dir)
    load_dir = Path(os.path.expanduser(load_dir)).absolute()
    save_dir = load_dir.joinpath("dataset")
    if not save_dir.is_dir():
        save_dir.mkdir()

    print(f"All files are saved in {str(save_dir)}")
    ### convert raw videos and IMU data to Kalibr supported formats
    print("Preprocessing raw videos and IMU data...")
    imu_convert_kalibr_format(load_dir, save_dir)
    ### create rosbag
    script_path = os.path.join(ROOT_DIR, "calibration/scripts/create_bag.py")
    bag_save_path = save_dir.joinpath("data.bag")
    cmd = [
        "python", script_path,
        "--folder", str(save_dir),
        "--output-bag", str(bag_save_path),
    ]
    print("Creating ROS bag...")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print("Failed to create ROS bag!")
        exit(1)
    
    print("Done!")


if __name__ == "__main__":
    main()
