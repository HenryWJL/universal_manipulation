"""
MIT License

Copyright (c) 2023 Columbia Artificial Intelligence and Robotics Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
"""
Main script for UMI SLAM pipeline.
"""

import sys
import os
import pathlib
import click
import subprocess

ROOT_DIR = os.path.dirname(__file__)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

@click.command()
@click.option("-l", "--load_dir", type=str, required=True, help="Directory where videos and raw IMU data are stored.")

def main(load_dir):
    load_dir = pathlib.Path(os.path.expanduser(load_dir)).absolute()
    script_dir = pathlib.Path(ROOT_DIR).joinpath("scripts_slam_pipeline")
    config_dir = pathlib.Path(ROOT_DIR).joinpath("config")

    print("############## 01_process_videos_and_imu #############")
    script_path = script_dir.joinpath("01_process_videos_and_imu.py")
    cmd = [
        'python', str(script_path),
        '-l', str(load_dir)
    ]
    result = subprocess.run(cmd)
    assert result.returncode == 0, "01_process_videos_and_imu failed!"

    print("############# 02_create_map ###########")
    script_path = script_dir.joinpath("02_create_map.py")
    mapping_dir = load_dir.joinpath('demos/mapping')
    assert mapping_dir.is_dir(), "Mapping directory not found!"
    map_path = mapping_dir.joinpath('map_atlas.osa')
    if not map_path.is_file():
        cmd = [
            'python', str(script_path),
            '-i', str(mapping_dir),
            '-m', str(map_path)
        ]
        result = subprocess.run(cmd)
        assert result.returncode == 0, "02_create_map failed!"

    print("Map already exists.")

    print("############# 03_batch_slam ###########")
    script_path = script_dir.joinpath("03_batch_slam.py")
    demo_dir = load_dir.joinpath('demos')
    cmd = [
        'python', str(script_path),
        '-i', str(demo_dir),
        '-m', str(map_path)
    ]
    result = subprocess.run(cmd)
    assert result.returncode == 0, "03_batch_slam failed!"

    print("############# 04_detect_aruco ###########")
    script_path = script_dir.joinpath("04_detect_aruco.py")
    camera_intrinsics = config_dir.joinpath('camera_intrinsics.json')
    aruco_config = config_dir.joinpath('aruco_config.yaml')
    assert camera_intrinsics.is_file(), "\"camera_intrinsics.json\" not found!"
    assert aruco_config.is_file(), "\"aruco_config.yaml\" not found!"

    cmd = [
        'python', str(script_path),
        '-i', str(demo_dir),
        '-ci', str(camera_intrinsics),
        '-ac', str(aruco_config)
    ]
    result = subprocess.run(cmd)
    assert result.returncode == 0, "04_detect_aruco failed!"

    print("############# 05_run_calibrations ###########")
    script_path = script_dir.joinpath("05_run_calibrations.py")
    cmd = [
        'python', str(script_path),
        '-i', str(demo_dir)
    ]
    result = subprocess.run(cmd)
    assert result.returncode == 0, "05_run_calibrations failed!"

    # print("############# 06_generate_dataset_plan ###########")
    # script_path = script_dir.joinpath("06_generate_dataset_plan.py")
    # assert script_path.is_file()
    # cmd = [
    #     'python', str(script_path),
    #     '--input', str(session)
    # ]
    # result = subprocess.run(cmd)
    # assert result.returncode == 0


if __name__ == "__main__":
    main()
