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
Detect ArUco markers on the gripper
"""
import sys
import os
import pathlib
import click
import subprocess
from tqdm import tqdm

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

@click.command()
@click.option('-i', '--input_dir', required=True, help='Directory for demos folder')
@click.option('-ci', '--camera_intrinsics', required=True, help='Camera intrinsics json file')
@click.option('-ac', '--aruco_yaml', required=True, help='Aruco config yaml file')
@click.option('-n', '--num_workers', type=int, default=None)

def main(input_dir, camera_intrinsics, aruco_yaml, num_workers):
    input_dir = pathlib.Path(os.path.expanduser(input_dir))  # <session>/demos
    input_video_dirs = [x.parent for x in input_dir.glob('*/VID*.mp4')]
    print(f'Find {len(input_video_dirs)} video directories')
    
    assert os.path.isfile(camera_intrinsics), "Camera intrinsics not found!"
    assert os.path.isfile(aruco_yaml), "ArUco configuration not found!"

    script_path = pathlib.Path(__file__).parent.parent.joinpath('scripts', 'detect_aruco.py')

    for video_dir in tqdm(input_video_dirs):
        video_dir = video_dir.absolute()
        video_path = list(video_dir.glob("VID*.mp4"))[0]
        pkl_path = video_dir.joinpath('tag_detection.pkl')

        if pkl_path.is_file():
            print(f"\"tag_detection.pkl\" already exists in {video_dir.name}, skip")
            continue

        ### detect ArUco markers on the gripper
        # TODO modify 'detect_aruco.py' to exclude fisheye
        cmd = [
            'python', script_path,
            '--input', str(video_path),
            '--output', str(pkl_path),
            '--intrinsics_json', camera_intrinsics,
            '--aruco_yaml', aruco_yaml,
            '--num_workers', '1'
        ]
        result = subprocess.run(
            cmd,
            cwd=str(video_dir)
        )
        if result.returncode != 0:
            print(f"ArUco detection failed! Error in {str(video_dir)}.")
            exit(1)

    print("Finish 04_detect_aruco")


if __name__ == "__main__":
    main()
