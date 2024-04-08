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
Obtain camera trajectories
"""
import sys
import os
import cv2
import pathlib
import click
import subprocess
import numpy as np
from tqdm import tqdm

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from utils.cv_util import draw_predefined_mask

@click.command()
@click.option('-i', '--input_dir', required=True, help='Directory for demos folder')
@click.option('-r', '--resolution', type=tuple, default=(2160, 3840), help='image resolution')
@click.option('-m', '--map_path', default=None, help='ORB_SLAM3 *.osa map atlas file')
@click.option('-d', '--docker_image', default="chicheng/orb_slam3:latest")
@click.option('-ml', '--max_lost_frames', type=int, default=60)
@click.option('-np', '--no_docker_pull', is_flag=True, default=False, help="pull docker image from docker hub")

def main(input_dir, resolution, map_path, docker_image, max_lost_frames, no_docker_pull):
    input_dir = pathlib.Path(os.path.expanduser(input_dir)).absolute()  # <session>/demos
    input_video_dirs = [x.parent for x in input_dir.glob('demo*/*.mp4')]
    input_video_dirs += [x.parent for x in input_dir.glob('map*/*.mp4')]  # no gripper calibration videos are needed
    print(f'Find {len(input_video_dirs)} video directories')
    
    if map_path is None:
        map_path = input_dir.joinpath('mapping', 'map_atlas.osa')

    else:
        map_path = pathlib.Path(os.path.expanduser(map_path)).absolute()

    assert map_path.is_file(), "Mapping file not found!"

    ### pull docker image
    if not no_docker_pull:
        print(f"Pulling docker image {docker_image}")
        cmd = [
            'docker',
            'pull',
            docker_image
        ]
        p = subprocess.run(cmd)
        if p.returncode != 0:
            print("Docker pull failed!")
            exit(1)

    for video_dir in tqdm(input_video_dirs):
        video_dir = video_dir.absolute()
        video_path = list(video_dir.glob("*.mp4"))[0]
        # if camear trajectory data already exist, skip
        if video_dir.joinpath('camera_trajectory.csv').is_file():
            print(f"\"camera_trajectory.csv\" already exists in {video_dir.name}, skip.")
            continue
        
        ### prepare docker volume
        mount_target = pathlib.Path('/data')
        csv_path = mount_target.joinpath('camera_trajectory.csv')
        video_path = mount_target.joinpath(video_path.name)
        json_path = mount_target.joinpath('imu_data.json')
        mask_path = mount_target.joinpath('slam_mask.png')
        mask_write_path = video_dir.joinpath('slam_mask.png')

        slam_mask = np.zeros(resolution, dtype=np.uint8)
        slam_mask = draw_predefined_mask(
            slam_mask, color=255, mirror=True, gripper=False, finger=True)
        cv2.imwrite(str(mask_write_path.absolute()), slam_mask)

        setting_mount_source = pathlib.Path(ROOT_DIR).joinpath("config/setting.yaml")
        setting_mount_target = pathlib.Path("/setting").joinpath(setting_mount_source.name)
        
        map_mount_source = map_path  # <session>/demos/mapping/map_atlas.osa
        map_mount_target = pathlib.Path('/map').joinpath(map_mount_source.name)
        
        ### mapping
        cmd = [
            'docker',
            'run',
            '--rm', # delete after finish
            '--volume', str(video_dir) + ':' + '/data',
            '--volume', str(map_mount_source.parent) + ':' + str(map_mount_target.parent),
            '--volume', str(setting_mount_source.parent) + ':' + '/setting',
            docker_image,
            '/ORB_SLAM3/Examples/Monocular-Inertial/gopro_slam',
            '--vocabulary', '/ORB_SLAM3/Vocabulary/ORBvoc.txt',
            '--setting', str(setting_mount_target),
            '--input_video', str(video_path), 
            '--input_imu_json', str(json_path),
            '--output_trajectory_csv', str(csv_path),
            '--load_map', str(map_mount_target),
            '--mask_img', str(mask_path),
            '--max_lost_frames', str(max_lost_frames)
        ]

        stdout_path = video_dir.joinpath('slam_stdout.txt')
        stderr_path = video_dir.joinpath('slam_stderr.txt')
        result = subprocess.run(
            cmd,
            cwd=str(video_dir),
            stdout=stdout_path.open('w'),
            stderr=stderr_path.open('w')
        )
        if result.returncode != 0:
            print(f"SLAM failed! Error in {str(video_dir)}.")
            exit(1)

    print("Finish 03_batch_slam")


if __name__ == "__main__":
    main()
