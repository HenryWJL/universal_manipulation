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
SLAM tag calibration and gripper range calibration
"""
import sys
import os
import pathlib
import click
import subprocess

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

@click.command()
@click.argument('session_dir', nargs=-1)

def main(session_dir):
    script_dir = pathlib.Path(__file__).parent.parent.joinpath('scripts')
    
    for session in session_dir:
        session = pathlib.Path(session)
        demos_dir = session.joinpath('demos')
        mapping_dir = demos_dir.joinpath('mapping')
        ### find relevant data
        script_path = script_dir.joinpath('calibrate_slam_tag.py')
        assert script_path.is_file()

        tag_path = mapping_dir.joinpath('tag_detection.pkl')
        assert tag_path.is_file(), f"\"tag_detection.pkl\" not found in {str(mapping_dir)}!"

        csv_path = mapping_dir.joinpath('camera_trajectory.csv')
        assert csv_path.is_file(), f"\"camera_trajectory.csv\" not found in {str(mapping_dir)}!"

        slam_tag_path = mapping_dir.joinpath('tx_slam_tag.json')
        ### run slam tag calibration
        cmd = [
            'python', str(script_path),
            '--tag_detection', str(tag_path),
            '--csv_trajectory', str(csv_path),
            '--output', str(slam_tag_path),
            '--keyframe_only'
        ]
        result = subprocess.run(cmd)

        if result.returncode != 0:
            print("SLAM tag calibration failed!")
            exit(1)
        
        ### gripper range calibration
        script_path = script_dir.joinpath('calibrate_gripper_range.py')
        assert script_path.is_file()
        
        for gripper_dir in demos_dir.glob("gripper_calibration*"):
            tag_path = gripper_dir.joinpath('tag_detection.pkl')
            assert tag_path.is_file(), f"\"tag_detection.pkl\" not found in {str(gripper_dir)}!"

            gripper_range_path = gripper_dir.joinpath('gripper_range.json')
            ### run gripper range calibration
            cmd = [
                'python', str(script_path),
                '--input', str(tag_path),
                '--output', str(gripper_range_path)
            ]
            result = subprocess.run(cmd)

            if result.returncode != 0:
                print(f"Gripper range calibration failed! Error in {str(gripper_dir)}.")
                exit(1)

            
if __name__ == "__main__":
    main()
