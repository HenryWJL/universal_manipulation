import os
import sys
import cv2
import click
import shutil
from pathlib import Path
from datetime import datetime

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from utils.timecode_util import mp4_get_start_datetime
from utils.imu_util import imu_convert_format

@click.command(help="Transform raw IMU data to UMI supported formats and move videos as well as processed IMU data to target directories.")
@click.option("-l", "--load_dir", type=str, required=True, help="Directory where videos and raw IMU data are stored.")

def main(load_dir):
    load_dir = Path(os.path.expanduser(load_dir)).absolute()
    input_dir = load_dir.joinpath('raw_videos')
    output_dir = load_dir.joinpath('demos')
    ### create "raw_videos" directory if doesn't exist
    if not input_dir.is_dir():
        input_dir.mkdir()
        ### search for mp4 videos and move them to "raw_videos" directory
        for mp4_path in list(load_dir.glob('VID*.mp4')):
            out_path = input_dir.joinpath(mp4_path.name)
            shutil.move(mp4_path, out_path)

        print("\"raw_videos\" does not exist! Create one and move all videos in.")
    
    ### create mapping video if don't exist
    mapping_vid_dir = input_dir.joinpath('mapping')
    if (not mapping_vid_dir.is_dir()):
        mapping_vid_dir.mkdir()
        ### find mp4 videos with maximum size and deem it as mapping videos
        max_size = -1
        path = None
        for mp4_path in list(input_dir.glob('VID*.mp4')):
            size = mp4_path.stat().st_size  # size of mp4 videos
            if size > max_size:
                max_size = size
                path = mp4_path

        out_path = mapping_vid_dir.joinpath(path.name)
        shutil.move(path, out_path)
        print(f"Mapping video does not exist! Select {path.name} for mapping.")   

    ### create gripper calibration videos if don't exist
    gripper_cal_dir = input_dir.joinpath('gripper_calibration')
    if not gripper_cal_dir.is_dir():
        gripper_cal_dir.mkdir()
        ### find mp4 videos with earliest start datatime 
        ### and use it for gripper calibration
        earliest_date = datetime(2077, 11, 15)
        path = None
        for mp4_path in list(input_dir.glob('VID*.mp4')):
            start_date = mp4_get_start_datetime(mp4_path)
            if start_date < earliest_date:
                earliest_date = start_date
                path = mp4_path
            
        out_path = gripper_cal_dir.joinpath(path.name)
        shutil.move(path, out_path)
        print(f"Gripper calibration video does not exist! Select {path.name} for gripper calibration.")

    ### move videos and IMU data to target directories in "demos"
    for mp4_path in list(input_dir.glob('**/VID*.mp4')):
        # symbolic links will be created later. If already exist, skip
        if mp4_path.is_symlink():
            print(f"Skip {mp4_path.name}, which is already done.")
            continue

        ### create directories used for storing metadata
        """Formats of <out_dname>

        1. 'demo_C3441328164125_<start_datetime>'
        2. 'mapping'
        3. 'gripper_C3441328164125_calibration_<start_datetime>'

        <start_datetime>: year.month.day_hour.minute.second.microsecond
        """
        """
        Remarks:
            Camera serials are not available (maybe due to the device itself).
            Hence, we use the camera serial in the "example_demo_session"
        """
        start_date = mp4_get_start_datetime(mp4_path)
        out_dname = 'demo_C3441328164125_' + start_date.strftime(r"%Y.%m.%d_%H.%M.%S.%f")
        # rename special folders 
        if mp4_path.parent.name.startswith('mapping'):
            out_dname = "mapping"

        elif mp4_path.parent.name.startswith('gripper_cal'):
            out_dname = "gripper_calibration_C3441328164125_" + start_date.strftime(r"%Y.%m.%d_%H.%M.%S.%f")

        this_out_dir = output_dir.joinpath(out_dname)  # <load_dir>/demos/<out_dname>
        this_out_dir.mkdir(parents=True, exist_ok=True)
        
        ### move mp4 videos to target directories
        out_video_path = this_out_dir.joinpath(mp4_path.name)  # <load_dir>/demos/<out_dname>/VID_*.mp4
        shutil.move(mp4_path, out_video_path)

        ### get camera fps
        cp = cv2.VideoCapture(str(out_video_path))
        fps = cp.get(cv2.CAP_PROP_FPS)
        cp.release()

        ### convert IMU data to certain formats and move them to target directories
        imu_load_dir = load_dir.joinpath(mp4_path.name[4: -4])
        """
        Remarks: 
            We assume that videos and imu data are recorded simultaneously.
            But actually, they are not.
        """
        imu_convert_format(imu_load_dir, this_out_dir, start_date, fps)

        ### create symbolic link for all videos. Links are in "raw_videos"
        dots = os.path.join(*['..'] * len(mp4_path.parent.relative_to(load_dir).parts))
        rel_path = str(out_video_path.relative_to(load_dir))
        symlink_path = os.path.join(dots, rel_path)                
        mp4_path.symlink_to(symlink_path)

    print("Finish 01_process_videos_and_imu.")


if __name__ == '__main__':
    if len(sys.argv) == 1:
        main.main(['--help'])

    else:
        main()
