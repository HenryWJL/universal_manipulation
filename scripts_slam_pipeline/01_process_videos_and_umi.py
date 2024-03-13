import os
import sys
import click
import shutil
from pathlib import Path
from exiftool import ExifToolHelper

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from utils.timecode_util import mp4_get_start_datetime
from utils.imu_util import imu_convert_format


@click.command(help="Transform raw IMU data to UMI supported formats and move videos as well as processed IMU data to target directories.")
@click.option("-l", "--load_dir", type=str, required=True, help="Directory where videos and raw IMU data are stored.")
# @click.option("-s", "--save_dir", type=str, default="", help="Directory used for saving videos and processed IMU data.")


def main(load_dir):
    load_dir = Path(os.path.expanduser(load_dir)).absolute()
    input_dir = load_dir.joinpath('raw_videos')
    output_dir = load_dir.joinpath('demos')
    
    if not input_dir.is_dir():
        input_dir.mkdir()

        ### search for mp4 videos and move them to "raw_videos" directory
        print("\"raw_videos\" does not exist! Create one and move all videos in.")
        for mp4_path in list(load_dir.glob('VID*.mp4')):
            out_path = input_dir.joinpath(mp4_path.name)
            shutil.move(mp4_path, out_path)
    
    ### create mapping video if don't exist
    mapping_vid_dir = input_dir.joinpath('mapping')
    if (not mapping_vid_dir.is_dir()):
        mapping_vid_dir.mkdir()

        max_size = -1
        max_path = None
        for mp4_path in list(input_dir.glob('VID*.mp4')):
            size = mp4_path.stat().st_size  # sizes of mp4 videos
            # find mp4 video with maximum size and deem it as mapping video
            if size > max_size:
                max_size = size
                max_path = mp4_path

        print("Mapping video does not exist! Create one using the video with maximum size.")
        out_path = mapping_vid_dir.joinpath(max_path.name)
        shutil.move(max_path, out_path)
        
    ### create gripper calibration video if don't exist
    gripper_cal_dir = input_dir.joinpath('gripper_calibration')
    if not gripper_cal_dir.is_dir():
        gripper_cal_dir.mkdir()
        
        serial_start_dict = dict()
        serial_path_dict = dict()
        with ExifToolHelper() as et:
            for mp4_path in list(input_dir.glob('VID*.mp4')):
                start_date = mp4_get_start_datetime(mp4_path)
                meta = list(et.get_metadata(str(mp4_path)))[0]
                cam_serial = meta['QuickTime:CameraSerialNumber']
                # find mp4 video with earliest starting datatime and apply it for gripper calibration
                if cam_serial in serial_start_dict:
                    if start_date < serial_start_dict[cam_serial]:
                        serial_start_dict[cam_serial] = start_date
                        serial_path_dict[cam_serial] = mp4_path
                else:
                    serial_start_dict[cam_serial] = start_date
                    serial_path_dict[cam_serial] = mp4_path
        
        for serial, path in serial_path_dict.items():
            print(f"Gripper calibration videos do not exist. Selected {path.name} for camera serial {serial}")
            out_path = gripper_cal_dir.joinpath(path.name)
            shutil.move(path, out_path)

    ### Move mp4 videos and processed IMU data to target directories in "demos"
    input_mp4_paths = list(input_dir.glob('**/VID*.mp4'))
    print(f'Found {len(input_mp4_paths)} mp4 videos')

    with ExifToolHelper() as et:
        for mp4_path in input_mp4_paths:
            # symbolic links will be created later. If already exist, skip
            if mp4_path.is_symlink():
                print(f"Skipping {mp4_path.name}, already moved.")
                continue

            ### create directories used for storing metadata
            """format (out_dname)

            1. 'demo_<cam_serial>_year.month.day_hour.minute.second.xxxx'
            2. 'mapping'
            3. 'gripper_calibration_<cam_serial>_year.month.day_hour.minute.second.xxxx'

            """
            start_date = mp4_get_start_datetime(mp4_path)
            meta = list(et.get_metadata(str(mp4_path)))[0]
            cam_serial = meta['QuickTime:CameraSerialNumber']
            out_dname = 'demo_' + cam_serial + '_' + start_date.strftime(r"%Y.%m.%d_%H.%M.%S.%f")
            # strftime(): year.month.day_hour.minute.second.microsecond. e.g, 2024.03.01_10.26.20.849084

            # special folders to rename
            if mp4_path.parent.name.startswith('mapping'):
                out_dname = "mapping"  # store mapping data

            elif mp4_path.parent.name.startswith('gripper_cal'):  # store gripper calibration data
                out_dname = "gripper_calibration_" + cam_serial + '_' + start_date.strftime(r"%Y.%m.%d_%H.%M.%S.%f")

            this_out_dir = output_dir.joinpath(out_dname)  # <load_dir>/demos/<out_dname>
            this_out_dir.mkdir(parents=True, exist_ok=True)
            
            ### move mp4 videos to target directories
            out_video_path = this_out_dir.joinpath(mp4_path.name)  # <load_dir>/demos/<out_dname>/VID_*.mp4
            shutil.move(mp4_path, out_video_path)

            ### process and move IMU data to target directories
            imu_load_dir = load_dir.joinpath(mp4_path.name[4: -4])
            """
            Remark: we assume that videos and imu data are recorded simultaneously
            """
            imu_convert_format(imu_load_dir, this_out_dir, start_date)

            ### create symbolic link for all videos. Links are in "raw_videos"
            dots = os.path.join(*['..'] * len(mp4_path.parent.relative_to(load_dir).parts))
            rel_path = str(out_video_path.relative_to(load_dir))
            symlink_path = os.path.join(dots, rel_path)                
            mp4_path.symlink_to(symlink_path)


if __name__ == '__main__':
    if len(sys.argv) == 1:
        main.main(['--help'])

    else:
        main()
