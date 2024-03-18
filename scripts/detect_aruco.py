import sys
import os
import click
import yaml
import json
import av
import numpy as np
import cv2
import pickle
from tqdm import tqdm

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

from utils.cv_util import (
    parse_aruco_config, 
    parse_pinhole_intrinsics,
    convert_pinhole_intrinsics_resolution,
    detect_localize_aruco_tags,
    draw_predefined_mask
)

@click.command()
@click.option('-i', '--input', required=True, help="mp4 video path")
@click.option('-o', '--output', required=True, help="output pkl file path")
@click.option('-ij', '--intrinsics_json', required=True, help="camera intrinsics json file path")
@click.option('-ay', '--aruco_yaml', required=True, help="ArUco config yaml file path")
@click.option('-n', '--num_workers', type=int, default=4)

def main(input, output, intrinsics_json, aruco_yaml, num_workers):
    cv2.setNumThreads(num_workers)

    ### load ArUco configuration
    aruco_config = parse_aruco_config(yaml.safe_load(open(aruco_yaml, 'r')))
    aruco_dict = aruco_config['aruco_dict']
    marker_size_map = aruco_config['marker_size_map']
    """
    aruco_dict = DICT_4X4_50 (default)

    marker_size_map = {
        0: <size_0>,
        1: <size_1>,
        ...
        49: <size_49>
    }

    If you assign values to certain ids (e.g. 0: 0.06) in the ArUco yaml file,
    <size_id> will be those values. Otherwise, <size_id> will be the "default".  
    
    """
    # load intrinsics
    raw_fisheye_intr = parse_pinhole_intrinsics(json.load(open(intrinsics_json, 'r')))

    results = list()
    with av.open(os.path.expanduser(input)) as in_container:
        in_stream = in_container.streams.video[0]
        in_stream.thread_type = "AUTO"
        in_stream.thread_count = num_workers
        ### get recorded videos' resolution
        in_res = np.array([in_stream.height, in_stream.width])[::-1]
        ### adjust camera intrinsics (resolution correction)
        fisheye_intr = convert_pinhole_intrinsics_resolution(
            opencv_intr_dict=raw_fisheye_intr, target_resolution=in_res)
        ### detect ArUco markers in each video frame
        for i, frame in tqdm(enumerate(in_container.decode(in_stream)), total=in_stream.frames):
            img = frame.to_ndarray(format='rgb24')
            frame_cts_sec = frame.pts * in_stream.time_base
            ### mask mirrors
            """
            Remarks: 
                Here we assume there are no mirrors. If mirrors are mounted, remember to
                set "mirror=True" in "draw_predefined_mask".
            """
            img = draw_predefined_mask(img, color=(0,0,0), mirror=False, gripper=False, finger=False)
            ### detect ArUco markers
            tag_dict = detect_localize_aruco_tags(
                img=img,
                aruco_dict=aruco_dict,
                marker_size_map=marker_size_map,
                fisheye_intr_dict=fisheye_intr,
                refine_subpix=True
            )
            """
            tag_dict = {
                'id_0': {
                    'rvec': rvec,
                    'tvec': tvec,
                    'corners': corners
                },
                'id_1': {...},
                ...
            }

            "id_x" is the marker ids found in the image.

            """
            result = {
                'frame_idx': i,
                'time': float(frame_cts_sec),
                'tag_dict': tag_dict
            }
            results.append(result)
    
    ### create pkl files
    pickle.dump(results, open(os.path.expanduser(output), 'wb'))
    """
    results: [
        {
            'frame_idx': 0,
            'time': frame_cts_sec,
            'tag_dict': tag_dict
        },
        ...
    ]
    """


if __name__ == "__main__":
    main()
