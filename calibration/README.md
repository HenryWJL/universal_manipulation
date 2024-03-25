# Camera & IMU Calibration
Calibrate camera intrinsics and camera-imu extrinsics.

## Usage
### 1. Process raw data and create ROS bag
```
python data_preprocess.py -l <load_dir>
```
Replace `<load_dir>` with your own loading directory. For instance, if raw data is stored in `~/universal_manipulation/2024_03_25-09_22_52`, `<load_dir>` should be `2024_03_25-09_22_52`. 
### 2. Calibrate camera intrinsics
#### Firstly, download and build Kalibr from source. Kalibr is a powerful tool used for camera and IMU calibration. You can view detailed information at [Kalibr](https://github.com/ethz-asl/kalibr).
```
cd ~/<workspace>/src
git clone https://github.com/ori-drs/kalibr.git
cd ..
catkin_make
source devel/setup.bash
```
Replace `<workspace>` with your own workspace.
#### Next, run camera calibration. 
```
rosrun kalibr kalibr_calibrate_cameras \
    --models pinhole-radtan \
    --topics /cam0/image_raw \
    --bag /.../universal_manipulation/calibration/data/data.bag \
    --target /.../universal_manipulation/calibration/yaml/apriltag_config.yaml
```
