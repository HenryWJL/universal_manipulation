# Camera & IMU Calibration
Calibrate camera intrinsics and camera-imu extrinsics.

## Usage
### 1. Download and build Kalibr from source
Kalibr is a powerful tool used for camera and IMU calibration. Detailed information can be found at [Kalibr](https://github.com/ethz-asl/kalibr). Follow the instructions below to build Kalibr from source.
```
cd ~/<workspace>/src
git clone https://github.com/ori-drs/kalibr.git
cd ..
catkin_make
source devel/setup.bash
```
Replace `<workspace>` with your own working space.

### 2. Process raw data and then create ROS bag
```
python data_preprocess.py -l <load_dir>
```
Replace `<load_dir>` with your own loading directory. For instance, if raw data is stored in `~/universal_manipulation/2024_03_25-09_22_52`, `<load_dir>` should be `2024_03_25-09_22_52`. 

### 3. Calibrate camera intrinsics
```
rosrun kalibr kalibr_calibrate_cameras \
    --models pinhole-radtan \
    --topics /cam0/image_raw \
    --bag /.../universal_manipulation/calibration/data/data.bag \
    --target /.../universal_manipulation/calibration/yaml/apriltag_config.yaml
```
