# Camera & IMU Calibration
Calibrate camera intrinsics and camera-imu extrinsics.

## Usage
### Option 1: Using OpenImuCameraCalibrator (Recommend)
#### (1) Build dependencies
a. Eigen 3.4
```
sudo apt-get install libdw-dev
wget -O ~/Downloads/eigen-3.4.0.tar.gz https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
cd ~/Downloads && sudo tar -xvzf eigen-3.4.0.tar.gz
cd eigen-3.4.0 && mkdir build && cd build
cmake ..
sudo make install
sudo cp -r /usr/local/include/eigen3 /usr/include
```
b. Ceres 2.1
```
sudo apt-get install liblapack-dev libblas-dev libeigen3-dev libgflags-dev libgoogle-glog-dev
sudo apt-get install liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev
wget -O ~/Downloads/ceres-solver-2.1.0.zip https://github.com/ceres-solver/ceres-solver/archive/2.1.0.zip
cd ~/Downloads && unzip ceres-solver-2.1.0.zip -d ~/Downloads
cd ceres-solver-2.1.0 && mkdir build && cd build
cmake ..
sudo make install -j4
```

#### 1. Download and build Kalibr from source
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
    --bag ~/universal_manipulation/<load_dir>/dataset/data.bag \
    --target ~/universal_manipulation/calibration/yaml/apriltag_config.yaml
```
Replace `<load_dir>` with your own loading directory as what you did in step 2.
