# Camera & IMU Calibration
Calibrate camera intrinsics and camera-imu extrinsics.

## Installation
### Option A: Using OpenImuCameraCalibrator (Recommend)
### 1. Build dependencies
#### (1) Eigen 3.4
```
sudo apt-get install libdw-dev
cd ~/Downloads
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
tar -xvzf eigen-3.4.0.tar.gz
rm eigen-3.4.0.tar.gz
cd eigen-3.4.0 && mkdir build && cd build
cmake ..
sudo make install -j4
sudo cp -r /usr/local/include/eigen3 /usr/include
```

#### (2) OpenCV 4.5.x with contrib module
If you are using **Ubuntu 22.04**, just run:
```
sudo apt-get install libopencv-dev libopencv-contrib-dev
```
Otherwise, follow the instructions [here](https://viking-drone.com/wiki/installing-opencv-4-5-2/).

#### (3) Ceres 2.1
```
sudo apt-get install liblapack-dev libblas-dev libgflags-dev libgoogle-glog-dev libsuitesparse-dev libcxsparse3 libgtest-dev
cd ~/Downloads
wget https://github.com/ceres-solver/ceres-solver/archive/2.1.0.zip
unzip 2.1.0.zip -d ~/Downloads
rm 2.1.0.zip
cd ceres-solver-2.1.0 && mkdir build && cd build
cmake ..
sudo make install -j4
```
#### (4) PyTheia
```
cd ~/Downloads
git clone https://github.com/urbste/pyTheiaSfM
cd pyTheiaSfM && git checkout 69c3d37 && mkdir -p build && cd build
cmake ..
sudo make install -j4
```
#### (5) OpenImuCameraCalibrator
```
cd ~/Downloads
git clone https://github.com/urbste/OpenImuCameraCalibrator
cd OpenImuCameraCalibrator && mkdir build && cd build
cmake ..
make -j
```
When running the last command, some errors may occur:
```shell
fatal error: opencv2/aruco.hpp: No such file or directory
   18 | #include <opencv2/aruco.hpp>
      |          ^~~~~~~~~~~~~~~~~~~
```

## Option 2: Using Kalibr
### (1) Download and build Kalibr from source
Kalibr is a powerful tool used for camera and IMU calibration. Detailed information can be found at [Kalibr](https://github.com/ethz-asl/kalibr). Follow the instructions below to build Kalibr from source.
```
cd ~/<workspace>/src
git clone https://github.com/ori-drs/kalibr.git
cd ..
catkin_make
source devel/setup.bash
```
Replace `<workspace>` with your own working space.

### (2) Process raw data and then create ROS bag
```
python data_preprocess.py -l <load_dir>
```
Replace `<load_dir>` with your own loading directory. For instance, if raw data is stored in `~/universal_manipulation/2024_03_25-09_22_52`, `<load_dir>` should be `2024_03_25-09_22_52`. 

### (3) Calibrate camera intrinsics
```
rosrun kalibr kalibr_calibrate_cameras \
    --models pinhole-radtan \
    --topics /cam0/image_raw \
    --bag ~/universal_manipulation/<load_dir>/dataset/data.bag \
    --target ~/universal_manipulation/calibration/yaml/apriltag_config.yaml
```
Replace `<load_dir>` with your own loading directory as what you did in step 2.
