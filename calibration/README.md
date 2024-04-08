# Camera & IMU Calibration
Calibrate camera intrinsics and camera-imu extrinsics.

## Installation
### Option A: Using OpenImuCameraCalibrator (Recommend)
#### 1. Build dependencies
##### (1) Eigen 3.4
```bash
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

##### (2) OpenCV 4.5.x with contrib module
If you are using **Ubuntu 22.04**, just run:
```bash
sudo apt-get install libopencv-dev libopencv-contrib-dev
```
Otherwise, follow the [instructions](https://viking-drone.com/wiki/installing-opencv-4-5-2/).

##### (3) Ceres 2.1
```bash
sudo apt-get install liblapack-dev libblas-dev libgflags-dev libgoogle-glog-dev libsuitesparse-dev libcxsparse3 libgtest-dev
cd ~/Downloads
wget https://github.com/ceres-solver/ceres-solver/archive/2.1.0.zip
unzip 2.1.0.zip -d ~/Downloads
rm 2.1.0.zip
cd ceres-solver-2.1.0 && mkdir build && cd build
cmake ..
sudo make install -j4
```
##### (4) PyTheia
```bash
cd ~/Downloads
git clone https://github.com/urbste/pyTheiaSfM
cd pyTheiaSfM && git checkout 69c3d37 && mkdir -p build && cd build
cmake ..
sudo make install -j4
```
##### (5) OpenImuCameraCalibrator
```bash
cd ~/Downloads
git clone https://github.com/urbste/OpenImuCameraCalibrator
cd OpenImuCameraCalibrator && mkdir build && cd build
cmake ..
make -j
```
When running the last command, some errors may occur:
```bash
fatal error: opencv2/aruco.hpp: No such file or directory
   18 | #include <opencv2/aruco.hpp>
      |          ^~~~~~~~~~~~~~~~~~~
```
To fix the errors, one way is to run:
```bash
sudo ln -s /usr/include/opencv4/opencv2 /usr/include/
```
#### 2. Create Conda environment
If you do not use Anaconda, run the last command only.
```bash
conda create -n openicc python=3.9 -y
conda activate openicc
pip install -r requirements.txt
```
#### 3. Prepare Docker
##### (1) Install Docker
See [official documentation](https://docs.docker.com/engine/install/ubuntu/).

##### (2) Build Docker container
```bash
cd OpenImuCameraCalibrator
sudo docker build -t openicc .
```
#### 4. Start calibration
Follow the [example](https://github.com/urbste/OpenImuCameraCalibrator/blob/master/docs/samsung_s20_calibration.md) and finish the first two steps. Afterwards, mount OpenICC folder and the folder that contains your calibration data to your docker container (`<parent_absolute_path>` is the absolute path of the parent directory of `MyS20Dataset`):
```bash
sudo docker run -it --rm -v `pwd`:/home -v <parent_absolute_path>:/data openicc
```
Lastly, run:
```bash
cd /home
python python/run_smartphone_calibration.py --path_calib_dataset=/data/MyS20Dataset --path_to_build ../OpenImuCameraCalibrator/build/applications/ --checker_size_m=0.02 --image_downsample_factor=1 --camera_model=PINHOLE --known_gravity_axis=Z
```
Remember to modify "checker_size_m", which is the real length of the larger grid of your ChArUco board.

### Option B: Using Kalibr
#### 1. Build Kalibr from source
Kalibr is a powerful tool used for camera and IMU calibration. Detailed information can be found at [Kalibr](https://github.com/ethz-asl/kalibr). Follow the instructions below to build Kalibr from source.
```bash
cd ~/<workspace>/src
git clone https://github.com/HenryWJL/kalibr.git
cd ..
catkin_make
source devel/setup.bash
```
Replace `<workspace>` with your own working space.

#### 2. Process raw data and create ROS bag
```bash
python data_preprocess.py -l <load_dir>
```
Replace `<load_dir>` with your own loading directory. For instance, if raw data is stored in `~/universal_manipulation/2024_03_25-09_22_52`, `<load_dir>` should be `2024_03_25-09_22_52`. 

#### 3. Calibrate camera intrinsics
```bash
rosrun kalibr kalibr_calibrate_cameras \
    --models pinhole-radtan \
    --topics /cam0/image_raw \
    --bag ~/universal_manipulation/<load_dir>/dataset/data.bag \
    --target ~/universal_manipulation/calibration/yaml/apriltag_config.yaml
```

#### 4. Calibrate IMU and camera extrinsics
```bash
rosrun kalibr kalibr_calibrate_imu_camera \
    --bag ~/universal_manipulation/<load_dir>/dataset/data.bag \
    --cam ~/universal_manipulation/calibration/yaml/cam_intrinsics.yaml \
    --imu ~/universal_manipulation/calibration/yaml/imu_intrinsics.yaml \
    --target ~/universal_manipulation/calibration/yaml/apriltag_config.yaml
```
