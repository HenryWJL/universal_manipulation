# universal_manipulation
Another version of universal manipulation interface (UMI)

## Usage
```
cd ~/<workspace>/src
git clone https://github.com/HenryWJL/universal_manipulation.git
cd ..
catkin_make
```
```
rosrun calibration data_preprocess.py --root /.../universal_manipulation/calibration/raw_data --save_dir /.../universal_manipulation/calibration/data
```
```
rosrun calibration create_bag.py --folder /.../universal_manipulation/calibration/data --output-bag /.../universal_manipulation/calibration/data/data.bag
```
```
rosrun kalibr kalibr_calibrate_cameras \
--models pinhole-radtan \
--topics /cam0/image_raw \
--bag /.../universal_manipulation/calibration/data/data.bag \
--target /.../universal_manipulation/calibration/yaml/apriltag_config.yaml
```
