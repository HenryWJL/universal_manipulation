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
rosrun calibration data_preprocess.py --root /home/<usrname>/<workspace>/src/universal_manipulation/calibration/raw_data --save_dir /home/<usrname>/<workspace>/src/universal_manipulation/calibration/data
```
```
rosrun calibration create_bag.py --folder /home/<usrname>/<workspace>/src/universal_manipulation/calibration/data --output-bag /home/<usrname>/<workspace>/src/universal_manipulation/calibration/data/data.bag
```
