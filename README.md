# Universal Manipulation
This is a variant of [universal manipulation interface (UMI)](https://github.com/real-stanford/universal_manipulation_interface). Concretely, we made the following changes: 
- Other than GoPro cameras, users can now utilize their mobile phones as visual sensors and IMU processors.
- Details of camera calibration and camera-imu calibration are included.

## Hardware Requirements
### UMI Gripper
[Hardware Guide](https://docs.google.com/document/d/1TPYwV9sNVPAi0ZlAupDMkXZ4CA1hsZx7YDMSmcEy6EU/edit#heading=h.5k5vwx2iqjqg)

### Mobile Phone with PilotGuru
Download [PilotGuru](https://play.google.com/store/apps/details?id=ru.pilotguru.recorder&gl=DE) on your mobile phone.

## Installation
1. Install system-level dependencies:
```bash
sudo apt install -y libosmesa6-dev libgl1-mesa-glx libglfw3 patchelf
```
2. Download this repository:
```bash
git clone https://github.com/HenryWJL/universal_manipulation.git
```
3. Create a conda environment with all the dependencies.
```bash
conda env create -f conda_environment.yaml
```

## Usage
### 1. Calibration 
Follow [calibration instructions](https://github.com/HenryWJL/universal_manipulation/tree/main/calibration). 

### 2. Data Collection
#### (1) Record mapping video
Print out the ArUco marker in `aruco_mapping.pdf` and place it in the real-world working space (e.g., if you want to manipulate objects on a table, place it on that table). Then record a video (1 min) using **PilotGuru**.

#### (2) Record demonstrations
Mount your mobile phone onto the UMI gripper and hold the gripper to manipulate objects with **PilotGuru** turned on. Make sure your motions are not too fast in order to avoid motion blur.

### 3. Dataset Generation
Now your mobile phone should contain a directory dubbed `PilotGuru` that stores several videos and IMU data recorded by **PilotGuru**. Move it under `universal_manipulation` directory and run:
```python
python run_slam_pipeline.py -l PilotGuru
```
