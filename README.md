# Universal Manipulation
This is a variant of [universal manipulation interface (UMI)](https://github.com/real-stanford/universal_manipulation_interface). Concretely, we made two changes to the original package. First, the GoPro camera is replaced by a mobile phone. Second, the detailed process of camera-imu calibration is included. We hope that these improvements will allow more people to gain access to UMI.

## Installation
```bash
git clone https://github.com/HenryWJL/universal_manipulation.git
```

## Requirements
### UMI gripper
[Hardware Guide](https://docs.google.com/document/d/1TPYwV9sNVPAi0ZlAupDMkXZ4CA1hsZx7YDMSmcEy6EU/edit#heading=h.5k5vwx2iqjqg)

### Mobile phone
Download [Pilotguru](https://play.google.com/store/apps/details?id=ru.pilotguru.recorder&gl=DE) on your mobile phone. This app will be used for recording videos and IMU data.

### Conda environment
```bash
conda env create -f conda_environment.yaml
```
The SLAM pipeline will be run in this environment.

## Usage
### 1. Calibration 
Follow the [calibration instructions](https://github.com/HenryWJL/universal_manipulation/tree/main/calibration) to calibrate your mobile phone. 

### 2. Data Collection
#### (1) Record mapping video
Print out the ArUco marker in `aruco_mapping.pdf` and place it in the real-world working space (e.g., if you want to manipulate objects on a table, place it on that table). Then record a video (1 min) using **PilotGuru**.

#### (2) Record manipulation data
Mount your mobile phone onto the UMI gripper and hold the gripper to manipulate objects with **PilotGuru** turned on. Make sure your motions are not too fast in order to avoid motion blur.

### 3. Dataset Creation
Now your mobile phone contains a directory dubbed `PilotGuru` that stores several videos and IMU data recorded by **PilotGuru**. Move it to the project directory on your computer
