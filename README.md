# Universal Manipulation
This is a variant of [universal manipulation interface (UMI)](https://github.com/real-stanford/universal_manipulation_interface). Concretely, we made two changes to the original package. First, the GoPro camera is replaced by a mobile phone. Second, the detailed process of camera-imu calibration is included. We hope that these improvements will allow more people to gain access to UMI.

## Requirements
### 1. Mobile phone
Download [Pilotguru](https://play.google.com/store/apps/details?id=ru.pilotguru.recorder&gl=DE) on your mobile phone. This app will be used for recording videos and IMU data.

### 2. Conda environment
```bash
conda env create -f conda_environment.yaml
```
The SLAM pipeline will be run in this environment.

## Usage
### 1. Calibration 
