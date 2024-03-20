"""
MIT License

Copyright (c) 2023 Columbia Artificial Intelligence and Robotics Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import numpy as np
import scipy.interpolate as si
import scipy.spatial.transform as st


def get_interp1d(t, x):
    gripper_interp = si.interp1d(
        t, x, 
        axis=0, bounds_error=False, 
        fill_value=(x[0], x[-1]))
    return gripper_interp


class PoseInterpolator:
    def __init__(self, t, x):
        pos = x[:,:3]
        rot = st.Rotation.from_rotvec(x[:,3:])
        self.pos_interp = get_interp1d(t, pos)
        self.rot_interp = st.Slerp(t, rot)
    
    @property
    def x(self):
        return self.pos_interp.x
    
    def __call__(self, t):
        min_t = self.pos_interp.x[0]
        max_t = self.pos_interp.x[-1]
        t = np.clip(t, min_t, max_t)

        pos = self.pos_interp(t)
        rot = self.rot_interp(t)
        rvec = rot.as_rotvec()
        pose = np.concatenate([pos, rvec], axis=-1)
        return pose

def get_gripper_calibration_interpolator(
        aruco_measured_width, 
        aruco_actual_width):
    """
    Assumes the minimum width in aruco_actual_width
    is measured when the gripper is fully closed
    and maximum width is when the gripper is fully opened
    """
    aruco_measured_width = np.array(aruco_measured_width)
    aruco_actual_width = np.array(aruco_actual_width)
    assert len(aruco_measured_width) == len(aruco_actual_width)
    assert len(aruco_actual_width) >= 2
    aruco_min_width = np.min(aruco_actual_width)
    gripper_actual_width = aruco_actual_width - aruco_min_width
    interp = get_interp1d(aruco_measured_width, gripper_actual_width)
    return interp
