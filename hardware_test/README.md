We record datasets on a Unitree A1 robot contains proproceptive sensor data, mocap system data and camera images. 

The name of every rosbag indicates the motion pattern of the robot is executing in that rosbag. 

Scripts in folder "hardware_test" runs EKF on these rosbags to estimate the calf link length.

To run the EKF, first load the rosbag. Modify the **bagselect** filepath in 
```matlab
rosbag_process.m
```
So that the correct bag can be loaded. The script also performs some elementary signal processing filters on sensor data.

Then, run the following script
```matlab
test_ekf_hardware.m
```

The estimated calf length will be plotted in a few seconds. The length changes between 0.19-0.21m during walking. 


Comparing to simulation, the hardware estimation is more noisy. This is due to sensor noise and inaccuracy in foot contact detection. Also, as we analysed in the paper, when the joint velocities and body angular velocities are close to zero, kinematic parameters become unobservable. So the estimation will slowly drift when the robot stands still (around 3-6s in datasets).