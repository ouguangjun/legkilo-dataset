# legkilo-dataset
This is a legged robot dataset containing leg kinematics (joint encoders and contact sesors), imu and lidar.

<p align='center'>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/map_dog.jpg" alt="drawing" width="600"/>
</p>



## sequence
- **corridor**: A long corridor with few features and surrounded by  glass windows. The trajectory of the legged robot walking shaped like "8". The end and the beginning of the trajectory coincide.
- **park**: A semi-open parking lot full of vehicles, where the legged robot walking with dynamic objects (e.g., pedestrians) around it.
- **indoor**: The robot operating in static, non-uniform indoor environment.
- **running**: The robot running a short but quick circle outside at an average speed of 1.50 m/s.
- **slope**: The robot operating in a long slope with height variation (> 6 m). The end and the beginning of the trajectory coincide.

<p align='center'>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/dog01.jpg" alt="drawing" width="300"/>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/dog02.jpg" alt="drawing" width="300"/>
</p>





**corridor**
<p align='center'>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/corridor.gif" alt="drawing" width="600"/>
</p>

<p align='center'>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/corridor02.gif" alt="drawing" width="600"/>
</p>

**park**
<p align='center'>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/park01.gif" alt="drawing" width="600"/>
</p>
<p align='center'>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/park02.gif" alt="drawing" width="600"/>
</p>

**running**
<p align='center'>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/running.gif" alt="drawing" width="600"/>
</p>

### Rosbag Download
The rosbag for each sequence is provided in link [Google Drive](https://drive.google.com/drive/folders/1Egpj7FngTTPCeQDEzlbiK3iesPPZtqiM?usp=drive_link), which contains the ground truth. We obtain truth values through prior maps or offline optimization methods like [GR-Fusion](https://ieeexplore.ieee.org/abstract/document/9636232). At the same time, in the sequence corridor and slope sequence, the starting point and the ending point coincide, which can be evaluated by end-to-end error.

## Sensor Information
### sensor overview

<p align='center'>
    <img src="https://github.com/ouguangjun/kilo-dataset/blob/main/figure/sensor.png" alt="drawing" width="600"/>
</p>

### sensor type 
The data set includes liDAR, IMU, joint encoders, contact sensors, etc. Taking the sequence **corridor** as an example, the specific data format types are as follows:

```
path:        corridor.bag
version:     2.0
duration:    7:25s (445s)
start:       Sep 12 2023 21:05:58.65 (1694523958.65)
end:         Sep 12 2023 21:13:24.57 (1694524404.57)
size:        3.0 GB
messages:    670538
compression: none [3006/3006 chunks]
types:       nav_msgs/Odometry             [cd5e73d190d741a2f92e81eda573aca7]
             sensor_msgs/Imu               [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/PointCloud2       [1158d486dd51d683ce2f1be655c3c181]
             unitree_legged_msgs/HighState [470422f324a1822fc8bf6481d8aad1e4]
topics:      /high_state   222039 msgs    : unitree_legged_msgs/HighState
             /imu_raw      222039 msgs    : sensor_msgs/Imu              
             /points_raw     4421 msgs    : sensor_msgs/PointCloud2      
             /state_SDK    222039 msgs    : nav_msgs/Odometry
```

### unitree_legged_msgs/HighState
unitree_legged_msgs/HighState is the sensor feedback type provided by Unitree, which can be easily obtained from the sdk provided by Unitree. 
The specific content of highstate is as follows:
```
time stamp                                               #Added a timestamp to the original data format
uint8[2] head                                           
uint8 levelFlag
uint8 frameReserve

uint32[2] SN
uint32[2] version
uint16 bandWidth
IMU imu                                                    # IMU meausrement, including
                                                                     # quaternion(float[4])  : [0]:x, [1]:y, [2]:z, [3]w
                                                                     # gyroscope(float[3]) : [0]:x, [1]:y, [2]:z
                                                                    # accelerometer(float[3]) : [0]:x, [1]:y, [2]:z
                                                                    # rpy(float[3]) : [0]:roll, [1]:pitch, [2]:yaw
                                                                    # temperature(int8_t)

MotorState[20] motorState            #The first 12 arrays are valid, representing 12 encoder information FR_{0, 1, 2}, FL_{0, 1, 2}, RR_{0, 1, 2}, RL_{0, 1, 2}
                                                                    #every MotorState including:
                                                                    #q: angle (rad)  dq: velocity(rad/s) , etc.
BmsState bms
int16[4] footForce                                #contact force
int16[4] footForceEst
uint8 mode
float32 progress
uint8 gaitType		   
float32 footRaiseHeight		  
float32[3] position                                 #position, velocity, footPosition2Body, footSpeed2Body are all computed by Unitree built-in estimation.
float32 bodyHeight			  
float32[3] velocity 
float32 yawSpeed				   
float32[4] rangeObstacle
Cartesian[4] footPosition2Body 
Cartesian[4] footSpeed2Body	
uint8[40] wirelessRemote
uint32 reserve

uint32 crc
```
## Sensor Parameter
For parameters of each sensor, such as external parameters and leg kinematic parameters, please refer to ```config/go1.yaml ```

##  Kinematic-inertial Estimator
We provide a kinematic-inertial legged state estimation adapted to this dataset, which is derived from [MIT Cheetah 3](https://github.com/mit-biomimetics/Cheetah-Software) and configured by [ ShuoYangRobotics ](https://github.com/ShuoYangRobotics/A1-QP-MPC-Controller?tab=readme-ov-file). On the basis of the above, we added the adaptation of sensor data, and it was verified on Go1.

### Build
The estimator is tested in Ubuntu 18.04 and ROS1.  Make following command:
```
cd catkin_ws/src
git clone https://github.com/ouguangjun/legkilo-dataset.git
cd ..
catkin_make
```
### Run
```
source devel/setup.bash
roslaunch kalman_go1 pubLegKF.launch
```
and  play rosbag on another terminal

```
rosbag play corridor.bag
```

If you want to run this program on your robot, you can call the SDK to publish highstate data:
```
roslaunch kalman_go1 pub_highstate.launch
```

## Run on Lidar-based algorithm

In order to better make the open source lidar-based algorithm run better on the dataset, we provide some parameter modification suggestions:

-** A-LOAM**: LOAM is a pure lidar odometry, which is stable in our experiment and rarely has large height fluctuation, but the global error is larger than other LIOs.  Loop close detection can be added to improve the global accuracy, such as [SC-LOAM](https://github.com/gisbi-kim/SC-A-LOAM).
-**LIOSAM**: The vibration or impacts of the legged robot makes the IMU degenerate, while the IMU odometry based on pre-integration in LIOSAM accumulates a large error in a short time. We suggest replacing IMU's odometry with kinematic-inertial odometry provided by us, and adding the leg odometry factor to the factor map, which will greatly improve the accuracy of LIOSAM.
-**FASTLIO ** and **Point-lio**: fastlio and point-lio are algorithms based on Kalman filtering. For IMU observation, the accelerometer covariance can be increased or the error standard deviation of the lidar observation can be reduced. The Point-lio algorithm performs best in our experiment because it uses IMU as a measurement value rather than an input value, which makes it more robust when encountering unstable IMU data.




