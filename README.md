# BlueRov2 SLAM

## deprecated



<p align="center">
<img src="https://github.com/Bluerov2/MASTER/blob/sonar_mapping/images/hh.jpg " width="400" >
</p>

Recent approaches to robot localisation in complex environments have been data intensive,for example detailed LIDAR-derived maps have been used in self-driving cars.  In the under-water domain, the ability to gather or exploit equivalent data is more limited, due to both, the diffculty of access and the characteristics of the sensors typically deployed. Most of the workdone around offshore energy installations use remotely operated vehicles (ROVs) fitted withvideo cameras and sonars. The video data is used for close-range navigation of several metresas well as to inspect a system/product.  Sonars are used as a pilot navigation aid over longerranges where the use of video is not practical. this project will evaluate simultaneous localisa-tion and mapping (SLAM) algorithms for fusing sonar with DVL and IMU to produce maps forautonomous underwater vehicle (AUV) navigation or underwater ROV. Using a combinationof real-world and simulated data, the aim is to evaluate different SLAM performance by: Extracting range data from sonar images, DVL and IMU;•Building depth maps and converting into a 2D reconstruction of a broader scene; thee resulting 2D maps from different algorithms will be assessed with reference to the simu-lation environment or known features within the context of real world data

The entire implementation will be done using [ROS kinetic](http://wiki.ros.org/kinetic) framework with UUV simulation as environment. The model is simulated using [Desistek SAGA](https://uuvsimulator.github.io/packages/desistek_saga/intro/) but you can use your own model as well. We modified the sonar to be se same as the [Micron Sonar](https://www.tritech.co.uk/product/small-rov-mechanical-sector-scanning-sonar-tritech-micron) from Tritech. The sonar based SLAM is acheived using an ICP coupled with a Kalman Filter. The purpose was to implemente a robust sonar-based SLAM using only the simulator.

(more infromation available at the end of this page)


<p align="center">
 <img src="https://github.com/Bluerov2/MASTER/blob/sonar_mapping/images/BlueROV2-4-lumen-1-300x300.png " width="400" >
</p>

## Video presentation
<p align="center">
<a href="https://www.youtube.com/watch?v=qZnpLRyUY9A&feature=youtu.be
" target="_blank"><img src="https://github.com/Tim-HW/Tim-HW-BlueRov2_Sonar_based_SLAM-/blob/master/images/video.png" /></a>
</p>


## Related Packages

Required: Ubuntu 16.04 with ROS Kinetic (http://wiki.ros.org/kinetic). Also, the following packages are required:

* UUV-simulator:

  https://github.com/uuvsimulator/uuv_simulator

* Octomap_server :
  
  https://github.com/OctoMap/octomap_mapping.git
  
* Desistek SAGA ROV vehicle:

  https://github.com/uuvsimulator/desistek_saga.git
  
* Point cloud Converter:
  
  https://github.com/pal-robotics-forks/point_cloud_converter.git


  
  
  
To install every packages needed run the commands:

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/uuvsimulator/uuv_simulator
$ git clone https://github.com/uuvsimulator/desistek_saga.git
$ git clone https://github.com/pal-robotics-forks/point_cloud_converter.git
$ git clone https://github.com/fada-catec/amcl3d.git
$ cd ~/catkin_ws
$ catkin_make
```

## Installation

```
$ cd ~/catkin_ws/src
$ git clone https://github.com/Bluerov2/MASTER.git
$ cd ~/catkin_ws
$ catkin_make # or <catkin build>, if you are using catkin_tools
```

## Add the DVL

The original Robot doesn't have any DLV installed. Thus, we need to provide one :

Run the following command:
```
$ roscd desistek_saga_description/urdf/
$ sudo gedit desistek_saga_sensors.xacro 
```

then add the following command:
```xml
<!-- DVL  -->
<xacro:default_dvl_macro
  namespace="${namespace}"
  parent_link="${namespace}/base_link"
  inertial_reference_frame="${inertial_reference_frame}">
  <origin xyz="0 0 0" rpy="0 ${0.5*pi} 0"/>
</xacro:default_dvl_macro>
```

## Mechanical Sonar

 * you will need to change some URDF code from the ROV to allow a 360° vision.
for this run the following commands:

```
$ roscd uuv_simulator
$ cd ..
$ cd uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf
$ sudo gedit sonar_snippets.xacro
```
 * and the following sonar description :
```xml
  <xacro:macro name="micron_sonar" params="namespace parent_link *origin">
      <xacro:multibeam_sonar
        namespace="${namespace}"
        suffix=""
        parent_link="${parent_link}"
        topic="sonar"
        mass="0.02"
        update_rate="15"
        samples="396"
        fov="6.3"
        range_min="0.3"
        range_max="75"
        range_stddev="0.027"
        mesh="">
        <inertia ixx="0.00001" ixy="0.0" ixz="0.0" iyy="0.00001" iyz="0.0" izz="0.00001" />
        <xacro:insert_block name="origin" />
        <visual>
          <geometry>
            <mesh filename="file://$(find uuv_sensor_ros_plugins)/meshes/p900.dae" scale="1 1 1"/>
          </geometry>
        </visual>
      </xacro:multibeam_sonar>
    </xacro:macro>
```
 * Then we will need to change the sonar called in the URDF of the desitek sage :

 * go here : desistek_saga/desistek_saga_description/urdfcand
 * open : desistek_saga_sensors.xacro
 * remove the orignial forward sonar and repace it by :
```xml
  <!-- MSIS sonar sensor -->
  <xacro:micron_sonar namespace="${namespace}" parent_link="${namespace}/base_link">
    <origin xyz="0.0 0 -0.1" rpy="0 0 0"/>
  </xacro:micron_sonar>
```
![sonar2](https://github.com/Bluerov2/MASTER/blob/sonar_mapping/images/9e9dd76fd4f547150d948ba49b7f92b3_74108.jpeg)


## Launch

To launch the Simulation using UUV simulation, playground and robotmodel:

```
$ roslaunch sonar_mapping sim.launch
```

To launch a rosbag of the real sonar in a square tank :

```
$ roslaunch sonar_mapping bag.launch
```

## List of Task

- [x] IMU & DLV fused (/odom)
- [x] Scan-matching using ICP
- [x] Kalman Filter (Localisation)
- [x] Mapping
- [x] SLAM

## Thesis

https://github.com/Tim-HW/Tim-HW-BlueRov2_Sonar_based_SLAM-/blob/master/Heriot_Watt_University__HWU__CS_Masters_thesis_Sonar_based_SLAM_for_underwater_ROV.pdf
