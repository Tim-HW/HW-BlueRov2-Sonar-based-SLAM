# BlueRov2 SLAM


WORK IN PROGRESS ! 

<p align="center">
<img src="https://github.com/Bluerov2/MASTER/blob/sonar_mapping/images/hh.jpg " width="400" >
</p>


The aim of this project is to provide a start of a 3D SLAM for underwater ROV. This project uses UUV simulation to fufill this objectif. The model is simulated using Deskitek Saga but you can use your own model as well. We modified the sonar to be se same as the [Micron Sonar](https://www.tritech.co.uk/product/small-rov-mechanical-sector-scanning-sonar-tritech-micron) from Tritech.

<p align="center">
 <img src="https://github.com/Bluerov2/MASTER/blob/sonar_mapping/images/BlueROV2-4-lumen-1-300x300.png " width="400" >
</p>



## Related Packages

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

you will need to change some URDF code from the ROV to allow a 360° vision.
for this run the following commands:

```
$ roscd uuv_simulator
$ cd ..
$ cd uuv_sensor_plugins/uuv_sensor_ros_plugins/urdf
$ sudo gedit sonar_snippets.xacro
```
and change the forward_multibeam_p900 with those new values.
```xml
<xacro:macro name="forward_multibeam_p900" params="namespace parent_link *origin">
    <xacro:multibeam_sonar
      namespace="${namespace}"
      suffix=""
      parent_link="${parent_link}"
      topic="sonar"
      mass="0.02"
      update_rate="15"
      samples="200"
      fov="6.0"
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
![sonar2](https://github.com/Bluerov2/MASTER/blob/sonar_mapping/images/9e9dd76fd4f547150d948ba49b7f92b3_74108.jpeg)

## Octomap launching

before launching anything you will have to do some modification:
We call ocotmap server 50 seconds after the beginning of the simulation. we are doing this because for some reason
the first 360 value of the sonar are drifted. Therefore, we launch a python file that will wait for 50 seconds before launching octomap.launch. but to achived this we will have to change the path of the file:

```unix
$ roscd sonar_mapping
$ cd src
$ sudo gedit octomap_launch.py
```
change the path:

>"/home/tim/bluerov_ws/src/sonar_mapping/launch/octomap.launch"

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

- [x] UUV simulation senario

Static SLAM:

- [x] IMU & DLV fused (/odom)
- [x] Scan-matching using ICP
- [ ] Kalman Filter (Localisation)
- [ ] Mapping

