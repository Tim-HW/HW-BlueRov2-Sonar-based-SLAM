# BlueRov2 SLAM


<p align="center">
<img src="https://github.com/Bluerov2/MASTER/blob/sonar_mapping/images/hh.jpg " width="400" >
</p>


The aim of this project is to provide a start of a 2D SLAM for underwater ROV. This project uses UUV simulation to fufill this objectif. The model is simulated using Deskitek Saga but you can use your own model as well. We modified the sonar to be se same as the [Micron Sonar](https://www.tritech.co.uk/product/small-rov-mechanical-sector-scanning-sonar-tritech-micron) from Tritech. The sonar based SLAM is acheived using an ICP coupled with a Kalman Filter.

<p align="center">
 <img src="https://github.com/Bluerov2/MASTER/blob/sonar_mapping/images/BlueROV2-4-lumen-1-300x300.png " width="400" >
</p>

## Video prentation
<p align="center">
<a href="(https://youtu.be/X4j5ylzuf_o
" target="_blank"><img src="https://github.com/Tim-HW/Tim-HW-BlueRov2_Sonar_based_SLAM-/blob/master/images/video.png" /></a>
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

