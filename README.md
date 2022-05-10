# step-by-step-multi-sensor-fusion

The aim of the project is to build a multi-sensor localization and mapping system. Its contents are related to the below toppics:
- `sensor assembling`
- `sensor testing` 
- `sychonization and calibration`
- `dataset recording`
- `main code of the multi-sensorlocalization and mapping system`

## sensor assembling

Four different sensors are utilized including:
- `GNSS receiver` ublox M8T
- `IMU` xsens-mti-g-710
- `camera` realsense D435
- `Lidar` VLP-16

`xsens-mti-g-710` is a GNSS/IMU sensor, which can outpus both GNSS position and IMU data.  `realsense D435` is stereo camera, which can output one RGB image, two IR images and one depth image at the same time, but only one RGB or IR will be used in this project.

The above sensors are assembled on a aluminum plate by screws.
<p align="center">
  <img src="./pic/sensor-assembling.jpeg" alt="drawing" width="600"/>
</p>
Their 2D(left) and 3D(right) body coordinate systems or are appoximate to: 
<p align="center">
  <img src="./pic/3d-2d-sensors.png" alt="drawing" width="300" />
</p>

## sensor testing

The ROS drivers of utilized sensors are installed and tested under the operation system `Ubuntu 18.04 + ROS melodic`.
-  [ublox driver](https://github.com/HKUST-Aerial-Robotics/ublox_driver) 
-  [xsens mti driver](http://wiki.ros.org/xsens_mti_driver)
-  [velodyne lidar driver](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
-  [realsense camera driver](https://github.com/IntelRealSense/realsense-ros)

There is also another [ublox ROS driver](https://github.com/KumarRobotics/ublox) maintained by KumarRobotics, which is more popular. The outputs topics from these two drivers are different, but both can be transferred to `RINEX` easily. Though, `xsens-mti-g-710` can provide GNSS position, the RAW GNSS measurments are not available, so `ublox M8T`is utilized.

## sensor calibration







