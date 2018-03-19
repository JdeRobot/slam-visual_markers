# Installation
## JdeRobot
Following the instructions from JdeRobot page: [http://jderobot.org/Installation]

## Aruco
To install this dependency, I have followed the instructions from [http://miloq.blogspot.com.es/2012/12/install-aruco-ubuntu-linux.html]
and [https://sourceforge.net/projects/aruco/files/?source=navbar] to download the 2.0.9 version
## Compile
To install this component you have to copy the folder slam_markers and paste in tools from jderobot repository.
After that, you have to compile only the component slam_markers so you have to do:
> $ mkdir build && cd build

> $ cmake -Dbuild-default=OFF -Dbuild_slam_markers=ON ..

> $ make

> $ sudo make install

Then, you have the installed component and you can launch it.

# USAGE
To launch this component you need some files:
- "markers.txt": This file contains the pose markers to localizate the sensor
- configuration.yml: This file contains the following information:
	- The camera information (ICE proxy or ROS topic): 
	- The ICE proxy or ROS topic)  to publish the estimate pose 3d
	- The ROS topic to publish the number of detected markers.
	- The ROS topic to publish the time in which publish the estimated pose.
- sim_calib.yml: this file contains the calibration parameters of the camera.
> IMPORTANT: I have calibrated the camera with the fool from jderobot, so if you use other you have to change the names to camera_matrix and distortion_coefficients.

Anyways, to launch the component you have to type:
> $ slam_markers path_to_configuration.yml path_to_calibration_file.yml

Where you launch it, it must contains the files "markers.txt" and "ardrone_slam.glade".
