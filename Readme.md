# Installation
To install this component you have to set the permissions to the file "installation.sh" with sudo chmod 777 installation.sh and launch it: ./installation.sh. This file will install the library Aruco (2.0.9 version), jderobot dependencies and the component.


# USAGE
To launch this component you need some files:
- "markers.txt": This file contains the pose markers to localizate the sensor
- config.yml: This file contains the information to receive the camera information (ICE proxy or ROS topic), to publish the estimate pose 3d(ICE or ROS topic), number of detected markers (ROS topic) and the time of these detections.
- sim_calib.yml: this file contains the calibration parameters of the camera.
> IMPORTANT: I have calibrated the camera with the jderobot tool, so if you use other you have to change the names to camera_matrix and distortion_coefficients.

Anyways, to launch the component you have to type:
> $ visual_markers config.yml sim_calib.yml

Where you launch it, it must contains the files "markers.txt" and glade files.
