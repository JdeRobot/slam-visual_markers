#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <eigen3/Eigen/Geometry>
#include <jderobot/pose3d.h>
#include "geometryutils.h"





using namespace std;
class rosPublisher {
private:

	ros::NodeHandle n;
	ros::Publisher pub;
	geometry_msgs::Pose pose3d;
	Eigen::Matrix3f m;


public:
	rosPublisher(const std::string& topic);
	void setPose(const Ardrone::Pose p);


};
