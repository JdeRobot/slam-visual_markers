#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include <std_msgs/Float64.h>






using namespace std;
class timerPublisherROS {
private:

	ros::NodeHandle n;
	ros::Publisher pub;




public:
	timerPublisherROS(const std::string& topic);
	void setTime(double t);


};
