#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include <std_msgs/Int8.h>



using namespace std;
class numMarkerPublisherROS {
private:

	ros::NodeHandle n;
	ros::Publisher pub;




public:
	numMarkerPublisherROS(const std::string& topic);
	void setNumMarker(std_msgs::Int8 n);


};
