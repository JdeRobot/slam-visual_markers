#include "numMarkerPublisherROS.h"

numMarkerPublisherROS::numMarkerPublisherROS(const std::string& topic)
{

	pub = n.advertise <std_msgs::Int8>(topic, 1);

}




void numMarkerPublisherROS::setNumMarker(std_msgs::Int8 n)
{

pub.publish(n);

}
