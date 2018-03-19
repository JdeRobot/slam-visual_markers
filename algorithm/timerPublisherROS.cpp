#include "timerPublisherROS.h"

timerPublisherROS::timerPublisherROS(const std::string& topic)
{

	pub = n.advertise <std_msgs::Float64> (topic, 1);

}




void timerPublisherROS::setTime(double t)
{

std_msgs::Float64 tfloat;
tfloat.data = t;
pub.publish(tfloat);

}
