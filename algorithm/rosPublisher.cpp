#include "rosPublisher.h"

rosPublisher::rosPublisher(const std::string& topic)
{

	pub = n.advertise <geometry_msgs::Pose> (topic, 1);

}




void rosPublisher::setPose(const Ardrone::Pose p)
{
geometry_msgs::Pose Pose;


m = Eigen::AngleAxisf(p.GetYaw(), Eigen::Vector3f::UnitZ())
* Eigen::AngleAxisf(p.GetPitch(), Eigen::Vector3f::UnitY())
* Eigen::AngleAxisf(p.GetRoll(), Eigen::Vector3f::UnitX());
Eigen::Quaternionf quatFromRot2(m);
quatFromRot2.normalize();

Pose.position.x=p.GetX();
Pose.position.y=p.GetY();
Pose.position.z=p.GetZ();


Pose.orientation.x=quatFromRot2.x();
Pose.orientation.y=quatFromRot2.y();
Pose.orientation.z=quatFromRot2.z();
Pose.orientation.w=quatFromRot2.w();
pub.publish(Pose);

}
