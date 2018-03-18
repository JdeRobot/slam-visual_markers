#include "sharer.h"
//#include <Eigen/Geometry>
//#include <Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <cmath>
#include <iostream>

using std::max;
using std::min;

Sharer* Sharer::pInstance = NULL; /* Initializes the pointer. */

Sharer::Sharer()
{
    pthread_mutex_init(&this->synchPose3D, NULL);
}

Sharer::~Sharer()
{
    pthread_mutex_destroy(&this->synchPose3D);
}

Sharer* Sharer::getInstance()
{
    if ( pInstance == NULL ) // Only allow one instance of class to be generated.
    {
        pInstance = new Sharer();
        atexit(&DestroySharer);		// At exit, destroy the sharer.
    }
    return pInstance;
}

void Sharer::DestroySharer()
{
    if ( pInstance != NULL )
        delete pInstance;
}

structPose3D Sharer::getPose3D()
{
    pthread_mutex_lock(&this->synchPose3D);
    structPose3D retVal = this->pose3d;
    pthread_mutex_unlock(&this->synchPose3D);

    return retVal;
}

void Sharer::setPose3D(Ardrone::Pose p, double markersDetected)
{
    Eigen::Matrix3f m2;
    m2 = Eigen::AngleAxisf(p.GetYaw(), Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(p.GetPitch(), Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(p.GetRoll(), Eigen::Vector3f::UnitX());
    Eigen::Quaternionf quatFromRot2(m2);
    quatFromRot2.normalize();
    //std::cout << "--Quaternion from roll, pitch, yaw" << std::endl;
    //std::cout << "q0: " << quatFromRot2.w() << " q1: " << quatFromRot2.x() << " q2: " << quatFromRot2.y() << " q3: " << quatFromRot2.z() << std::endl;

//    double roll2, pitch2, yaw2;
//    Eigen::Matrix3f m3;
//    m3 = Eigen::AngleAxisf(p.GetYaw(), Eigen::Vector3f::UnitZ())
//    * Eigen::AngleAxisf(p.GetPitch(), Eigen::Vector3f::UnitY())
//    * Eigen::AngleAxisf(p.GetRoll(), Eigen::Vector3f::UnitX());
//    Eigen::Matrix3d rot;
//    rot(0,0)=m3(0,0);
//    rot(0,1)=m3(0,1);
//    rot(0,2)=m3(0,2);
//    rot(1,0)=m3(1,0);
//    rot(1,1)=m3(1,1);
//    rot(1,2)=m3(1,2);
//    rot(2,0)=m3(2,0);
//    rot(2,1)=m3(2,1);
//    rot(2,2)=m3(2,2);
//    Ardrone::GeometryUtils::RotationMatrixToRPY(rot, roll2, pitch2, yaw2);
//    std::cout << "--Rot XYZ order: " << std::endl;
//    std::cout << "roll: " << roll2 << " pitch: " << pitch2 << " yaw: " << yaw2 << std::endl;

//    double roll3, pitch3, yaw3;
//    Eigen::Matrix3f m4;
//    m4 = Eigen::AngleAxisf(p.GetRoll(), Eigen::Vector3f::UnitX())
//    * Eigen::AngleAxisf(p.GetPitch(), Eigen::Vector3f::UnitY())
//    * Eigen::AngleAxisf(p.GetYaw(), Eigen::Vector3f::UnitZ());
//    Eigen::Matrix3d rot2;
//    rot2(0,0)=m4(0,0);
//    rot2(0,1)=m4(0,1);
//    rot2(0,2)=m4(0,2);
//    rot2(1,0)=m4(1,0);
//    rot2(1,1)=m4(1,1);
//    rot2(1,2)=m4(1,2);
//    rot2(2,0)=m4(2,0);
//    rot2(2,1)=m4(2,1);
//    rot2(2,2)=m4(2,2);
//    Ardrone::GeometryUtils::RotationMatrixToRPY(rot2, roll3, pitch3, yaw3);
//    std::cout << "--Rot ZYX order: " << std::endl;
//    std::cout << "roll: " << roll3 << " pitch: " << pitch3 << " yaw: " << yaw3 << std::endl;

    pthread_mutex_lock(&this->synchPose3D);
    this->pose3d.x = p.GetX();
    this->pose3d.y = p.GetY();
    this->pose3d.z = p.GetZ();
    this->pose3d.h = markersDetected;
    this->pose3d.q0 = quatFromRot2.w();
    this->pose3d.q1 = quatFromRot2.x();
    this->pose3d.q2 = quatFromRot2.y();
    this->pose3d.q3 = quatFromRot2.z();
    pthread_mutex_unlock(&this->synchPose3D);
}
