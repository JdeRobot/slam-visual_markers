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
