#include "pose3di.h"

Pose3DI::Pose3DI()
{
    pose3dData = new jderobot::Pose3DData();
    sharer = Sharer::getInstance();
}

Pose3DI::~Pose3DI()
{
}

Ice::Int Pose3DI::setPose3DData(const jderobot::Pose3DDataPtr& pose3dData,
        const Ice::Current&)
{
    return 0;
}

jderobot::Pose3DDataPtr Pose3DI::getPose3DData(const Ice::Current&)
{
    structPose3D pose3d = sharer->getPose3D();
    pose3dData->x = pose3d.x;
    pose3dData->y = pose3d.y;
    pose3dData->z = pose3d.z;
    pose3dData->h = pose3d.h;
    pose3dData->q0 = pose3d.q0;
    pose3dData->q1 = pose3d.q1;
    pose3dData->q2 = pose3d.q2;
    pose3dData->q3 = pose3d.q3;
    return pose3dData;
}
