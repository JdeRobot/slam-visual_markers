#ifndef POSE3DI_H
#define POSE3DI_H

#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <jderobot/pose3d.h>
#include "sharer.h"

class Pose3DI : virtual public jderobot::Pose3D
{

    public:

        Pose3DI();

        virtual ~Pose3DI();

        virtual Ice::Int setPose3DData(const jderobot::Pose3DDataPtr& pose3dData, const Ice::Current&);

        virtual jderobot::Pose3DDataPtr getPose3DData(const Ice::Current&);

    private:

        jderobot::Pose3DDataPtr pose3dData;

        Sharer *sharer;

};


#endif // POSE3DI_H
