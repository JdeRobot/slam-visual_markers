#ifndef SHARER_H
#define SHARER_H

#include <pthread.h>
#include <stdlib.h>
#include <jderobot/pose3d.h>
#include "geometryutils.h"

/// Struct of Pose3D.
struct structPose3D {
    double x;
    double y;
    double z;
    double h;
    double q0;
    double q1;
    double q2;
    double q3;
};

class Sharer
{

public:

    /// Default destructor.
    virtual ~Sharer();

    /// Method for controlling the access to the single instanceof this class with singleton pattern.
    static Sharer* getInstance();

    structPose3D getPose3D();

    void setPose3D(const Ardrone::Pose p, double markersDetected);

private:

    /// Default constructor.
    Sharer();	/// Private so it can not be called.

    /** Delete the sharer instance */
    static void DestroySharer();

    static Sharer* pInstance; /// Pointer to the single instance of this class with singleton pattern.

    structPose3D pose3d; /// Current Pose3D.

    pthread_mutex_t synchPose3D; /// Mutex for thread-safe access to pose3d related data.
};

#endif // SHARER_H
