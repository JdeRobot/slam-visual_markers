#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "geometryutils.h"

namespace Ardrone
{
    class KalmanFilter
    {
    private:
        double m_Dt;
        Eigen::VectorXd m_Xhat;
        Eigen::MatrixXd m_A;
        Eigen::MatrixXd m_H;
        Eigen::MatrixXd m_Q;
        Eigen::MatrixXd m_R;
        Eigen::MatrixXd m_P;
        Eigen::MatrixXd m_I;

    private:
        double SubstractAngles(double a, double b);

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        KalmanFilter();
        virtual ~KalmanFilter();

        void Reset();
        Pose GetFilteredPose(const Pose& zpose);
    };
}

#endif // KALMANFILTER_H
