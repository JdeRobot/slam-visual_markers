#include "kalmanfilter.h"
#include <sys/time.h>

using namespace Ardrone;

#define _ONLY_POSITION_ 1 //TODO Falta completar el filtro con la orientación

KalmanFilter::KalmanFilter()
{
    //s = [x y z r p y vx vy vz wr wp wy]
    //
    //    [1 0 0 0 0 0 t 0 0 0 0 0]
    //    [0 1 0 0 0 0 0 t 0 0 0 0]
    //A = [0 0 1 0 0 0 0 0 t 0 0 0]
    //    [0 0 0 1 0 0 0 0 0 t 0 0]
    //    [0 0 0 0 1 0 0 0 0 0 t 0]
    //    [0 0 0 0 0 1 0 0 0 0 0 t]
    //    [0 0 0 0 0 0 1 0 0 0 0 0]
    //    [0 0 0 0 0 0 0 1 0 0 0 0]
    //    [0 0 0 0 0 0 0 0 1 0 0 0]
    //    [0 0 0 0 0 0 0 0 0 1 0 0]
    //    [0 0 0 0 0 0 0 0 0 0 1 0]
    //    [0 0 0 0 0 0 0 0 0 0 0 1]

#if !_ONLY_POSITION_
    int stateSize = 12;
#else
    int stateSize = 6;
#endif

    //Identidad
    m_I = Eigen::MatrixXd::Identity(stateSize, stateSize);

    //Estado
    m_Xhat = Eigen::VectorXd(stateSize);
    for (int i = 0; i < stateSize; ++i)
    {
        m_Xhat(i) = 0;
    }

    //Relación entre estado actual y el anterior (por ahora posición: s = v*t)
    m_Dt = 0.08;
    m_A = Eigen::MatrixXd::Identity(stateSize, stateSize);
#if !_ONLY_POSITION_
    m_A(0,6) = m_Dt;
    m_A(1,7) = m_Dt;
    m_A(2,8) = m_Dt;
    m_A(3,9) = m_Dt;
    m_A(4,10) = m_Dt;
    m_A(5,11) = m_Dt;
#else
    m_A(0,3) = m_Dt;
    m_A(1,4) = m_Dt;
    m_A(2,5) = m_Dt;
#endif

    //Relación entre estado y medida
    //m_H = m_I;
    m_H = Eigen::MatrixXd::Identity(stateSize/2, stateSize);

    //Ruido del proceso
    double q = 0.01;
    m_Q = q*m_I;

    //Ruido de la medida
    double r = 0.0001;
    m_R = r*Eigen::MatrixXd::Identity(stateSize/2, stateSize/2);

    //Covarianza de error
    m_P = m_I;
}

KalmanFilter::~KalmanFilter()
{

}

double
KalmanFilter::SubstractAngles(double a, double b)
{
    a = GeometryUtils::StandardRad2(a);
    b = GeometryUtils::StandardRad2(b);

    double result = a - b;
    if (result > CV_PI)
    {
        result = -(result - 2*CV_PI);
    }
    else if (result < -CV_PI)
    {
        result = -(result + 2*CV_PI);
    }

    return result;
}

void
KalmanFilter::Reset()
{
    //TODO
}

Pose
KalmanFilter::GetFilteredPose(const Pose& zpose)
{
//    struct timeval start, end;
//    gettimeofday(&start, NULL);
//    printf("%u,%u\n", start.tv_sec, start.tv_usec/1000);

    //Predicción
    Eigen::VectorXd xhatminus = m_A*m_Xhat;
    /******************************************************************/
    xhatminus(3) = GeometryUtils::StandardRad(xhatminus(3));
    xhatminus(4) = GeometryUtils::StandardRad(xhatminus(4));
    xhatminus(5) = GeometryUtils::StandardRad(xhatminus(5));
    /******************************************************************/
    Eigen::MatrixXd Pminus = m_A*(m_P*m_A.transpose()) + m_Q;

    //Medida
#if !_ONLY_POSITION_
    Eigen::VectorXd z(6);
    Eigen::VectorXd auxz(6);
    z(0) = zpose.GetX();
    z(1) = zpose.GetY();
    z(2) = zpose.GetZ();
    z(3) = zpose.GetRoll();
    z(4) = zpose.GetPitch();
    z(5) = zpose.GetYaw();
#else
    Eigen::VectorXd z(3);
    Eigen::VectorXd auxz(3);
    z(0) = zpose.GetX();
    z(1) = zpose.GetY();
    z(2) = zpose.GetZ();
#endif

    //Corrección
    Eigen::MatrixXd Knum = Pminus*m_H.transpose();
    Eigen::MatrixXd Kden = m_H*Pminus*m_H.transpose() + m_R;
    Eigen::MatrixXd K = Knum*Kden.transpose().eval();
    m_Xhat = xhatminus + K*(z - m_H*xhatminus);
    /******************************************************************/
#if !_ONLY_POSITION_
    Eigen::VectorXd aux1 = m_H*xhatminus;
    aux1(3) = GeometryUtils::StandardRad(aux1(3));
    aux1(4) = GeometryUtils::StandardRad(aux1(4));
    aux1(5) = GeometryUtils::StandardRad(aux1(5));
    auxz = z - aux1;
    auxz(3) = SubstractAngles(z(3), aux1(3));
    auxz(4) = SubstractAngles(z(4), aux1(4));
    auxz(5) = SubstractAngles(z(5), aux1(5));
    Eigen::VectorXd aux2 = K*auxz; //TODO
    m_Xhat(3) = GeometryUtils::StandardRad(m_Xhat(3));
    m_Xhat(4) = GeometryUtils::StandardRad(m_Xhat(4));
    m_Xhat(5) = GeometryUtils::StandardRad(m_Xhat(5));
#endif
    /******************************************************************/
    m_P = (m_I - K*m_H)*Pminus;

    //printf("%.5f %.5f %.5f --> %.5f %.5f %.5f. %.5f %.5f, %.5f\n", z(6), z(7), z(8), m_Xhat(6), m_Xhat(7), m_Xhat(8), z(9), z(10), z(11));

#if !_ONLY_POSITION_
    return Pose(m_Xhat(0), m_Xhat(1), m_Xhat(2), 1, m_Xhat(3), m_Xhat(4), m_Xhat(5), zpose.GetBaseFoaX(), zpose.GetBaseFoaY(), zpose.GetBaseFoaZ());
#else
    return Pose(m_Xhat(0), m_Xhat(1), m_Xhat(2), 1, zpose.GetRoll(), zpose.GetPitch(), zpose.GetYaw(), zpose.GetBaseFoaX(), zpose.GetBaseFoaY(), zpose.GetBaseFoaZ());
#endif
}
