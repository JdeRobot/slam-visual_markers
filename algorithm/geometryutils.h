#ifndef GEOMETRYUTILS_H
#define GEOMETRYUTILS_H

#include <progeo/progeo.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <eigen3/Eigen/Dense>

namespace Ardrone
{
    class Pose
    {
    private:
        HPoint3D m_Position;
        HPoint3D m_BaseFoa;
        HPoint3D m_Foa;
        float m_Roll;
        float m_Pitch;
        float m_Yaw;
        Eigen::Matrix4d m_RT;
        double m_Weight;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Pose();
        Pose(float x, float y, float z, float h, float roll, float pitch, float yaw, float foax = 0.0f, float foay = 0.0f, float foaz = 0.0f);
        Pose(const Eigen::Matrix4d& rt);

        void SetBaseFoa(float foax, float foay, float foaz, float h = 1.0f);
        void Update(float x, float y, float z, float h, float roll, float pitch, float yaw);
        void Update(const Eigen::Matrix4d& rt);

        HPoint3D GetPosition() { return m_Position; }
        HPoint3D GetFoa() { return m_Foa; }
        float GetX() const { return m_Position.X; }
        float GetY() const { return m_Position.Y; }
        float GetZ() const { return m_Position.Z; }
        float GetH() const { return m_Position.H; }
        float GetRoll() const { return m_Roll; }
        float GetPitch() const { return m_Pitch; }
        float GetYaw() const { return m_Yaw; }
        float GetBaseFoaX() const { return m_BaseFoa.X; }
        float GetBaseFoaY() const { return m_BaseFoa.Y; }
        float GetBaseFoaZ() const { return m_BaseFoa.Z; }
        float GetBaseFoaH() const { return m_BaseFoa.H; }
        float GetFoaX() const { return m_Foa.X; }
        float GetFoaY() const { return m_Foa.Y; }
        float GetFoaZ() const { return m_Foa.Z; }
        float GetFoaH() const { return m_Foa.H; }

        const Eigen::Matrix4d& GetRT() { return m_RT; }

        double GetWeight() { return m_Weight; }
        void SetWeight(double weight) { m_Weight = weight; }
    };

    class Line
    {
    private:
        float x0, y0, z0;
        float u1, u2, u3;
        float h;

    public:
        Line(HPoint3D a, HPoint3D b)
        {
            /**** Ecuaciones paramétricas de la recta ****/
            // x = x0 + u1*t = a.X + (b.X-a.X)*t
            // y = y0 + u2*t = a.Y + (b.Y-a.Y)*t
            // z = z0 + u3*t = a.Z + (b.Z-a.Z)*t
            /*********************************************/

            x0 = a.X;
            y0 = a.Y;
            z0 = a.Z;
            u1 = b.X - a.X;
            u2 = b.Y - a.Y;
            u3 = b.Z - a.Z;
            h = b.H;
        }

        HPoint3D GetPoint(float t) const
        {
            HPoint3D result;

            result.X = x0 + u1*t;
            result.Y = y0 + u2*t;
            result.Z = z0 + u3*t;
            result.H = h;

            return result;
        }

        float X0() const { return x0; }
        float Y0() const { return y0; }
        float Z0() const { return z0; }
        float U1() const { return u1; }
        float U2() const { return u2; }
        float U3() const { return u3; }
    };

    class Plane
    {
    private:
        float a, b, c;
        float x0, y0, z0;
        float h;

    public:
        Plane(HPoint3D o, HPoint3D n)
        {
            /******** Ecuación normal plano ********/
            // o es un punto del plano y o-n el vector normal
            // A*(x-x0) + B*(y-y0) + C*(z-z0) = 0
            // (n.X - o.X)*(x-o.X) + (n.Y - o.Y)*(y-o.Y) + (n.Z - o.Z)*(z-o.Z) = 0
            /***************************************/

            a = n.X - o.X;
            b = n.Y - o.Y;
            c = n.Z - o.Z;
            x0 = o.X;
            y0 = o.Y;
            z0 = o.Z;
            h = n.H;
        }

        float A() const { return a; }
        float B() const { return b; }
        float C() const { return c; }
        float X0() const { return x0; }
        float Y0() const { return y0; }
        float Z0() const { return z0; }
    };

    class GeometryUtils
    {
    public:
        /**
         * Dada una recta calcula el punto correspondiente a una determinada distancia del punto a.
         * a y b determinan la recta
         */
        static HPoint3D GetPointOfLine(HPoint3D a, HPoint3D b, double dist);

        /**
         * Calcula el punto de intersección de una recta y un plano
         */
        static HPoint3D GetPointOfLineAndPlane(const Line& line, const Plane& plane);

        static Eigen::Matrix4d BuildRTMat(float x, float y, float z, float rotX, float rotY, float rotZ);

        static float quatToRoll(float qw, float qx, float qy, float qz);

        static float quatToPitch(float qw, float qx, float qy, float qz);

        static float quatToYaw(float qw, float qx, float qy, float qz);

        static double StandardRad(double t);
        static double StandardRad2(double t);

        /**
         * Convert rotation matrix to roll, pitch, yaw angles
         */
        static void RotationMatrixToRPY(const Eigen::Matrix3d& rot, double& roll, double& pitch, double& yaw);

        /**
         * Convert roll, pitch, yaw angles to rotation matrix
         */
        static void RPYToRotationMatrix(double roll, double pitch, double yaw, Eigen::Matrix3f& rot);

        static double GetDistance(double x, double y, double z);

        static double GetRadialError(const Pose& realPose, const Pose& estimatedPose);

        static double GetXError(const Pose& realPose, const Pose& estimatedPose);

        static double GetYError(const Pose& realPose, const Pose& estimatedPose);

        static double GetZError(const Pose& realPose, const Pose& estimatedPose);

        static double GetAngularError(const Pose& realPose, const Pose& estimatedPose);

        static double GetRollError(const Pose& realPose, const Pose& estimatedPose);

        static double GetPitchError(const Pose& realPose, const Pose& estimatedPose);

        static double GetYawError(const Pose& realPose, const Pose& estimatedPose);
    };
}

#endif // GEOMETRYUTILS_H
