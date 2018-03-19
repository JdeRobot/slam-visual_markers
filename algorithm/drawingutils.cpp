#include "drawingutils.h"
#include "geometryutils.h"
#include <GL/gl.h>
#include <glut.h>

using namespace Ardrone;
using namespace cv;

HPoint2D
DrawingUtils::graficas2Opticas(double i, double j, double rows, double columns)
{
    //En este proyecto no hace falta pasar de gráficas a ópticas y viceversa
    HPoint2D result;
    //result.x = rows - 1 - j;
    //result.y = i;
    result.x = i/* - columns/2*/;
    result.y = j/* - rows/2*/;
    result.h = 1.0;
    return result;
}

HPoint2D
DrawingUtils::opticas2Graficas(double i, double j, double rows, double columns)
{
    //En este proyecto no hace falta pasar de gráficas a ópticas y viceversa
    HPoint2D result;
    //result.x = j;
    //result.y = rows - 1 - i;
    result.x = i/* + columns/2*/;
    result.y = j/* + rows/2*/;
    result.h = 1.0;
    return result;
}

void
DrawingUtils::drawLine(CvPoint3D32f orig, CvPoint3D32f dest, float width, CvPoint3D32f color)
{
    glColor3f(color.x, color.y, color.z);
    glLineWidth(width);
    glBegin(GL_LINES);
    glVertex3f(orig.x, orig.y, orig.z);
    glVertex3f(dest.x, dest.y, dest.z);
    glEnd();
}

void
DrawingUtils::drawSphere(CvPoint3D32f orig, float width, CvPoint3D32f color)
{
    glPushMatrix();
    glColor3f(color.x, color.y, color.z);
    glTranslatef(orig.x, orig.y, orig.z);
    glutSolidSphere(0.5, width, width);
    glPopMatrix();
}

void
DrawingUtils::drawAxis(CvPoint3D32f orig, CvPoint3D32f destX, CvPoint3D32f destY, CvPoint3D32f destZ, float width)
{
    drawLine(orig, destX, width, cvPoint3D32f(1.0, 0.0, 0.0));
    drawLine(orig, destY, width, cvPoint3D32f(0.0, 1.0, 0.0));
    drawLine(orig, destZ, width, cvPoint3D32f(0.0, 0.0, 1.0));
}

void
DrawingUtils::drawCamera(const TPinHoleCamera& cam, CvPoint3D32f color, const Eigen::Matrix3d& K, const Eigen::Matrix4d& RT)
{
    HPoint3D pyramid1, pyramid2, pyramid3, pyramid4, pyramidCenter;
    backproject(&pyramid1, graficas2Opticas(0, 0, cam.rows, cam.columns), cam); //Rayo de retroproyección de la esquina superior izquierda
    backproject(&pyramid2, graficas2Opticas(cam.columns, 0, cam.rows, cam.columns), cam);  //Rayo de retroproyección de la esquina superior derecha
    backproject(&pyramid3, graficas2Opticas(0, cam.rows, cam.rows, cam.columns), cam); //Rayo de retroproyección de la esquina inferior izquierda
    backproject(&pyramid4, graficas2Opticas(cam.columns, cam.rows, cam.rows, cam.columns), cam); //Rayo de retroproyección de la esquina inferior derecha
    backproject(&pyramidCenter, graficas2Opticas(cam.columns/2, cam.rows/2, cam.rows, cam.columns), cam); //Rayo de retroproyección del centro óptico
    HPoint3D center1 = Ardrone::GeometryUtils::GetPointOfLine(cam.position, pyramidCenter, cam.fdistx/2000.0);
    HPoint3D center2 = Ardrone::GeometryUtils::GetPointOfLine(cam.position, pyramidCenter, 1.0);
    HPoint3D center3 = Ardrone::GeometryUtils::GetPointOfLine(cam.position, pyramidCenter, 2.0);
    Line line(cam.position, center2);
    Plane plane1(center1, cam.position);
    Plane plane2(center2, cam.position);
    Line line1(cam.position, pyramid1);
    HPoint3D aux1 = Ardrone::GeometryUtils::GetPointOfLineAndPlane(line1, plane1);
    HPoint3D aux2 = Ardrone::GeometryUtils::GetPointOfLineAndPlane(line1, plane2);
    Line line2(cam.position, pyramid2);
    HPoint3D aux3 = Ardrone::GeometryUtils::GetPointOfLineAndPlane(line2, plane1);
    HPoint3D aux4 = Ardrone::GeometryUtils::GetPointOfLineAndPlane(line2, plane2);
    Line line3(cam.position, pyramid3);
    HPoint3D aux5 = Ardrone::GeometryUtils::GetPointOfLineAndPlane(line3, plane1);
    HPoint3D aux6 = Ardrone::GeometryUtils::GetPointOfLineAndPlane(line3, plane2);
    Line line4(cam.position, pyramid4);
    HPoint3D aux7 = Ardrone::GeometryUtils::GetPointOfLineAndPlane(line4, plane1);
    HPoint3D aux8 = Ardrone::GeometryUtils::GetPointOfLineAndPlane(line4, plane2);
    //Orientación
    drawLine(cvPoint3D32f(cam.position.X, cam.position.Y, cam.position.Z), cvPoint3D32f(center3.X, center3.Y, center3.Z), 2.0f, color);
    //Pirámide
    drawLine(cvPoint3D32f(aux1.X, aux1.Y, aux1.Z), cvPoint3D32f(aux2.X, aux2.Y, aux2.Z), 2.0f, color);
    drawLine(cvPoint3D32f(aux3.X, aux3.Y, aux3.Z), cvPoint3D32f(aux4.X, aux4.Y, aux4.Z), 2.0f, color);
    drawLine(cvPoint3D32f(aux5.X, aux5.Y, aux5.Z), cvPoint3D32f(aux6.X, aux6.Y, aux6.Z), 2.0f, color);
    drawLine(cvPoint3D32f(aux7.X, aux7.Y, aux7.Z), cvPoint3D32f(aux8.X, aux8.Y, aux8.Z), 2.0f, color);
    //Plano imagen
    drawLine(cvPoint3D32f(aux1.X, aux1.Y, aux1.Z), cvPoint3D32f(aux3.X, aux3.Y, aux3.Z), 2.0f, color);
    drawLine(cvPoint3D32f(aux3.X, aux3.Y, aux3.Z), cvPoint3D32f(aux7.X, aux7.Y, aux7.Z), 2.0f, color);
    drawLine(cvPoint3D32f(aux7.X, aux7.Y, aux7.Z), cvPoint3D32f(aux5.X, aux5.Y, aux5.Z), 2.0f, color);
    drawLine(cvPoint3D32f(aux5.X, aux5.Y, aux5.Z), cvPoint3D32f(aux1.X, aux1.Y, aux1.Z), 2.0f, color);
    //Plano
    drawLine(cvPoint3D32f(aux2.X, aux2.Y, aux2.Z), cvPoint3D32f(aux4.X, aux4.Y, aux4.Z), 2.0f, color);
    drawLine(cvPoint3D32f(aux4.X, aux4.Y, aux4.Z), cvPoint3D32f(aux8.X, aux8.Y, aux8.Z), 2.0f, color);
    drawLine(cvPoint3D32f(aux8.X, aux8.Y, aux8.Z), cvPoint3D32f(aux6.X, aux6.Y, aux6.Z), 2.0f, color);
    drawLine(cvPoint3D32f(aux6.X, aux6.Y, aux6.Z), cvPoint3D32f(aux2.X, aux2.Y, aux2.Z), 2.0f, color);
}


HPoint2D
DrawingUtils::myproject(HPoint3D point, const Eigen::Matrix3d& K, const Eigen::Matrix4d& RT)
{
    HPoint2D result;

    Eigen::Vector4d in(point.X/point.H, point.Y/point.H, point.Z/point.H, 1);
    Eigen::Vector4d a = RT*in;

    Eigen::Vector3d out = K*a.head(3);

    result.x = out(0)/out(2);
    result.y = out(1)/out(2);
    result.h = 1.0;

    return result;
}

HPoint3D
DrawingUtils::mybackproject(HPoint2D point, const Eigen::Matrix3d& K, const Eigen::Matrix4d& RT)
{
    //TODO NO FUNCIONA

    HPoint3D result;

    Eigen::Matrix3d ik;
    ik = K;
    ik = ik.inverse().eval();

    //std::cout << "punto: " << point(0) << ", " << point(1) << ", " << point(2) << std::endl;

    Eigen::Vector3d Pi(point.x/point.h, point.y/point.h, 1);

    Eigen::Vector3d a;
    a = ik*Pi;

    //std::cout << "a: " << a(0) << ", " << a(1) << ", " << a(2) << std::endl;

    Eigen::Vector4d aH;
    aH(0) = a(0);
    aH(1) = a(1);
    aH(2) = a(2);
    aH(3) = 1.0;

    Eigen::Matrix4d RT2;
    RT2 = RT;

    RT2(0, 3) = .0;
    RT2(1, 3) = .0;
    RT2(2, 3) = .0;
    RT2(3, 3) = 1.0;


    Eigen::Vector4d b;

    b = RT2.transpose()*aH;

    //std::cout << "b: " << b(0) << ", " << b(1) << ", " << b(2) << std::endl;

    Eigen::Matrix4d Translate;
    Translate.setIdentity();
    Translate(0, 3) = RT(0,3)/RT(3,3);
    Translate(1, 3) = RT(1,3)/RT(3,3);
    Translate(2, 3) = RT(2,3)/RT(3,3);

    b = Translate*b;

    /*OJO*/
    result.X = b(0)/b(3);
    result.Y = b(1)/b(3);
    result.Z = b(2)/b(3);
    result.H = 1.0/*b(3)*/;

    printf("UEEEEE %f, %f, %f\n", result.X, result.Y, result.Z);

    return result;
}
