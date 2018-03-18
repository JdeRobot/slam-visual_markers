#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "sensors/sensors.h"
#include "algorithm/cameramanager.h"
#include "algorithm/markerinfo.h"
#include "algorithm/kalmanfilter.h"
#include "algorithm/weightedaveragefilter.h"
#include "april_tags/apriltag.h"
#include "algorithm/world.h"
#include "algorithm/IGraphichAlgorithmer.h"
#include "ardronedefines.h"
#include <QLineEdit>
#include "sharer.h"
#include "rosPublisher.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow,
                   public IGraphicAlgorithmer
{
    Q_OBJECT

private:
    cv::Mat m_Image;
    cv::Mat m_GreyImage;
    Ardrone::CameraManager* m_CameraManager;
    Ardrone::KalmanFilter* m_KalmanFilter;
    Ardrone::WeightedAverageFilter* m_WeightedAverageFilter;
    Ardrone::ETemporalFusionFilter m_TemporalFusionFilter;
    std::fstream m_CameraRegisterStream;
    cv::VideoCapture m_VideoCapture;
    World* m_World;
    QTimer* m_Timer;
    Sensors* m_Sensors;
    Eigen::Matrix4d m_RT;
    Sharer *sharer;
	rosPublisher *myPublisher;

	int m_option; //ICE or ROS
	std::string m_topic;
	std::string m_calibFile;



public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setSensors(Sensors* sensors);
	void setOption(int option, std::string);
	void setCalibFile(std::string calib_filename);

    void updateThreadGUI();

    virtual void RunGraphicAlgorithm();

Q_SIGNALS:
    void signal_updateGUI();

private Q_SLOTS:
    void updateGUI_recieved();
    void on_worldBtn_clicked();
    void on_tempFusionChk_toggled(bool checked);
    void on_kalmanRdb_toggled(bool checked);
    void on_weightRdb_toggled(bool checked);
    void on_errorBtn_clicked();
    void updatePoseInfo();
    void on_minusBtn_clicked();
    void on_plusBtn_clicked();
    void on_copyBtn_clicked();

private:

    Ui::MainWindow *ui;


private:
    void RegisterError();
    void SetNumberInTextBox(QLineEdit* txt, float number);
    void SetNumbersInTextBox(QLineEdit* txt, int number1, int number2);
};

#endif // MAINWINDOW_H
