#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "drawingutils.h"
#include "april_tags/tag36h11.h"
#include "april_tags/matd.h"
#include "opencv2/videoio.hpp"

using namespace Ardrone;

#define WEIGHTED_AVERAGE_FILTER_SIZE 3

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    connect(this, SIGNAL(signal_updateGUI()), this, SLOT(updateGUI_recieved()));
    ui->setupUi(this);

    m_World = new World(this);  

   
    m_KalmanFilter = new KalmanFilter();
    m_WeightedAverageFilter = new WeightedAverageFilter(WEIGHTED_AVERAGE_FILTER_SIZE);

    //FILTER TYPE
    m_TemporalFusionFilter = TEMPORAL_FUSION_FILTER_NONE;

    //m_RT = GeometryUtils::BuildRTMat(0, -0.15, 0, 0, 0, 0);
	
	

	



//    time_t t = time(0);
//    struct tm* now = localtime(&t);
//    char dt[256];
//    sprintf(dt, "%d%02d%2d_%02d%02d", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday, now->tm_hour, now->tm_min);
//    std::string dtStr = dt;

    std::string file = "/tmp/camera_error.txt";
    bool exists = QFileInfo(file.c_str()).exists();
    m_CameraRegisterStream.open(file.c_str(), std::fstream::out | std::fstream::app);
    if (!exists)
    {
        m_CameraRegisterStream << "X\tY\tZ\tRoll\tPitch\tYaw\tDistance\tMRoll\tMPitch\tMYaw\tERadial\tEX\tEY\tEZ\tEAngular\tERoll\tEPitch\tEYaw" << std::endl;
    }

//    m_VideoCapture.open(0);
//    m_VideoCapture.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH); //TODO Parametrizar
//    m_VideoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT); //TODO Parametrizar
//    if (!m_VideoCapture.isOpened())
//    {
//        printf("Could not open video capture\n");
//        abort();
//    }

    m_Timer = new QTimer(this);
    m_Timer->setInterval(1000);
    connect(m_Timer, SIGNAL(timeout()), this, SLOT(updatePoseInfo()));
    m_Timer->start();
}

MainWindow::~MainWindow()
{
    m_Timer->stop();
    delete m_Timer;
    m_CameraRegisterStream.close();
    delete m_WeightedAverageFilter;
    delete m_KalmanFilter;
    delete m_CameraManager;
    delete ui;
}

void
MainWindow::updateThreadGUI()
{
    //m_VideoCapture >> m_Image;
    m_Image = m_Sensors->getImage();

    if (m_CameraManager->ProcessImage(m_Image)) //Si se han detectado balizas
    {
        /*
        if (m_TemporalFusionFilter == TEMPORAL_FUSION_FILTER_KALMAN)
        {
            Pose p = m_KalmanFilter->GetFilteredPose(m_CameraManager->GetEstimatedPose());
            m_CameraManager->SetEstimatedPose(p.GetX(), p.GetY(), p.GetZ(), p.GetH(), p.GetRoll(), p.GetPitch(), p.GetYaw());

        }
        else if (m_TemporalFusionFilter == TEMPORAL_FUSION_FILTER_WEIGHTED_AVERAGE)
        {
            Pose p = m_WeightedAverageFilter->GetFilteredPose(m_CameraManager->GetEstimatedPose());
            m_CameraManager->SetEstimatedPose(p.GetX(), p.GetY(), p.GetZ(), p.GetH(), p.GetRoll(), p.GetPitch(), p.GetYaw());
        }
        */


        //RegisterError();

        //PASAR LOS DOS FILTROS (TESTS)
        /*
        Pose p = m_WeightedAverageFilter->GetFilteredPose(m_CameraManager->GetEstimatedPose());
        m_CameraManager->SetEstimatedPose(p.GetX(), p.GetY(), p.GetZ(), p.GetH(), p.GetRoll(), p.GetPitch(), p.GetYaw());
        printf("--- %03f %03f %03f ---\n", p.GetX(), p.GetY(), p.GetZ());
        */
        Pose p = m_KalmanFilter->GetFilteredPose(m_CameraManager->GetEstimatedPose());
        m_CameraManager->SetEstimatedPose(p.GetX(), p.GetY(), p.GetZ(), p.GetH(), p.GetRoll(), p.GetPitch(), p.GetYaw());
        printf("--- %03f %03f %03f ---\n", p.GetX(), p.GetY(), p.GetZ());
        printf("------------------------------------------------------------");


        p = m_WeightedAverageFilter->GetFilteredPose(m_CameraManager->GetEstimatedPose());
                m_CameraManager->SetEstimatedPose(p.GetX(), p.GetY(), p.GetZ(), p.GetH(), p.GetRoll(), p.GetPitch(), p.GetYaw());
                printf("--- %03f %03f %03f ---\n", p.GetX(), p.GetY(), p.GetZ());
		
		numMarkers = m_CameraManager->getNumMarkersDetected();


		if (m_option == 2)
		{

		myPublisher->setPose(m_CameraManager->GetEstimatedPose());
		}
		
		else
		{
        sharer->setPose3D(m_CameraManager->GetEstimatedPose(), 1);
		}


		//publish num markers detected
		m_NumMarkerROS->setNumMarker(numMarkers);

		//publish time
		double t = ros::Time::now().toSec();
		m_TimerROS->setTime(t);
		
    }
    else
    {	

			if (m_option == 2)
			{
			//myPublisher->publishPose(0,0,0,0,0,0);
				myPublisher->setPose(m_CameraManager->GetEstimatedPose());
			}		
			else
			{
		    sharer->setPose3D(m_CameraManager->GetEstimatedPose(), 0);
			}
		//publish num markers detected
		numMarkers.data = 0;
		m_NumMarkerROS->setNumMarker(numMarkers);

    }

    if (ui->realMarkerChk->isChecked())
    {
        m_CameraManager->DrawProjectedRealMarker(m_Image, ui->markerIdTxt->text().toInt());
    }

    if (ui->arDemoChk->isChecked())
    {
        std::ifstream file("object.txt");
        std::string line;
        while (std::getline(file, line))
        {
            double x, y, z, r, g, b;
            std::stringstream aux(line);
            aux >> x;
            aux >> y;
            aux >> z;
            aux >> r;
            aux >> g;
            aux >> b;
            m_CameraManager->DrawEstimatedPoint(m_Image, cvPoint3D32f(x, y, z + 2), cv::Scalar(r, g, b));
        }
    }

    Q_EMIT signal_updateGUI();
}

void MainWindow::setSensors(Sensors* sensors)
{
    m_Sensors = sensors;
}

void MainWindow::setOption(int option, std::string topic)
{
	m_option = option;
	if (m_option==2)
	{	
		std::cout<<"ROS selected to publish pose3d"<<std::endl;
		myPublisher = new rosPublisher(topic);

	}
	else
	{
		sharer = Sharer::getInstance();
	}


}


void MainWindow::setNumMarkerPublisher(std::string topic)
{

	m_NumMarkerROS = new numMarkerPublisherROS(topic);


}

void MainWindow::setTimerPublisherROS(std::string topic)
{
	m_TimerROS = new timerPublisherROS(topic);


}









void MainWindow::setCalibFile(std::string calib_filename)
{
	m_calibFile = calib_filename;
 	m_CameraManager = new CameraManager(m_calibFile, 0, 0, 1); //(0, 0, 1) porque el eje óptico de la cámara es el eje z, me sirve para calcular el foa

}






void MainWindow::updateGUI_recieved()
{
    QImage imageQt = QImage((const unsigned char*)(m_Image.data),
                            m_Image.cols,
                            m_Image.rows,
                            m_Image.step,
                            QImage::Format_RGB888);
    ui->cameraLbl->setPixmap(QPixmap::fromImage(imageQt));
}

void MainWindow::RunGraphicAlgorithm()
{
    //Ejes de referencia del mundo
    DrawingUtils::drawAxis(cvPoint3D32f(0.0, 0.0, 0.0), cvPoint3D32f(1.0, 0.0, 0.0), cvPoint3D32f(0.0, 1.0, 0.0), cvPoint3D32f(0.0, 0.0, 1.0), 4.0f);

    //Balizas
    for (std::map<int, MarkerInfo*>::const_iterator iter = CameraManager::MARKERS.begin(); iter != CameraManager::MARKERS.end(); ++iter)
    {
        DrawingUtils::drawAxis(iter->second->GetPosition(), iter->second->GetAxisX(), iter->second->GetAxisY(), iter->second->GetAxisZ(), 2.0f);
    }

    //Cámara calculada
    DrawingUtils::drawCamera(m_CameraManager->GetEstimatedCamera(), cvPoint3D32f(0.0, 0.0, 1.0), m_CameraManager->GetIntrinsicsMatrix(), m_CameraManager->GetEstimatedPose().GetRT().inverse().eval());



    //COSAS MÍAS
    //CÁMARAS DE LAS BALIZAS
    std::vector<TPinHoleCamera, Eigen::aligned_allocator<TPinHoleCamera> >& BalCams = m_CameraManager->GetBalCameras();
    std::vector<Pose, Eigen::aligned_allocator<Pose> >& BalPos = m_CameraManager->GetBalPoses();
    for (int i = 0; i < BalCams.size(); ++i)
    {
        printf("hello");
        DrawingUtils::drawCamera(BalCams[i], cvPoint3D32f(0.5, 0.5, 1.0), m_CameraManager->GetIntrinsicsMatrix(), BalPos[i].GetRT().inverse().eval());
    }


    //Cámara real
    //DrawingUtils::drawCamera(m_CameraManager->GetRealCamera(), cvPoint3D32f(1.0, 0.0, 0.0), m_CameraManager->GetIntrinsicsMatrix(), m_CameraManager->GetRealPose().GetRT().inverse().eval());
}

void MainWindow::RegisterError()
{
    Pose rp = m_CameraManager->GetRealPose(); //Real pose
    Pose ep = m_CameraManager->GetEstimatedPose(); //Estimated pose
    double realDistance;
    if (m_CameraManager->GetRealDistanceToMarker(0, realDistance))
    {
        double realRoll, realPitch, realYaw;
        if (m_CameraManager->GetRealAnglesToMarker(0, realRoll, realPitch, realYaw))
        {
            //printf("V: ROLL %f PITCH %f YAW %f\n", realRoll, realPitch, realYaw);
            double radialError = GeometryUtils::GetRadialError(rp, ep);
            double xError = GeometryUtils::GetXError(rp, ep);
            double yError = GeometryUtils::GetYError(rp, ep);
            double zError = GeometryUtils::GetZError(rp, ep);
            double angularError = GeometryUtils::GetAngularError(rp, ep);
            double rollError = GeometryUtils::GetRollError(rp, ep);
            double pitchError = GeometryUtils::GetPitchError(rp, ep);
            double yawError = GeometryUtils::GetYawError(rp, ep);
            m_CameraRegisterStream << rp.GetX() << "\t" << rp.GetY() << "\t" << rp.GetZ() << "\t";
            m_CameraRegisterStream << rp.GetRoll() << "\t" << rp.GetPitch() << "\t" << rp.GetYaw() << "\t";
            m_CameraRegisterStream << realDistance << "\t" << realRoll << "\t" << realPitch << "\t" << realYaw << "\t";
            m_CameraRegisterStream << radialError << "\t" << xError << "\t" << yError << "\t" << zError << "\t";
            m_CameraRegisterStream << angularError << "\t" << rollError << "\t" << pitchError << "\t" << yawError << std::endl;
        }
    }
}

void MainWindow::SetNumberInTextBox(QLineEdit* txt, float number)
{
//    std::stringstream aux;
//    aux.precision(3);
//    aux << number;
    char aux[32];
    sprintf(aux, "%.3f", number);
    std::string str;
    str.assign(aux);
    txt->setText(QString::fromStdString(str.c_str()));
}

void MainWindow::SetNumbersInTextBox(QLineEdit* txt, int number1, int number2)
{
    std::stringstream aux;
    aux << number1 << "," << number2;
    txt->setText(QString::fromStdString(aux.str()));
}

void MainWindow::on_worldBtn_clicked()
{
    if (ui->worldBtn->text() == "Show World")
    {
        ui->worldBtn->setText("Hide World");
        m_World->show();
    }
    else
    {
        ui->worldBtn->setText("Show World");
        m_World->hide();
    }
}

void MainWindow::on_tempFusionChk_toggled(bool checked)
{
    ui->kalmanRdb->setEnabled(checked);
    ui->weightRdb->setEnabled(checked);

    if (!checked)
    {
        m_TemporalFusionFilter = TEMPORAL_FUSION_FILTER_NONE;
    }
    else if (ui->kalmanRdb->isChecked())
    {
        m_TemporalFusionFilter = TEMPORAL_FUSION_FILTER_KALMAN;
    }
    else if (ui->weightRdb->isChecked())
    {
        m_TemporalFusionFilter = TEMPORAL_FUSION_FILTER_WEIGHTED_AVERAGE;
    }
}

void MainWindow::on_kalmanRdb_toggled(bool checked)
{
    if (checked)
    {
        m_TemporalFusionFilter = TEMPORAL_FUSION_FILTER_KALMAN;
    }
}

void MainWindow::on_weightRdb_toggled(bool checked)
{
    if (checked)
    {
        m_TemporalFusionFilter = TEMPORAL_FUSION_FILTER_WEIGHTED_AVERAGE;
    }
}

void MainWindow::on_errorBtn_clicked()
{
    RegisterError();
}

void MainWindow::updatePoseInfo()
{
    m_CameraManager->SetRealPose(ui->xRealTxt->text().toDouble(),
                                 ui->yRealTxt->text().toDouble(),
                                 ui->zRealTxt->text().toDouble(),
                                 1.0,
                                 ui->rollRealTxt->text().toDouble(),
                                 ui->pitchRealTxt->text().toDouble(),
                                 ui->yawRealTxt->text().toDouble());

    Pose ep = m_CameraManager->GetEstimatedPose();
    Pose rp = m_CameraManager->GetRealPose();

    SetNumberInTextBox(ui->xEstimTxt, ep.GetX());
    SetNumberInTextBox(ui->yEstimTxt, ep.GetY());
    SetNumberInTextBox(ui->zEstimTxt, ep.GetZ());
    SetNumberInTextBox(ui->rollEstimTxt, ep.GetRoll());
    SetNumberInTextBox(ui->pitchEstimTxt, ep.GetPitch());
    SetNumberInTextBox(ui->yawEstimTxt, ep.GetYaw());

    SetNumberInTextBox(ui->xErrorTxt, GeometryUtils::GetXError(rp, ep));
    SetNumberInTextBox(ui->yErrorTxt, GeometryUtils::GetYError(rp, ep));
    SetNumberInTextBox(ui->zErrorTxt, GeometryUtils::GetZError(rp, ep));
    SetNumberInTextBox(ui->rollErrorTxt, GeometryUtils::GetRollError(rp, ep));
    SetNumberInTextBox(ui->pitchErrorTxt, GeometryUtils::GetPitchError(rp, ep));
    SetNumberInTextBox(ui->yawErrorTxt, GeometryUtils::GetYawError(rp, ep));
    SetNumberInTextBox(ui->radialErrorTxt, GeometryUtils::GetRadialError(rp, ep));
    SetNumberInTextBox(ui->angularErrorTxt, GeometryUtils::GetAngularError(rp, ep));

    if (ui->realMarkerChk->isChecked())
    {
        std::vector<cv::Point2f> diffs = m_CameraManager->GetCornerDiffs(ui->markerIdTxt->text().toInt());
        if (diffs.size() == 4)
        {
            SetNumbersInTextBox(ui->corner0Txt, diffs[0].x, diffs[0].y);
            SetNumbersInTextBox(ui->corner1Txt, diffs[1].x, diffs[1].y);
            SetNumbersInTextBox(ui->corner2Txt, diffs[2].x, diffs[2].y);
            SetNumbersInTextBox(ui->corner3Txt, diffs[3].x, diffs[3].y);
        }
        else
        {
            ui->corner0Txt->setText("");
            ui->corner1Txt->setText("");
            ui->corner2Txt->setText("");
            ui->corner3Txt->setText("");
        }
    }
}

void MainWindow::on_minusBtn_clicked()
{
    QLineEdit* le = NULL;
    if (ui->xRdb->isChecked())
    {
        le = ui->xRealTxt;
    }
    else if (ui->yRdb->isChecked())
    {
        le = ui->yRealTxt;
    }
    else if (ui->zRdb->isChecked())
    {
        le = ui->zRealTxt;
    }
    else if (ui->rollRdb->isChecked())
    {
        le = ui->rollRealTxt;
    }
    else if (ui->pitchRdb->isChecked())
    {
        le = ui->pitchRealTxt;
    }
    else if (ui->yawRdb->isChecked())
    {
        le = ui->yawRealTxt;
    }

    if (le)
    {
        SetNumberInTextBox(le, le->text().toDouble() - 0.001);
    }
}

void MainWindow::on_plusBtn_clicked()
{
    QLineEdit* le = NULL;
    if (ui->xRdb->isChecked())
    {
        le = ui->xRealTxt;
    }
    else if (ui->yRdb->isChecked())
    {
        le = ui->yRealTxt;
    }
    else if (ui->zRdb->isChecked())
    {
        le = ui->zRealTxt;
    }
    else if (ui->rollRdb->isChecked())
    {
        le = ui->rollRealTxt;
    }
    else if (ui->pitchRdb->isChecked())
    {
        le = ui->pitchRealTxt;
    }
    else if (ui->yawRdb->isChecked())
    {
        le = ui->yawRealTxt;
    }

    if (le)
    {
        SetNumberInTextBox(le, le->text().toDouble() + 0.001);
    }
}

void MainWindow::on_copyBtn_clicked()
{
    ui->xRealTxt->setText(ui->xEstimTxt->text());
    ui->yRealTxt->setText(ui->yEstimTxt->text());
    ui->zRealTxt->setText(ui->zEstimTxt->text());
    ui->rollRealTxt->setText(ui->rollEstimTxt->text());
    ui->pitchRealTxt->setText(ui->pitchEstimTxt->text());
    ui->yawRealTxt->setText(ui->yawEstimTxt->text());
}
