#include "configurationhelperwindow.h"
#include "ui_configurationhelperwindow.h"

#include <QDebug>


QString m_cameraModeAttrName = "CameraAttributes::CameraControls::ModeAndAlgorithmControls::CameraMode";
QString m_exposureTimeAttrName = "CameraAttributes::CameraControls::SensorControls::ExposureTimeAbs";
QString m_framePeriodeAttrName = "CameraAttributes::CameraControls::SensorControls::FramePeriode";
QString m_thresholdAttrName = "CameraAttributes::CameraControls::AOIs::AoiThreshold";
QString m_multipleSlopeModeAttrName = "CameraAttributes::CameraControls::SensorControls::MultipleSlopeMode";
QString m_dualSlopeTimeAttrName = "CameraAttributes::CameraControls::SensorControls::DualSlopeTime";
QString m_tripleSlopeTimeAttrName = "CameraAttributes::CameraControls::SensorControls::TripleSlopeTime";
QString m_vLow2AttrName = "CameraAttributes::CameraControls::SensorControls::AdvancedSensorsettings::Vlow2";
QString m_vLow3AttrName = "CameraAttributes::CameraControls::SensorControls::AdvancedSensorsettings::Vlow3";
QString m_vRamp1AttrName = "CameraAttributes::CameraControls::SensorControls::AdvancedSensorsettings::Vramp1";
QString m_vRamp2AttrName = "CameraAttributes::CameraControls::SensorControls::AdvancedSensorsettings::Vramp2";


ConfigurationHelperWindow::ConfigurationHelperWindow(Z3D::LTSCamera3D::WeakPtr camera3D,
        QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ConfigurationHelperWindow),
    m_camera3D(camera3D)
{
    ui->setupUi(this);

    m_camera = m_camera3D->camera2d()->camera().data();

    m_camera3D->stopAcquisition();

    /// connect directly to camera signal
    QObject::connect(m_camera.data(), SIGNAL(newImageReceived(Z3D::ZImageGrayscale::Ptr)),
                     this, SLOT(onNewImageReceived(Z3D::ZImageGrayscale::Ptr))); //, Qt::DirectConnection);

    m_camera3D->setAcquisitionMode(Z3D::LTSCamera3D::ProfileAcquisitionMode);

    /// we want to see the image of the profiles, but everything else exactly as profile mode
    m_camera->setAttribute(m_cameraModeAttrName, "Image Mode (IMG)");

    /// to limit fps
    m_camera->setAttribute(m_framePeriodeAttrName, ui->framePeriodeSpinBox->value());

    updateValues();

    m_camera3D->startAcquisition();
}

ConfigurationHelperWindow::~ConfigurationHelperWindow()
{
    m_camera3D->stopAcquisition();
    m_camera3D->setAcquisitionMode(Z3D::LTSCamera3D::ProfileAcquisitionMode);

    delete ui;
}

void ConfigurationHelperWindow::onNewImageReceived(Z3D::ZImageGrayscale::Ptr image)
{
    cv::Mat cvimg = image->cvMat();

    /// calculate laser line
    std::vector<unsigned int> massDotPositionSum;
    massDotPositionSum.resize(cvimg.cols);

    std::vector<unsigned long> massSum;
    massSum.resize(cvimg.cols);

    int m_currentThreshold = ui->thresholdSpinBox->value();

    /// calculate center of gravity for points above threshold
    if (cvimg.type() == CV_16UC1) {
        for (int y=0; y<cvimg.rows; ++y) {
            unsigned short *rowBegin = (unsigned short *) cvimg.ptr(y);
            for (int x=0; x<cvimg.cols; ++x) {
                if (*rowBegin > m_currentThreshold) {
                    massDotPositionSum[x] += (*rowBegin) * y;
                    massSum[x] += (*rowBegin);
                }

                /// next col
                rowBegin++;
            }
        }
    } else if (cvimg.type() == CV_8UC1) {
        for (int y=0; y<cvimg.rows; ++y) {
            unsigned char *rowBegin = (unsigned char *) cvimg.ptr(y);
            for (int x=0; x<cvimg.cols; ++x) {
                if (*rowBegin > m_currentThreshold) {
                    massDotPositionSum[x] += (*rowBegin) * y;
                    massSum[x] += (*rowBegin);
                }

                /// next col
                rowBegin++;
            }
        }
    } else {
        qWarning() << "unsupported image type" << cvimg.type();
        return;
    }

    int profileVisibleWidth = 30;
    cv::Mat cvLaserImg = cv::Mat::zeros(cv::Size(cvimg.cols, profileVisibleWidth+20), CV_8UC1);
    int middleRow = cvLaserImg.rows / 2;

    //! TODO este 4 no deberia estar hardcodeado!
    /// es porque el maximo que da la imagen en 16 bits es 1024 y lo queremos en 8bit = 256

    int divisor;
    if (cvimg.type() == CV_16UC1)
        divisor = 4;
    else
        divisor = 1;

    /// draw centered profiles
    for (int x=0; x<cvimg.cols; ++x) {
        if (massSum[x] > 0) {
            int y = ((float)massDotPositionSum[x]) / massSum[x];

            //cvLaserImg.at<unsigned char>(y,x) = 0;
            for (int y2 = -profileVisibleWidth/2; y2<=profileVisibleWidth/2; ++y2) {
                if (middleRow+y2 < 0
                        || middleRow+y2 > cvLaserImg.rows
                        || y+y2 < 0
                        || y+y2 > cvimg.rows)
                    continue;

                if (cvimg.type() == CV_16UC1)
                    cvLaserImg.at<unsigned char>(middleRow+y2,x) = cvimg.at<unsigned short>(y+y2,x) / divisor;
                else if (cvimg.type() == CV_8UC1)
                    cvLaserImg.at<unsigned char>(middleRow+y2,x) = cvimg.at<unsigned char>(y+y2,x);
            }
        }
    }

    /// display centered laser profiles
    ui->centeredProfileImageWidget->setImage(cvLaserImg);


    /// calculate histogram
    int binSize = 2;
    cv::Mat histogram = cv::Mat::zeros(256/binSize, 64, CV_16UC1);
    for (int y=0; y<cvLaserImg.rows; ++y) {
        for (int x=0; x<cvLaserImg.cols; ++x) {
            int window = cvLaserImg.at<unsigned char>(y,x);
            if (window > m_currentThreshold/divisor) {
                /// skip values below threshold, we don't want to see them
                window /= binSize;
                histogram.at<unsigned short>(histogram.rows-1-window,y) += 1;
            }
        }
    }

    /// convert to "visible" image to show in window
    double minVal, maxVal;
    cv::minMaxIdx(histogram, &minVal, &maxVal);

    //qDebug() << "min" << minVal << "max" << maxVal;

    cv::Mat histogramImage;
    histogram.convertTo(histogramImage, CV_16UC1, 255.0/maxVal, 0);

    ui->profileHistogramImageWidget->setImage(histogramImage);


/*
    /// are we calculating ok?
    for (int x=0; x<cvimg.cols; ++x) {
        if (massSum[x] > 0) {
            int y = ((float)massDotPositionSum[x]) / massSum[x];

            /// draw over normal image
            if (cvimg.type() == CV_16UC1)
                cvimg.at<unsigned short>(y,x) = 0;
            else if (cvimg.type() == CV_8UC1)
                cvimg.at<unsigned char>(y,x) = 0;
        }
    }
*/

    /// display normal image
    /// if type is 16 bits per pixel, the camera only uses 10
    if (cvimg.type() == CV_16UC1) {
        cvimg /= divisor;
    }

    ui->currentImageWidget->setImage(cvimg);



    /// calculate average every X centered profiles
    int avgWinWidth = ui->avgWindowWidthSpinBox->value();
    int cols = std::ceil((float)cvimg.cols/avgWinWidth);

    //qDebug() << cols << "x" << cvLaserImg.rows;

    cols += (cols % 4)>0 ? 4 - (cols % 4) : 0; /// always multiple of 4, for memory alignment between rows!
    cv::Mat meanProfile = cv::Mat::zeros(cv::Size(cols, cvLaserImg.rows), CV_8UC1);

    for (int y=0; y<meanProfile.rows; ++y) {
        for (int ix=0; ix<meanProfile.cols; ++ix) {
            int startx = ix * avgWinWidth;
            int counter = 0;
            int sum = 0;
            for (int x3=0; x3<avgWinWidth; ++x3) {
                if (x3 > cvLaserImg.cols)
                    break;

                int x = startx + x3;
                if (massSum[x] > 0) {
                    counter++;
                    sum += (int) cvLaserImg.at<unsigned char>(y,x);
                }
            }
            if (counter > 0)
                meanProfile.at<unsigned char>(y,ix) = (unsigned char) ((float)sum/counter);
        }
    }

    ui->profileHelperImageWidget->setImage(meanProfile);
}

void ConfigurationHelperWindow::updateValues()
{
    ui->framePeriodeSpinBox->setValue( m_camera->getAttribute(m_framePeriodeAttrName).toInt() );
    ui->multiSlopeModeComboBox->setEditText( m_camera->getAttribute(m_multipleSlopeModeAttrName).toString() );
    ui->exposureTimeSpinBox->setValue( m_camera->getAttribute(m_exposureTimeAttrName).toInt() );
    ui->dualSlopeTimeSpinBox->setValue( m_camera->getAttribute(m_dualSlopeTimeAttrName).toInt() );
    ui->tripleSlopeTimeSpinBox->setValue( m_camera->getAttribute(m_tripleSlopeTimeAttrName).toInt() );
    //ui->thresholdSpinBox->setValue( m_camera->getAttribute(m_thresholdAttrName).toInt() );
    ui->vLow2SpinBox->setValue( m_camera->getAttribute(m_vLow2AttrName).toInt() );
    ui->vLow3SpinBox->setValue( m_camera->getAttribute(m_vLow3AttrName).toInt() );
    ui->vRamp1SpinBox->setValue( m_camera->getAttribute(m_vRamp1AttrName).toInt() );
    ui->vRamp2SpinBox->setValue( m_camera->getAttribute(m_vRamp2AttrName).toInt() );
}

void ConfigurationHelperWindow::on_framePeriodeSpinBox_valueChanged(int arg1)
{
    m_camera->stopAcquisition();
    if ( !m_camera->setAttribute(m_framePeriodeAttrName, arg1) ) {
        //ui->framePeriodeSpinBox->setValue( m_camera->getAttribute(m_framePeriodeAttrName).toInt() );
    }
    m_camera->startAcquisition();
    updateValues();
}

void ConfigurationHelperWindow::on_multiSlopeModeComboBox_currentIndexChanged(const QString &arg1)
{
    m_camera->stopAcquisition();
    if ( !m_camera->setAttribute(m_multipleSlopeModeAttrName, arg1) ) {
        //ui->multiSlopeModeComboBox->setEditText( m_camera->getAttribute(m_multipleSlopeModeAttrName).toString() );
    }
    m_camera->startAcquisition();
    updateValues();
}

void ConfigurationHelperWindow::on_exposureTimeSpinBox_valueChanged(int arg1)
{
    m_camera->stopAcquisition();
    if ( !m_camera->setAttribute(m_exposureTimeAttrName, arg1) ) {
        //ui->exposureTimeSpinBox->setValue( m_camera->getAttribute(m_exposureTimeAttrName).toInt() );
    }
    m_camera->startAcquisition();
    updateValues();
}

void ConfigurationHelperWindow::on_dualSlopeTimeSpinBox_valueChanged(int arg1)
{
    if ( !m_camera->setAttribute(m_dualSlopeTimeAttrName, arg1) ) {
        //ui->dualSlopeTimeSpinBox->setValue( m_camera->getAttribute(m_dualSlopeTimeAttrName).toInt() );
    }
    updateValues();
}

void ConfigurationHelperWindow::on_tripleSlopeTimeSpinBox_valueChanged(int arg1)
{
    if ( !m_camera->setAttribute(m_tripleSlopeTimeAttrName, arg1) ) {
        //ui->tripleSlopeTimeSpinBox->setValue( m_camera->getAttribute(m_tripleSlopeTimeAttrName).toInt() );
    }
    updateValues();
}

void ConfigurationHelperWindow::on_thresholdSpinBox_valueChanged(int arg1)
{
    /// este no hace nada porque no estamos en modo perfil...
    /// pero lo usamos para nuestro calculo!
    updateValues();
}

void ConfigurationHelperWindow::on_vLow2SpinBox_valueChanged(int arg1)
{
    if ( !m_camera->setAttribute(m_vLow2AttrName, arg1) ) {
        //ui->vLow2SpinBox->setValue( m_camera->getAttribute(m_vLow2AttrName).toInt() );
    }
    updateValues();
}

void ConfigurationHelperWindow::on_vLow3SpinBox_valueChanged(int arg1)
{
    if ( !m_camera->setAttribute(m_vLow3AttrName, arg1) ) {
        //ui->vLow3SpinBox->setValue( m_camera->getAttribute(m_vLow3AttrName).toInt() );
    }
    updateValues();
}

void ConfigurationHelperWindow::on_vRamp1SpinBox_valueChanged(int arg1)
{
    if ( !m_camera->setAttribute(m_vRamp1AttrName, arg1) ) {
        //ui->vRamp1SpinBox->setValue( m_camera->getAttribute(m_vRamp1AttrName).toInt() );
    }
    updateValues();
}

void ConfigurationHelperWindow::on_vRamp2SpinBox_valueChanged(int arg1)
{
    if ( !m_camera->setAttribute(m_vRamp2AttrName, arg1) ) {
        //ui->vRamp2SpinBox->setValue( m_camera->getAttribute(m_vRamp2AttrName).toInt() );
    }
    updateValues();
}

void ConfigurationHelperWindow::closeEvent(QCloseEvent * /*event*/)
{
    /// delete on close
    deleteLater();
}

