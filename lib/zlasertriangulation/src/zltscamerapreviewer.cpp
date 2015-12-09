#include "zltscamerapreviewer.h"
#include "ui_zltscamerapreviewer.h"

#include "zltsprofileplot.h"
#include <qmath.h>

#include <QMessageBox>

namespace Z3D
{

LTSCameraPreviewer::LTSCameraPreviewer(Z3D::LTSCamera3D::Ptr camera, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::LTSCameraPreviewer),
    m_camera(camera)
{
    ui->setupUi(this);

    m_plot = new LTSProfilePlot();
    ui->plotLayout->addWidget(m_plot);

    ui->scanViewDisplayModeComboBox->addItem("Normal", ScanImageViewer::NormalDisplay);
    ui->scanViewDisplayModeComboBox->addItem("Line subtraction", ScanImageViewer::LineSubtractionDisplay);
    ui->scanViewDisplayModeComboBox->addItem("Line subtraction (RANSAC)", ScanImageViewer::LineSubtractionRansacDisplay);
    ui->scanViewDisplayModeComboBox->addItem("Unsharp masking", ScanImageViewer::UnsharpMaskingDisplay);

    /// connect to scan image view range changes
    QObject::connect(ui->scanImageView, SIGNAL(minimumChanged(double)),
                     this, SLOT(onRangeMinimumChanged(double)));
    QObject::connect(ui->scanImageView, SIGNAL(maximumChanged(double)),
                     this, SLOT(onRangeMaximumChanged(double)));
    QObject::connect(ui->scanImageView, SIGNAL(visibleImageCountChanged(int)),
                     ui->visibleImagesSpinBox, SLOT(setValue(int)));
    QObject::connect(ui->visibleImagesSpinBox, SIGNAL(valueChanged(int)),
                     ui->scanImageView, SLOT(setVisibleImageCount(int)));

    ui->scaleValuesCheckBox->setChecked(ui->scanImageView->scaleValues());
    QObject::connect(ui->scaleValuesCheckBox, SIGNAL(toggled(bool)),
                     ui->scanImageView, SLOT(setScaleValues(bool)));

    ///
//    QObject::connect(ui->rangeMinSpinBox, SIGNAL(valueChanged(double)),
//                     ui->scanImageView, SLOT(setMinimum(double)));
//    QObject::connect(ui->rangeMaxSpinBox, SIGNAL(valueChanged(double)),
//                     ui->scanImageView, SLOT(setMaximum(double)));

    /// connect to camera signals
    QObject::connect(m_camera->camera2d()->camera().data(), SIGNAL(newImageReceived(Z3D::ZImageGrayscale::Ptr)),
                     this, SLOT(onNewImageReceived(Z3D::ZImageGrayscale::Ptr)));
    QObject::connect(m_camera.data(), SIGNAL(newPointCloudReceived(Z3D::ZPointCloud::Ptr)),
                     this, SLOT(onNewPointCloudReceived(Z3D::ZPointCloud::Ptr)));
}

LTSCameraPreviewer::~LTSCameraPreviewer()
{
    delete ui;
}

void LTSCameraPreviewer::closeEvent(QCloseEvent *)
{
    deleteLater();
}

void LTSCameraPreviewer::onNewImageReceived(ZImageGrayscale::Ptr image)
{
    /// update scan "top view"
    ui->scanImageView->updateImage(image);

    float m_divisor = m_camera->profileDivisor();

    int howManyRois = std::min(image->height(), 2);

    /// update curve plot of the last profile
    m_samples.clear();
    m_samples.reserve(image->width() * howManyRois);
    quint16 *data = (quint16*) image->buffer();

    bool useCalibration = ui->actionDisplayCalibratedValues->isChecked() && m_camera->camera2d()->calibration();

    int x;
    float y;
    for (int i=0; i<image->width() * howManyRois; ++i) {
        if (*data) {
            x = i % image->width();
            y = float(*data)/m_divisor;
            if (useCalibration) {
                float calX, calY;
                if (m_camera->camera2d()->calibration()->getCalibratedSubPixel(x, y, &calX, &calY))
                    m_samples.push_back(QPointF(calX, calY) );
            } else {
                m_samples.push_back(QPointF(x, y) );
            }
        }
        data++;
    }

    m_plot->setSamples(m_samples);
    m_plot->replot();
}

void LTSCameraPreviewer::onNewPointCloudReceived(ZPointCloud::Ptr cloud)
{
    Q_UNUSED(cloud)
}

void LTSCameraPreviewer::onRangeMinimumChanged(double min)
{
    ui->rangeMinSpinBox->setValue(min / m_camera->profileDivisor());
}

void LTSCameraPreviewer::onRangeMaximumChanged(double max)
{
    ui->rangeMaxSpinBox->setValue(max / m_camera->profileDivisor());
}

void LTSCameraPreviewer::on_rangeMinSpinBox_valueChanged(double arg1)
{
    ui->scanImageView->setMinimum(arg1 * m_camera->profileDivisor());
}

void LTSCameraPreviewer::on_rangeMaxSpinBox_valueChanged(double arg1)
{
    ui->scanImageView->setMaximum(arg1 * m_camera->profileDivisor());
}

void LTSCameraPreviewer::on_actionPlayPause_triggered()
{
    if (m_camera->camera2d()->camera()->isRunning())
        m_camera->stopAcquisition();
    else
        m_camera->startAcquisition();
}

void LTSCameraPreviewer::on_actionAutoSearchROI_triggered()
{
    if (!m_camera->camera2d()->camera()->setAttribute("Device::CameraControls::Commands::SearchAoi", true))
        QMessageBox::warning(0,
                             "Command execution failed",
                             "The camera couldn't execute the command. Is the camera currently running an acquisition?",
                             QMessageBox::Ok);
}

void LTSCameraPreviewer::on_actionUnsharpMasking_triggered()
{
    ///
}

void LTSCameraPreviewer::on_actionShowCameraSettings_triggered()
{
    m_camera->camera2d()->camera()->showSettingsDialog();
}

void LTSCameraPreviewer::on_autoSetMinMaxButton_clicked()
{
    ui->scanImageView->setAutoScale();
}

void LTSCameraPreviewer::on_scanViewDisplayModeComboBox_currentIndexChanged(int index)
{
    bool ok;
    int displayModeInt = ui->scanViewDisplayModeComboBox->itemData(index).toInt(&ok);
    if (ok) {
        ui->scanImageView->setDisplayMode( (ScanImageViewer::DisplayMode) displayModeInt );
    } else {
        qWarning() << "error while getting ScanImageViewer::DisplayMode from ComboBox item data (UserRole)";
    }
}


} // namespace Z3D
