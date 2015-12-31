#include "zcameracalibratorwidget.h"
#include "ui_zcameracalibratorwidget.h"

#include "zcameracalibratorworker.h"

#include "zcalibrationdistortionplot.h"
#include "zcalibrationimagemodel.h"
#include "zcalibrationpatternfinderprovider.h"

#include "3rdParty/objectcontroller.h"

#include <QDateTime>
#include <QFileDialog>
#include <QProgressBar>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QThread>

#define CALIBRATION_PATTERN_VIEW 0
#define IMAGE_VIEW 1
#define CAMERA_VIEW 2
#define CALIBRATION_VIEW 3
#define RESULTS_VIEW 4

namespace Z3D
{

ZCameraCalibratorWidget::ZCameraCalibratorWidget(ZCalibratedCamera::Ptr camera, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZCameraCalibratorWidget)
    , m_model(new ZCalibrationImageModel(this))
    , m_calibratorWorker(new ZCameraCalibratorWorker())
    , m_camera(camera)
    , m_distortionPlot(0)
    , m_currentCalibration(0)
{
    ui->setupUi(this);

    updateWindowTitle();

    /// create and add progress bar used in the statusbar
    m_statusProgressBar = ui->progressBar;
    m_statusProgressBar->setVisible(false);

    /// always start with welcome page visible
    ui->stackedWidget->setCurrentIndex(0);
    ui->topFrameStackedWidget->setCurrentIndex(0);
    setImagesListVisible(false);

    /// connect ui events
    QObject::connect(ui->newSessionButton, SIGNAL(clicked()),
                     this, SLOT(newSession()));
    QObject::connect(ui->loadSessionButton, SIGNAL(clicked()),
                     this, SLOT(loadSession()));

    /// to update current camera calibration model
    QObject::connect(ui->cameraModelTypeComboBox, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(onCameraModelTypeChanged(int)));

    /// available camera models
    m_cameraCalibratorList << ZCameraCalibrationProvider::getCameraCalibrators();

    foreach (ZCameraCalibrator *calibrator, m_cameraCalibratorList)
        ui->cameraModelTypeComboBox->addItem(calibrator->name());

    /// to update current pattern finder
    QObject::connect(ui->calibrationPatternTypeComboBox, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(onCalibrationPatternTypeChanged(int)));

    /// load different calibration pattern finder types
    m_patternFinderList = ZCalibrationPatternFinderProvider::getAll();
    foreach (ZCalibrationPatternFinder::Ptr finder, m_patternFinderList) {
        ui->calibrationPatternTypeComboBox->addItem(finder->name());
    }

    /// configure image list view
    ui->listView->setModel(m_model);
    QObject::connect(m_model, &ZCalibrationImageModel::newImagesAdded,
                     [&](){ this->setImagesListVisible(true); });
    QObject::connect(ui->listView->selectionModel(), SIGNAL(currentChanged(QModelIndex,QModelIndex)),
                     this, SLOT(onCurrentSelectionChanged(QModelIndex,QModelIndex)));
    /// whenever someone clicks the list view, switch to image view
    QObject::connect(ui->listView, SIGNAL(clicked(QModelIndex)),
                     this, SLOT(on_imageViewPageButton_clicked()));

    /// set up image view type
    ui->imageViewTypeComboBox->addItem(tr("View calibration pattern"), ZCalibrationImageViewer::ShowMarkers);
    ui->imageViewTypeComboBox->addItem(tr("View original"), ZCalibrationImageViewer::NoMarkers);

    ///
    m_calibratorWorker->setImageModel(m_model);

    /// calibrator signals
    QObject::connect(m_calibratorWorker, SIGNAL(progressChanged(float,QString)),
                     this, SLOT(onProgressChanged(float,QString)));
    QObject::connect(m_calibratorWorker, SIGNAL(calibrationChanged(Z3D::ZCameraCalibration::Ptr)),
                     this, SLOT(onCalibrationChanged(Z3D::ZCameraCalibration::Ptr)));

    /// move to worker thread
    m_workerThread = new QThread(this);
    m_calibratorWorker->moveToThread(m_workerThread);

    //! TODO esto es necesario?
    foreach (ZCameraCalibrator *cameraCalibrator, m_cameraCalibratorList)
        cameraCalibrator->moveToThread(m_workerThread);
    foreach (ZCalibrationPatternFinder::Ptr patternFinder, m_patternFinderList)
        patternFinder->moveToThread(m_workerThread);

    m_workerThread->start();

    /// add calibration params view/editor
    m_calibrationParamsController = new ObjectController(this);
    ui->calibrationParamsLayout->addWidget(m_calibrationParamsController);

    /// if camera is present
    if (m_camera) {
        /// set up camera image preview
        QObject::connect(m_camera->camera().data(), SIGNAL(newImageReceived(Z3D::ZImageGrayscale::Ptr)),
                         ui->cameraImageViewer, SLOT(updateImage(Z3D::ZImageGrayscale::Ptr)));

        /// show current calibration, but a copy of it
        /// we don't want to change anything from the original, unless the user
        /// says so by clicking "update camera calibration"
        if (m_camera->calibration())
            onCalibrationChanged(m_camera->calibration()->clone());
    } else {
        ui->cameraViewPageButton->setVisible(false);
    }
}

ZCameraCalibratorWidget::~ZCameraCalibratorWidget()
{
    m_workerThread->terminate();

    if (m_camera && ui->stackedWidget->currentIndex() == CAMERA_VIEW)
        m_camera->camera()->stopAcquisition();

    delete ui;
}

void ZCameraCalibratorWidget::closeEvent(QCloseEvent * /*event*/)
{
    deleteLater();
}

void ZCameraCalibratorWidget::updateWindowTitle()
{
    if (m_camera) {
        if (m_sessionFolder.isEmpty()) {
            setWindowTitle(tr("Camera calibration - %1 [empty session]")
                           .arg(m_camera->camera()->uuid()));
        } else {
            setWindowTitle(tr("Camera calibration - %1 [%2]")
                           .arg(m_camera->camera()->uuid())
                           .arg(m_sessionFolder));
        }
    } else {
        if (m_sessionFolder.isEmpty()) {
            setWindowTitle(tr("Camera calibration [empty session]"));
        } else {
            setWindowTitle(tr("Camera calibration [%2]")
                           .arg(m_sessionFolder));
        }
    }
}

void ZCameraCalibratorWidget::setCurrentView(int newView)
{
    if (ui->stackedWidget->currentIndex() == newView)
        return;

    if (m_camera && ui->stackedWidget->currentIndex() == CAMERA_VIEW && newView != CAMERA_VIEW)
        m_camera->camera()->stopAcquisition();

    switch (newView) {
    case CALIBRATION_PATTERN_VIEW:
        setImagesListVisible(true);
        break;
    case IMAGE_VIEW:
        setImagesListVisible(true);
        ui->imageViewPageButton->setChecked(true);
        break;
    case CAMERA_VIEW:
        setImagesListVisible(true);
        if (m_camera)
            m_camera->camera()->startAcquisition();
        break;
    case CALIBRATION_VIEW:
        setImagesListVisible(false);
        break;
    case RESULTS_VIEW:
        setImagesListVisible(false);
        break;
    default:
        break;
    }

    ui->stackedWidget->setCurrentIndex(newView);
    ui->topFrameStackedWidget->setCurrentIndex(newView);
}

void ZCameraCalibratorWidget::setImagesListVisible(bool visible)
{
    /*QList<int> sizes;
    if (visible && m_model->images().size() > 0)
        sizes << 100 << 100;
    else
        sizes << 0 << 100;
    ui->imageListSplitter->setSizes(sizes);*/

    int startValue, endValue;
    if (visible && m_model->images().size() > 0) {
        startValue = ui->listView->maximumWidth();
        endValue = 224;
    } else {
        startValue = ui->listView->maximumWidth();
        endValue = 0;
    }

    if (startValue != endValue) {
        QPropertyAnimation *animation1 = new QPropertyAnimation(ui->listView, "maximumWidth");
        QPropertyAnimation *animation2 = new QPropertyAnimation(ui->listView, "minimumWidth");
        animation1->setStartValue(startValue);
        animation2->setStartValue(startValue);
        animation1->setEndValue(endValue);
        animation2->setEndValue(endValue);
        animation1->setDuration(150);
        animation2->setDuration(150);
        QParallelAnimationGroup *group = new QParallelAnimationGroup();
        group->addAnimation(animation1);
        group->addAnimation(animation2);
        group->start();
    }
}

void ZCameraCalibratorWidget::onCurrentSelectionChanged(QModelIndex current, QModelIndex previous)
{
    Q_UNUSED(previous)

    if (!current.isValid())
        return;

    QVariant variant = current.model()->data(current, ZCalibrationImageModel::DataRole);
    if (!variant.isValid() || !variant.canConvert<Z3D::ZCalibrationImage::Ptr>())
        return;

    Z3D::ZCalibrationImage::Ptr image = variant.value<Z3D::ZCalibrationImage::Ptr>();

    if (image) {
        ui->imageViewer->updateCalibrationImage(image);
    }

    ui->imageViewPageButton->setChecked(true);
}

void ZCameraCalibratorWidget::onCameraModelTypeChanged(int index)
{
    /// remove previous config widget and hide it
    ZCameraCalibrator *currentCameraCalibrator = m_calibratorWorker->cameraCalibrator();
    if (currentCameraCalibrator) {
        QWidget *previousWidget = currentCameraCalibrator->configWidget();
        if (previousWidget) {
            previousWidget->setVisible(false);
            ui->cameraModelConfigLayout->removeWidget(previousWidget);
        }
    }

    /// update current camera calibrator
    currentCameraCalibrator = m_cameraCalibratorList[index];
    m_calibratorWorker->setCameraCalibrator( currentCameraCalibrator );

    /// add config widget
    QWidget *currentWidget = currentCameraCalibrator->configWidget();
    if (currentWidget) {
        currentWidget->setVisible(true);
        ui->cameraModelConfigLayout->addWidget(currentWidget);
    }
}

void ZCameraCalibratorWidget::onCalibrationPatternTypeChanged(int index)
{
    /// remove previous config widget and hide it
    ZCalibrationPatternFinder::Ptr m_currentPatternFinder = m_calibratorWorker->patternFinder();
    if (m_currentPatternFinder) {
        QWidget *previousWidget = m_currentPatternFinder->configWidget();
        previousWidget->setVisible(false);
        ui->calibrationPatternConfigLayout->removeWidget(previousWidget);
    }

    /// update current pattern finder
    m_currentPatternFinder = m_patternFinderList[index];
    m_calibratorWorker->setPatternFinder(m_currentPatternFinder);

    /// add config widget
    QWidget *currentWidget = m_currentPatternFinder->configWidget();
    currentWidget->setVisible(true);
    ui->calibrationPatternConfigLayout->addWidget(currentWidget);
}

void ZCameraCalibratorWidget::onProgressChanged(float progress, QString message)
{
    m_statusProgressBar->setValue(100 * progress);
    m_statusProgressBar->setVisible(progress < 1.);

    /*if (progress == 1 && message.isEmpty())
        ui->statusbar->showMessage(tr(""), 5000);
    else if (!message.isEmpty())
        ui->statusbar->showMessage(message, progress < 1 ? 0 : 10000);
        */
}

void ZCameraCalibratorWidget::onCalibrationChanged(Z3D::ZCameraCalibration::Ptr newCalibration)
{
    if (newCalibration) {
        m_currentCalibration = newCalibration;

        //newCalibration->saveToFile(QString("%1/%2.cameracalib.ini").arg(m_sessionFolder).arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmss")));

        //! show calibration results!
        m_calibrationParamsController->setObject(newCalibration.data());

        /// remove previous plot
        if (m_distortionPlot) {
            ui->distortionPlotLayout->removeWidget(m_distortionPlot);
            m_distortionPlot->deleteLater();
        }

        /// add camera distortion plot
        m_distortionPlot = new ZCalibrationDistortionPlot(newCalibration);
        ui->distortionPlotLayout->addWidget(m_distortionPlot);
    }

    /// enable button again, calibration has finished
    ui->runCalibrationButton->setEnabled(true);
}

void ZCameraCalibratorWidget::loadSession()
{
    QString dirname = QFileDialog::getExistingDirectory(this, "Select image folder", "", QFileDialog::ReadOnly);

    if (dirname.isEmpty()) {
        /// nothing selected
        return;
    }

    m_model->clear();

    m_sessionFolder = dirname;
    m_model->addFolder(dirname);

    updateWindowTitle();
}

void ZCameraCalibratorWidget::newSession()
{
    m_model->clear();

    m_sessionFolder = QString("tmp/cameracalibration/%1_%2")
            .arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmss"))
            .arg(m_camera->camera()->uuid());

    if (!QDir::current().mkpath(m_sessionFolder)) {
        qWarning() << "unable to create folder" << m_sessionFolder;
        m_sessionFolder.clear();
    }

    updateWindowTitle();
}

void ZCameraCalibratorWidget::on_calibrationPatternViewButton_clicked()
{
    setCurrentView(CALIBRATION_PATTERN_VIEW);
}

void ZCameraCalibratorWidget::on_imageViewPageButton_clicked()
{
    setCurrentView(IMAGE_VIEW);
}

void ZCameraCalibratorWidget::on_cameraViewPageButton_clicked()
{
    setCurrentView(CAMERA_VIEW);
}

void ZCameraCalibratorWidget::on_calibrationPageButton_clicked()
{
    setCurrentView(CALIBRATION_VIEW);
}

void ZCameraCalibratorWidget::on_resultsPageButton_clicked()
{
    setCurrentView(RESULTS_VIEW);
}

void ZCameraCalibratorWidget::on_runCalibrationButton_clicked()
{
    /// disable button until calibration has finished
    ui->runCalibrationButton->setEnabled(false);

    m_calibratorWorker->calibrate();
}

void ZCameraCalibratorWidget::on_imageViewTypeComboBox_currentIndexChanged(int index)
{
    ZCalibrationImageViewer::DisplayMode displayMode = (ZCalibrationImageViewer::DisplayMode) ui->imageViewTypeComboBox->itemData(index).toInt();
    ui->imageViewer->setDisplayMode(displayMode);
}

void ZCameraCalibratorWidget::on_saveCameraImageButton_clicked()
{
    ZImageGrayscale::Ptr image = m_camera->camera()->getSnapshot();

    if (m_sessionFolder.isEmpty()) {
        newSession();
    }

    QString fileName = QString("%1/%2.png")
            .arg(m_sessionFolder)
            .arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmsszzz"));

    if (image->save(fileName)) {
        qDebug() << "saved image" << fileName;

        /// add to calibration images model
        ZCalibrationImage::Ptr calibrationImage(new ZCalibrationImage(fileName));
        if (ui->saveOnlyValidImagesCheckBox->isChecked()) {
            /// only add if image is valid and the pattern is found
            if (calibrationImage->isValid() && calibrationImage->findPattern(m_calibratorWorker->patternFinder().data()))
                m_model->addImage(calibrationImage);
            else
                QDir::current().remove(fileName);
        } else {
            /// add all images
            m_model->addImage(calibrationImage);
            m_calibratorWorker->findCalibrationPattern();
        }
    } else {
        qWarning() << "unable to save image to" << fileName;
    }
}

void ZCameraCalibratorWidget::on_cameraSettingsButton_clicked()
{
    if (m_camera)
        m_camera->camera()->showSettingsDialog();
}

void ZCameraCalibratorWidget::on_saveCalibrationToFileButton_clicked()
{
    if (!m_currentCalibration) {
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(
                this,
                "Save calibration as...",
                QString("%1/%2.cameracalib.ini")
                    .arg(!m_sessionFolder.isEmpty() ? m_sessionFolder : QDir::homePath())
                    .arg(m_camera ? m_camera->camera()->uuid() : QDateTime::currentDateTime().toString("yyyyMMddhhmmss")),
                QString(tr("Camera calibration files (*.cameracalib.ini)"))
                );

    if (fileName.isEmpty()) {
        /// nothing selected
        return;
    }

    m_currentCalibration->saveToFile(fileName);
}

void ZCameraCalibratorWidget::on_updateCameraCalibrationButton_clicked()
{
    if (!m_camera) {
        return;
    }

    if (!m_currentCalibration) {
        return;
    }

    m_camera->setCalibration(m_currentCalibration->clone());
}

} // namespace Z3D
