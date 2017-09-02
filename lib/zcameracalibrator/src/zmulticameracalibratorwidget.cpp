//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "zmulticameracalibratorwidget.h"
#include "ui_zmulticameracalibratorwidget.h"



#include "zcalibrationdistortionplot.h"
#include "zcalibrationimageviewer.h"
#include "zcalibrationpatternfinderprovider.h"
#include "zmulticalibrationimagemodel.h"
#include "zmulticameracalibratorworker.h"

#include "zimageviewer.h"

#include <QDateTime>
#include <QFileDialog>
#include <QProgressBar>
#include <QThread>

#define CALIBRATION_PATTERN_VIEW 0
#define IMAGE_VIEW 1
#define CAMERA_VIEW 2
#define CALIBRATION_VIEW 3

namespace Z3D
{

ZMultiCameraCalibratorWidget::ZMultiCameraCalibratorWidget(std::vector<ZCalibratedCamera::Ptr> cameras, QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZMultiCameraCalibratorWidget)
    , m_model(new ZMultiCalibrationImageModel(this))
    , m_calibratorWorker(new ZMultiCameraCalibratorWorker())
    , m_cameras(cameras)
{
    ui->setupUi(this);

    updateWindowTitle();

    for (auto camera : m_cameras) {
        /// camera views
        ZImageViewer *cameraImageViewer = new ZImageViewer(ui->cameraViewsLayout->widget());
        ui->cameraViewsLayout->addWidget(cameraImageViewer, 1);
        m_cameraImageViewer.push_back(cameraImageViewer);

        /// set up camera image preview
        if (camera) {
            QObject::connect(camera->camera().data(), SIGNAL(newImageReceived(Z3D::ZImageGrayscale::Ptr)),
                             cameraImageViewer, SLOT(updateImage(Z3D::ZImageGrayscale::Ptr)));
        }

        /// image views
        ZCalibrationImageViewer *imageViewer = new ZCalibrationImageViewer(ui->imageViewsLayout->widget());
        ui->imageViewsLayout->addWidget(imageViewer, 1);
        m_imageViewer.push_back(imageViewer);
    }

    /// create and add progress bar used in the statusbar
    m_statusProgressBar = new QProgressBar();
    m_statusProgressBar->setVisible(false);
    m_statusProgressBar->setMaximumWidth(300);
    ui->statusbar->addPermanentWidget(m_statusProgressBar);

    /// always start with welcome page visible
    ui->stackedWidget->setCurrentIndex(0);

    /// connect ui events
    QObject::connect(ui->newSessionButton, SIGNAL(clicked()),
                     this, SLOT(newSession()));
    QObject::connect(ui->loadSessionButton, SIGNAL(clicked()),
                     this, SLOT(loadSession()));

    /// to update current camera calibration model
    QObject::connect(ui->cameraModelTypeComboBox, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(onCameraModelTypeChanged(int)));

    /// available camera models
    m_cameraCalibratorList = ZCameraCalibrationProvider::getMultiCameraCalibrators();

    foreach (ZMultiCameraCalibrator *calibrator, m_cameraCalibratorList)
        ui->cameraModelTypeComboBox->addItem(calibrator->name());

    /// to update current pattern finder
    QObject::connect(ui->calibrationPatternTypeComboBox, SIGNAL(currentIndexChanged(int)),
                     this, SLOT(onCalibrationPatternTypeChanged(int)));

    /// load different calibration pattern finder types
    m_patternFinderList = ZCalibrationPatternFinderProvider::getAll();

    foreach (ZCalibrationPatternFinder::Ptr patternFinder, m_patternFinderList)
        ui->calibrationPatternTypeComboBox->addItem(patternFinder->name());

    /// configure image list view
    ui->listView->setModel(m_model);
    QObject::connect(ui->listView->selectionModel(), SIGNAL(currentChanged(QModelIndex,QModelIndex)),
                     this, SLOT(onCurrentSelectionChanged(QModelIndex,QModelIndex)));
    /// whenever someone clicks the list view, switch to image view
    QObject::connect(ui->listView, SIGNAL(clicked(QModelIndex)),
                     this, SLOT(on_imageViewPageButton_clicked()));

    /// set up image view type
    ui->imageViewTypeComboBox->addItem(tr("View calibration pattern"), ZCalibrationImageViewer::ShowMarkers);
    ui->imageViewTypeComboBox->addItem(tr("View pattern coords"), ZCalibrationImageViewer::ShowMarkersAndCoords);
    ui->imageViewTypeComboBox->addItem(tr("View original"), ZCalibrationImageViewer::NoMarkers);

    m_calibratorWorker->setImageModel(m_model);

    /// calibrator signals
    QObject::connect(m_calibratorWorker, SIGNAL(progressChanged(float,QString)),
                     this, SLOT(onProgressChanged(float,QString)));
    QObject::connect(m_calibratorWorker, SIGNAL(calibrationChanged(std::vector<Z3D::ZCameraCalibration::Ptr>)),
                     this, SLOT(onCalibrationChanged(std::vector<Z3D::ZCameraCalibration::Ptr>)));

    /// move to worker thread
    m_workerThread = new QThread(this);
    m_calibratorWorker->moveToThread(m_workerThread);

    //! TODO esto es necesario?
    foreach (ZMultiCameraCalibrator *cameraCalibrator, m_cameraCalibratorList)
        cameraCalibrator->moveToThread(m_workerThread);
    foreach (ZCalibrationPatternFinder::Ptr patternFinder, m_patternFinderList)
        patternFinder->moveToThread(m_workerThread);

    m_workerThread->start();
}

ZMultiCameraCalibratorWidget::~ZMultiCameraCalibratorWidget()
{
    m_workerThread->terminate();

    for (auto camera : m_cameras) {
        if (camera && ui->stackedWidget->currentIndex() == CAMERA_VIEW)
            camera->camera()->stopAcquisition();
    }

    delete ui;
}

void ZMultiCameraCalibratorWidget::closeEvent(QCloseEvent * /*event*/)
{
    deleteLater();
}

void ZMultiCameraCalibratorWidget::updateWindowTitle()
{
    if (m_sessionFolder.isEmpty()) {
        setWindowTitle(tr("Multi camera calibration - [empty session]"));
    } else {
        setWindowTitle(tr("Multi camera calibration - [%1]")
                       .arg(m_sessionFolder));
    }
}

void ZMultiCameraCalibratorWidget::setCurrentView(int newView)
{
    if (ui->stackedWidget->currentIndex() == newView)
        return;

    for (auto camera : m_cameras) {
        if (camera && ui->stackedWidget->currentIndex() == CAMERA_VIEW && newView != CAMERA_VIEW)
            camera->camera()->stopAcquisition();
    }

    switch (newView) {
    case CALIBRATION_PATTERN_VIEW:
        ui->stackedWidget->setCurrentIndex(newView);
        break;
    case IMAGE_VIEW:
        ui->imageViewPageButton->setChecked(true);
        ui->stackedWidget->setCurrentIndex(newView);
        break;
    case CAMERA_VIEW:
        ui->stackedWidget->setCurrentIndex(newView);
        for (auto camera : m_cameras) {
            if (camera)
                camera->camera()->startAcquisition();
        }
        break;
    case CALIBRATION_VIEW:
        ui->stackedWidget->setCurrentIndex(newView);
        break;
    default:
        break;
    }
}

void ZMultiCameraCalibratorWidget::onCurrentSelectionChanged(QModelIndex current, QModelIndex previous)
{
    Q_UNUSED(previous)

    if (!current.isValid())
        return;

    QVariant variant = current.model()->data(current, ZMultiCalibrationImageModel::DataRole);
    if (!variant.isValid() || !variant.canConvert<Z3D::ZMultiCalibrationImage::Ptr>())
        return;

    Z3D::ZMultiCalibrationImage::Ptr multiImage = variant.value<Z3D::ZMultiCalibrationImage::Ptr>();

    if (multiImage) {
        for (int i=0; i<m_cameras.size(); ++i) {
            Z3D::ZCalibrationImage::Ptr image = multiImage->image(i);
            if (image)
                m_imageViewer[i]->updateCalibrationImage(image);
        }
    }

    ui->imageViewPageButton->setChecked(true);
}

void ZMultiCameraCalibratorWidget::onCameraModelTypeChanged(int index)
{
    /// remove previous config widget and hide it
    ZMultiCameraCalibrator *currentCameraCalibrator = m_calibratorWorker->cameraCalibrator();
    if (currentCameraCalibrator) {
        QWidget *previousWidget = ZCameraCalibrationProvider::getConfigWidget(currentCameraCalibrator);
        if (previousWidget) {
            previousWidget->setVisible(false);
            ui->cameraModelConfigLayout->removeWidget(previousWidget);
        }
    }

    /// update current camera calibrator
    currentCameraCalibrator = m_cameraCalibratorList[index];
    m_calibratorWorker->setCameraCalibrator( currentCameraCalibrator );

    /// add config widget
    QWidget *currentWidget = ZCameraCalibrationProvider::getConfigWidget(currentCameraCalibrator);
    if (currentWidget) {
        currentWidget->setVisible(true);
        ui->cameraModelConfigLayout->addWidget(currentWidget);
    }
}

void ZMultiCameraCalibratorWidget::onCalibrationPatternTypeChanged(int index)
{
    /// remove previous config widget and hide it
    ZCalibrationPatternFinder::Ptr m_currentPatternFinder = m_calibratorWorker->patternFinder();
    if (m_currentPatternFinder) {
        QWidget *previousWidget = ZCalibrationPatternFinderProvider::getConfigWidget(m_currentPatternFinder);
        previousWidget->setVisible(false);
        ui->calibrationPatternConfigLayout->removeWidget(previousWidget);
    }

    /// update current pattern finder
    m_currentPatternFinder = m_patternFinderList[index];
    m_calibratorWorker->setPatternFinder(m_currentPatternFinder);

    /// add config widget
    QWidget *currentWidget = ZCalibrationPatternFinderProvider::getConfigWidget(m_currentPatternFinder);
    currentWidget->setVisible(true);
    ui->calibrationPatternConfigLayout->addWidget(currentWidget);
}

void ZMultiCameraCalibratorWidget::onProgressChanged(float progress, QString message)
{
    m_statusProgressBar->setValue(100 * progress);
    m_statusProgressBar->setVisible(progress < 1.);

    if (progress == 1 && message.isEmpty())
        ui->statusbar->showMessage(tr(""), 5000);
    else if (!message.isEmpty())
        ui->statusbar->showMessage(message, progress < 1 ? 0 : 5000);
}

void ZMultiCameraCalibratorWidget::onCalibrationChanged(std::vector<Z3D::ZCameraCalibration::Ptr> newCalibrations)
{
    if (newCalibrations.size() == m_cameras.size()) {
        QString currentDateTime = QDateTime::currentDateTime().toString("yyyyMMddhhmmss");
        for (int i=0; i<m_cameras.size(); ++i) {
            Z3D::ZCameraCalibration::Ptr newCalibration = newCalibrations[i];
            QString fileName = QString("%1/%2__%3.cameracalib.ini")
                    .arg(m_sessionFolder)
                    .arg(currentDateTime)
                    .arg(m_cameras[i]->camera()->uuid());
            if (!newCalibration->saveToFile(fileName))
                qWarning() << "unable to save calibration to file:" << fileName;

            //! TODO show calibration results!
        }
    } else {
        qWarning() << "calibration went wrong, cameras.size != newCalibrations.size";
    }

    /// enable button again, calibration has finished
    ui->runCalibrationButton->setEnabled(true);
}

void ZMultiCameraCalibratorWidget::loadSession()
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

void ZMultiCameraCalibratorWidget::newSession()
{
    m_model->clear();

    m_sessionFolder = QString("tmp/multicameracalibration/%1")
            .arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmss"));

    /// common session folder
    if (!QDir::current().mkpath(m_sessionFolder)) {
        qWarning() << "unable to create folder" << m_sessionFolder;
        m_sessionFolder.clear();
    }

    /// folder for each camera inside session folder
    //foreach(Z3D::CalibratedCamera::Ptr camera, m_cameras) {
    for (int i=0; i<m_cameras.size(); ++i) {
        Z3D::ZCalibratedCamera::Ptr camera = m_cameras[i];

        QString cameraFolder = QString("%1/%2__%3")
                .arg(m_sessionFolder)
                .arg(i, 2, 10, QLatin1Char('0'))
                .arg(camera->camera()->uuid());

        if (!QDir::current().mkpath(cameraFolder)) {
            qWarning() << "unable to create folder" << cameraFolder;
            m_sessionFolder.clear();
            break;
        }
    }

    updateWindowTitle();
}

void ZMultiCameraCalibratorWidget::on_imageViewPageButton_clicked()
{
    setCurrentView(IMAGE_VIEW);
}

void ZMultiCameraCalibratorWidget::on_cameraViewPageButton_clicked()
{
    setCurrentView(CAMERA_VIEW);
}

void ZMultiCameraCalibratorWidget::on_calibrationPageButton_clicked()
{
    setCurrentView(CALIBRATION_VIEW);
}

void ZMultiCameraCalibratorWidget::on_runCalibrationButton_clicked()
{
    /// disable button until calibration has finished
    ui->runCalibrationButton->setEnabled(false);

    std::vector<Z3D::ZCameraCalibration::Ptr> currentCalibrations;
    for (auto camera : m_cameras) {
        currentCalibrations.push_back(camera->calibration());
    }

    m_calibratorWorker->calibrate(currentCalibrations);
}

void ZMultiCameraCalibratorWidget::on_imageViewTypeComboBox_currentIndexChanged(int index)
{
    ZCalibrationImageViewer::DisplayMode displayMode = (ZCalibrationImageViewer::DisplayMode) ui->imageViewTypeComboBox->itemData(index).toInt();
    foreach (ZCalibrationImageViewer *imageViewer, m_imageViewer)
        imageViewer->setDisplayMode(displayMode);
}

void ZMultiCameraCalibratorWidget::on_calibrationPatternViewButton_clicked()
{
    setCurrentView(CALIBRATION_PATTERN_VIEW);
}

void ZMultiCameraCalibratorWidget::on_saveCameraImageButton_clicked()
{
    QString currentDateTime = QDateTime::currentDateTime().toString("yyyyMMddhhmmsszzz");

    QList<ZImageGrayscale::Ptr> imagesList;
    QList<ZCalibrationImage::Ptr> calibrationImagesList;

    for (auto camera : m_cameras) {
        imagesList << camera->camera()->getSnapshot();
    }

    bool isValid = true;

    for (int i=0; i<m_cameras.size(); ++i) {
        Z3D::ZCalibratedCamera::Ptr camera = m_cameras[i];
        ZImageGrayscale::Ptr image = imagesList[i];

        if (m_sessionFolder.isEmpty()) {
            newSession();
        }

        QString fileName = QString("%1/%2__%3/%4.png")
                .arg(m_sessionFolder)
                .arg(i, 2, 10, QLatin1Char('0'))
                .arg(camera->camera()->uuid())
                .arg(currentDateTime);

        if (image->save(fileName)) {
            qDebug() << "saved image" << fileName;

            /// add to calibration images model
            ZCalibrationImage::Ptr calibrationImage(new ZCalibrationImage(fileName));
            if (ui->saveOnlyValidImagesCheckBox->isChecked()) {
                /// only add if image is valid and the pattern is found

                //! FIXME implementar busqueda de patron en paralelo!
                if (isValid && calibrationImage->isValid() && calibrationImage->findPattern(m_calibratorWorker->patternFinder().data())) {
                    calibrationImagesList << calibrationImage;
                } else {
                    isValid = false;
                    QDir::current().remove(fileName);
                }
            } else {
                /// add all images
                calibrationImagesList << calibrationImage;

                m_calibratorWorker->findCalibrationPattern();
            }
        } else {
            isValid = false;
            qWarning() << "unable to save image to" << fileName;
        }
    }

    if (isValid) {
        ZMultiCalibrationImage::Ptr images(new ZMultiCalibrationImage(calibrationImagesList));
        m_model->addImage(images);
    }
}

void ZMultiCameraCalibratorWidget::on_cameraSettingsButton_clicked()
{
    for (auto camera : m_cameras) {
        if (camera)
            camera->camera()->showSettingsDialog();
    }
}

} // namespace Z3D
