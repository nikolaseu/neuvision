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

#include "ZCameraCalibrator/zcameracalibratorwidget.h"
#include "ui_zcameracalibratorwidget.h"

#include "ZCameraCalibrator/zcalibrationimagemodel.h"
#include "ZCameraCalibrator/zcalibrationpatternfinder.h"
#include "ZCameraCalibrator/zcalibrationpatternfinderprovider.h"
#include "ZCameraCalibrator/zcameracalibratorworker.h"

#include "ZCalibratedCamera/zcalibratedcamera.h"
#include "ZCameraAcquisition/zcameraimage.h"
#include "ZCameraAcquisition/zcamerainterface.h"
#include "ZCameraCalibration/zcameracalibration.h"
#include "ZCameraCalibration/zcameracalibrationprovider.h"
#include "ZCameraCalibration/zcameracalibrator.h"

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

ZCameraCalibratorWidget::ZCameraCalibratorWidget(ZCalibratedCameraPtr camera, QWidget *parent)
    : ZWidget(parent)
    , ui(new Ui::ZCameraCalibratorWidget)
    , m_model(new ZCalibrationImageModel(this))
    , m_calibratorWorker(new ZCameraCalibratorWorker())
    , m_camera(camera)
    , m_currentCalibration(nullptr)
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
    QObject::connect(ui->newSessionButton, &QPushButton::clicked,
                     this, &ZCameraCalibratorWidget::newSession);
    QObject::connect(ui->loadSessionButton, &QPushButton::clicked,
                     this, &ZCameraCalibratorWidget::loadSession);

    /// to update current camera calibration model
    QObject::connect(ui->cameraModelTypeComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
                     this, &ZCameraCalibratorWidget::onCameraModelTypeChanged);

    /// available camera models
    m_cameraCalibratorList << ZCameraCalibrationProvider::getCameraCalibrators();

    for (ZCameraCalibrator *calibrator : m_cameraCalibratorList) {
        ui->cameraModelTypeComboBox->addItem(calibrator->name());
    }

    /// to update current pattern finder
    QObject::connect(ui->calibrationPatternTypeComboBox, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
                     this, &ZCameraCalibratorWidget::onCalibrationPatternTypeChanged);

    /// load different calibration pattern finder types
    m_patternFinderList = ZCalibrationPatternFinderProvider::getAll();
    for (const auto &finder : m_patternFinderList) {
        ui->calibrationPatternTypeComboBox->addItem(finder->name());
    }

    /// configure image list view
    ui->listView->setModel(m_model);
    QObject::connect(m_model, &ZCalibrationImageModel::newImagesAdded,
                     [&](){ this->setImagesListVisible(true); });
    QObject::connect(ui->listView->selectionModel(), &QItemSelectionModel::currentChanged,
                     this, &ZCameraCalibratorWidget::onCurrentSelectionChanged);
    /// whenever someone clicks the list view, switch to image view
    QObject::connect(ui->listView, &QListView::clicked,
                     this, &ZCameraCalibratorWidget::on_imageViewPageButton_clicked);

    /// set up image view type
    ui->imageViewTypeComboBox->addItem(tr("View calibration pattern"), ZCalibrationImageViewer::ShowMarkers);
    ui->imageViewTypeComboBox->addItem(tr("View pattern coords"), ZCalibrationImageViewer::ShowMarkersAndCoords);
    ui->imageViewTypeComboBox->addItem(tr("View original"), ZCalibrationImageViewer::NoMarkers);

    ///
    m_calibratorWorker->setImageModel(m_model);

    /// calibrator signals
    QObject::connect(m_calibratorWorker, &ZCameraCalibratorWorker::progressChanged,
                     this, &ZCameraCalibratorWidget::onProgressChanged);
    QObject::connect(m_calibratorWorker, &ZCameraCalibratorWorker::calibrationChanged,
                     this, &ZCameraCalibratorWidget::onCalibrationChanged);

    /// move to worker thread
    m_workerThread = new QThread(this);
    m_calibratorWorker->moveToThread(m_workerThread);

    //! TODO esto es necesario?
    for (auto cameraCalibrator : m_cameraCalibratorList)
        cameraCalibrator->moveToThread(m_workerThread);
    for (auto patternFinder : m_patternFinderList)
        patternFinder->moveToThread(m_workerThread);

    m_workerThread->start();

    /// add calibration params view/editor
    m_calibrationParamsController = new ObjectController(this);
    ui->calibrationParamsLayout->addWidget(m_calibrationParamsController);

    /// if camera is present
    if (m_camera) {
        /// set up camera image preview
        QObject::connect(m_camera->camera().get(), &ZCameraInterface::newImageReceived,
                         ui->cameraImageViewer, static_cast<void (ZImageViewer::*)(ZCameraImagePtr)>(&ZImageViewer::updateImage));

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
//            setWindowTitle(tr("Camera calibration - %1 [empty session]")
//                           .arg(m_camera->camera()->uuid()));
        } else {
//            setWindowTitle(tr("Camera calibration - %1 [%2]")
//                           .arg(m_camera->camera()->uuid())
//                           .arg(m_sessionFolder));
        }
    } else {
        if (m_sessionFolder.isEmpty()) {
//            setWindowTitle(tr("Camera calibration [empty session]"));
        } else {
//            setWindowTitle(tr("Camera calibration [%2]")
//                           .arg(m_sessionFolder));
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
    if (!variant.isValid() || !variant.canConvert<Z3D::ZCalibrationImagePtr>())
        return;

    auto image = variant.value<Z3D::ZCalibrationImagePtr>();

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
        ui->cameraModelConfigLayout->addWidget(currentWidget);
        currentWidget->setVisible(true);
    }
}

void ZCameraCalibratorWidget::onCalibrationPatternTypeChanged(int index)
{
    /// remove previous config widget and hide it
    auto m_currentPatternFinder = m_calibratorWorker->patternFinder();
    if (m_currentPatternFinder) {
        QWidget *previousWidget = ZCalibrationPatternFinderProvider::getConfigWidget(m_currentPatternFinder.get());
        previousWidget->setVisible(false);
        ui->calibrationPatternConfigLayout->removeWidget(previousWidget);
    }

    /// update current pattern finder
    m_currentPatternFinder = m_patternFinderList[index];
    m_calibratorWorker->setPatternFinder(m_currentPatternFinder);

    /// add config widget
    QWidget *currentWidget = ZCalibrationPatternFinderProvider::getConfigWidget(m_currentPatternFinder.get());
    ui->calibrationPatternConfigLayout->addWidget(currentWidget);
    currentWidget->setVisible(true);
}

void ZCameraCalibratorWidget::onProgressChanged(float progress, QString /*message*/)
{
    m_statusProgressBar->setValue(100 * progress);
    m_statusProgressBar->setVisible(progress < 1.f);

    /*if (progress == 1 && message.isEmpty())
        ui->statusbar->showMessage(tr(""), 5000);
    else if (!message.isEmpty())
        ui->statusbar->showMessage(message, progress < 1 ? 0 : 10000);
        */
}

void ZCameraCalibratorWidget::onCalibrationChanged(ZCameraCalibrationPtr newCalibration)
{
    if (newCalibration) {
        m_currentCalibration = newCalibration;

        //newCalibration->saveToFile(QString("%1/%2.cameracalib.ini").arg(m_sessionFolder).arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmss")));

        //! show calibration results!
        m_calibrationParamsController->setObject(newCalibration.get());
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
    ZCameraImagePtr image = m_camera->camera()->getSnapshot();

    if (m_sessionFolder.isEmpty()) {
        newSession();
    }

    QString fileName = QString("%1/%2.png")
            .arg(m_sessionFolder)
            .arg(QDateTime::currentDateTime().toString("yyyyMMddhhmmsszzz"));

    if (ZCameraImage::save(image, fileName)) {
        qDebug() << "saved image" << fileName;

        /// add to calibration images model
        ZCalibrationImagePtr calibrationImage(new ZCalibrationImage(fileName));
        if (ui->saveOnlyValidImagesCheckBox->isChecked()) {
            /// only add if image is valid and the pattern is found
            if (calibrationImage->isValid() && calibrationImage->findPattern(m_calibratorWorker->patternFinder().get()))
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
