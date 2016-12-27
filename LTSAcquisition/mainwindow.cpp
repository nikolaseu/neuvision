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

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "configurationhelperwindow.h"

//! FIXME make this work with Qt5
#if QT_VERSION < QT_VERSION_CHECK(5,0,0)
#  include "pointcloudwindow.h"
#  include "zlasertriangulationcalibratorwindow.h"
#endif

#include <Z3DCameraAcquisition>
#include <Z3DCameraCalibrationProvider>
#include <Z3DLaserTriangulation>

#include "zcameracalibratorwidget.h"
#include "zcameraselectordialog.h"

#include <QDebug>
#include <QDir>
#include <QMessageBox>
#include <QTime>
#include <QtConcurrentRun>


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_3dview(0),
    m_selectedCamera3D(0)
{
    ui->setupUi(this);

    /// load camera plugins from application folder
    Z3D::ZCameraProvider::loadPlugins();

    //! load camera calibration plugins from application folder
    Z3D::ZCameraCalibrationProvider::loadPlugins();

    ///
    ui->acqModeComboBox->addItem("Image mode",   Z3D::LTSCamera3D::ImageAcquisitionMode);
    ui->acqModeComboBox->addItem("Profile mode", Z3D::LTSCamera3D::ProfileAcquisitionMode);
    ui->acqModeComboBox->addItem("Dual mode",    Z3D::LTSCamera3D::ImageAcquisitionMode | Z3D::LTSCamera3D::DualAcquisitionMode);

    /// configure camera list view
    ui->cameraListView->setModel( &m_cameraListModel );
    QObject::connect(ui->cameraListView->selectionModel(), SIGNAL(currentChanged(QModelIndex,QModelIndex)),
                     this, SLOT(onCurrentSelectionChanged(QModelIndex,QModelIndex)));

    /// enable/disable (des)connection
    QObject::connect(this, SIGNAL(connectedChanged(bool)),
                     ui->connectedCamerasGroupBox, SLOT(setEnabled(bool)));
    QObject::connect(this, SIGNAL(connectedChanged(bool)),
                     ui->cameraAttributesGroupBox, SLOT(setEnabled(bool)));
    QObject::connect(this, SIGNAL(connectedChanged(bool)),
                     ui->acqControlGroupBox, SLOT(setEnabled(bool)));
    QObject::connect(this, SIGNAL(connectedChanged(bool)),
                     ui->acqOptionsGroupBox, SLOT(setEnabled(bool)));

    emit connectedChanged(false);
}

MainWindow::~MainWindow()
{
    /// delete cameras
    m_camera3DList.clear();

    Z3D::ZCameraProvider::unloadPlugins();

    delete ui;
}

void MainWindow::on_connectButton_clicked()
{
    bool connectedSuccessfully = false;

    if (m_camera3DList.size()) {
        /// disconnection request
        /// delete camera (shared pointers handles this)
        if (m_selectedCamera3D)
            m_selectedCamera3D = ZLTSThreadedCamera::Ptr(0);

        m_camera3DList.clear();

        /// clear list view (set empty list model)
        m_cameraListModel.setStringList( QStringList() );
    } else {
        /// connection requested
        QStringList cameraStrList;

        foreach(Z3D::LTSCamera3D::Ptr camera3D, Z3D::LTSProvider::loadCameras()) {
            cameraStrList << camera3D->uuid();

            ZLTSThreadedCamera::Ptr threadedCamera(new ZLTSThreadedCamera(camera3D));

            m_camera3DList << threadedCamera;

            /// connect camera messages to console
            QObject::connect(camera3D->camera2d()->camera().data(), SIGNAL(message(QString)),
                             ui->consoleTextEdit, SLOT(appendPlainText(QString)));
            QObject::connect(camera3D->camera2d()->camera().data(), SIGNAL(warning(QString)),
                             ui->consoleTextEdit, SLOT(appendPlainText(QString)));
            QObject::connect(camera3D->camera2d()->camera().data(), SIGNAL(error(QString)),
                             ui->consoleTextEdit, SLOT(appendPlainText(QString)));
        }

        m_cameraListModel.setStringList(cameraStrList);

        if (m_camera3DList.size()) {
            /// set current selected mode
            int currentAcqModeIndex = ui->acqModeComboBox->currentIndex();
            ui->acqModeComboBox->setCurrentIndex(-1); // to be sure it changes
            ui->acqModeComboBox->setCurrentIndex(currentAcqModeIndex);

            connectedSuccessfully = true;
        } else {
            qWarning() << "can't find camera!";
        }
    }

    if (connectedSuccessfully) {
        ui->connectButton->setText("Disconnect");
    } else {
        ui->connectButton->setText("Connect");
    }

    emit connectedChanged(connectedSuccessfully);
}

void MainWindow::on_acqModeComboBox_currentIndexChanged(int index)
{
    if (index >= 0) {
        foreach(ZLTSThreadedCamera::Ptr threadedCamera, m_camera3DList) {
            Z3D::LTSCamera3D::Ptr camera3D = threadedCamera->camera3d();
            camera3D->setAcquisitionMode( ui->acqModeComboBox->itemData(index).toInt() );
        }
    }
}

void MainWindow::on_open3DViewButton_clicked()
{
    if (!m_selectedCamera3D)
        return;

#if QT_VERSION < QT_VERSION_CHECK(5,0,0)
    if (!m_3dview)
        m_3dview = new PointCloudWindow();

    m_3dview->setCamera(m_selectedCamera3D->camera3d().data());

    m_3dview->show();
#endif
}

void MainWindow::on_openImageViewButton_clicked()
{
    if (!m_selectedCamera3D)
        return;

    /// use correct preview based on acquisition mode
    if (m_selectedCamera3D->camera3d()->acquisitionMode() & Z3D::LTSCamera3D::ImageAcquisitionMode) {
        Z3D::ZCameraPreviewer *cameraPreviewer = new Z3D::ZCameraPreviewer();
        cameraPreviewer->setCamera(m_selectedCamera3D->camera2d()->camera());
        cameraPreviewer->show();
    } else {
        Z3D::LTSCameraPreviewer *cameraPreviewer = new Z3D::LTSCameraPreviewer(m_selectedCamera3D->camera3d());
        cameraPreviewer->show();
    }
}


void MainWindow::onCurrentSelectionChanged(QModelIndex current, QModelIndex previous)
{
    Q_UNUSED(previous)

    if (!current.isValid())
        return;

    /// update selected camera
    m_selectedCamera3D = m_camera3DList[current.row()];
}

void MainWindow::on_startAcqButton_clicked()
{
    foreach(ZLTSThreadedCamera::Ptr threadedCamera, m_camera3DList) {
        threadedCamera->startAcquisition();
    }
}

void MainWindow::on_stopAcqButton_clicked()
{
    foreach(ZLTSThreadedCamera::Ptr threadedCamera, m_camera3DList) {
        threadedCamera->stopAcquisition();
    }
}

void MainWindow::on_frameRecorderEnabledCheckBox_toggled(bool checked)
{
    foreach(ZLTSThreadedCamera::Ptr threadedCamera, m_camera3DList) {
        threadedCamera->setFrameRecorderEnabled(checked);
    }
}

void MainWindow::on_cloudRecorderEnabledCheckBox_toggled(bool checked)
{
    foreach(ZLTSThreadedCamera::Ptr threadedCamera, m_camera3DList) {
        threadedCamera->setCloudRecorderEnabled(checked);
    }
}

void MainWindow::on_calibrateLTSButton_clicked()
{
    if (!m_selectedCamera3D)
        return;

    //CalibrationWindow *calibWindow = new CalibrationWindow(m_selectedCamera3D->camera3d(), this);
#if QT_VERSION < QT_VERSION_CHECK(5,0,0)
    Z3D::ZLaserTriangulationCalibratorWindow *calibWindow = new Z3D::ZLaserTriangulationCalibratorWindow(m_selectedCamera3D->camera2d().data());
    calibWindow->show();
#endif
}

void MainWindow::on_cameraSettingsHelperButton_clicked()
{
    if (!m_selectedCamera3D)
        return;

    ConfigurationHelperWindow *configHelperWindow = new ConfigurationHelperWindow(m_selectedCamera3D->camera3d().data());
    configHelperWindow->show();
}

void MainWindow::on_openCameraSettingsButton_clicked()
{
    if (!m_selectedCamera3D)
        return;

    m_selectedCamera3D->camera2d()->camera()->showSettingsDialog();
}

void MainWindow::on_calibrateCameraButton_clicked()
{
    if (!m_selectedCamera3D)
        return;

    Z3D::ZCameraCalibratorWidget *calibWindow = new Z3D::ZCameraCalibratorWidget(m_selectedCamera3D->camera3d()->camera2d());
    calibWindow->show();
}

void MainWindow::on_addCameraButton_clicked()
{
    Z3D::ZCameraInterface::Ptr camera = Z3D::ZCameraSelectorDialog::getCamera();

    if (!camera)
        return;

    Z3D::ZCalibratedCamera::Ptr calibratedCamera(new Z3D::ZCalibratedCamera(camera, Z3D::ZCameraCalibration::Ptr(0)));
    Z3D::LTSCamera3D::Ptr camera3D(new Z3D::LTSCamera3D(calibratedCamera));

    ZLTSThreadedCamera::Ptr threadedCamera(new ZLTSThreadedCamera(camera3D));

    m_camera3DList << threadedCamera;

    /// connect camera messages to console
    QObject::connect(camera3D->camera2d()->camera().data(), SIGNAL(message(QString)),
                     ui->consoleTextEdit, SLOT(appendPlainText(QString)));
    QObject::connect(camera3D->camera2d()->camera().data(), SIGNAL(warning(QString)),
                     ui->consoleTextEdit, SLOT(appendPlainText(QString)));
    QObject::connect(camera3D->camera2d()->camera().data(), SIGNAL(error(QString)),
                     ui->consoleTextEdit, SLOT(appendPlainText(QString)));

    QStringList cameraStrList;
    foreach(ZLTSThreadedCamera::Ptr threadedCamera, m_camera3DList) {
            cameraStrList << threadedCamera->camera3d()->uuid();
    }

    m_cameraListModel.setStringList(cameraStrList);

    emit connectedChanged(true);
}
