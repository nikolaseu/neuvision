#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "zcalibratedcamera.h"
#include "zcameracalibratorwidget.h"
#include "zcameraselectorwidget.h"

#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->finishButton->setVisible(false);

    QObject::connect(ui->continueButton, SIGNAL(clicked(bool)),
                     this, SLOT(onContinueButtonClicked()));

    QObject::connect(ui->finishButton, SIGNAL(clicked(bool)),
                     this, SLOT(onFinishButtonClicked()));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onCameraSelected(Z3D::ZCameraInterface::Ptr camera)
{
    m_selectedCamera = camera;

    ui->finishButton->setVisible(m_selectedCamera);
}

void MainWindow::onContinueButtonClicked()
{
    if (ui->calibrateFromFilesRadioButton->isChecked()) {
        /// open the camera calibration window directly, without a camera
        m_selectedCamera = Z3D::ZCameraInterface::Ptr(0);
        onFinishButtonClicked();
    } else if (ui->calibrateOnlineRadioButton->isChecked()) {
        /// open the camera calibration window without a camera
        static Z3D::ZCameraSelectorWidget *cameraSelectorWidget = 0;
        if (!cameraSelectorWidget) {
            cameraSelectorWidget = new Z3D::ZCameraSelectorWidget(ui->pageCameraSelection);
            QObject::connect(cameraSelectorWidget, SIGNAL(cameraSelected(Z3D::ZCameraInterface::Ptr)),
                             this, SLOT(onCameraSelected(Z3D::ZCameraInterface::Ptr)));

            ui->cameraSelectionLayout->addWidget(cameraSelectorWidget);
        }
        ui->stackedWidget->setCurrentWidget(ui->pageCameraSelection);
    } else {
        qWarning() << "something is wrong!";
    }
}

void MainWindow::onFinishButtonClicked()
{
    if (m_selectedCamera) {
        Z3D::ZCalibratedCamera::Ptr calibratedCamera(new Z3D::ZCalibratedCamera(m_selectedCamera, Z3D::ZCameraCalibration::Ptr(0)));
        Z3D::ZCameraCalibratorWidget *calibratorWindow = new Z3D::ZCameraCalibratorWidget(calibratedCamera);
        calibratorWindow->show();
    } else {
        Z3D::ZCameraCalibratorWidget *calibratorWindow = new Z3D::ZCameraCalibratorWidget();
        calibratorWindow->show();
    }

    close();
    //deleteLater(); // we can't delete this, its the "main"!!
}
