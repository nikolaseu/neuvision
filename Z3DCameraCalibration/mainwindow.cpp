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
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::onCameraSelected(Z3D::ZCameraInterface::Ptr camera)
{
    if (camera) {
        Z3D::ZCalibratedCamera::Ptr calibratedCamera(new Z3D::ZCalibratedCamera(camera, Z3D::ZCameraCalibration::Ptr(0)));
        Z3D::ZCameraCalibratorWidget *calibratorWindow = new Z3D::ZCameraCalibratorWidget(calibratedCamera);
        calibratorWindow->show();
    } else {
        Z3D::ZCameraCalibratorWidget *calibratorWindow = new Z3D::ZCameraCalibratorWidget();
        calibratorWindow->show();
    }

    close();
    //deleteLater(); // we can't delete this, its the "main"!!
}

void MainWindow::on_continueButton_clicked()
{
    if (ui->calibrateFromFilesRadioButton->isChecked()) {
        /// open the camera calibration window directly, without a camera
        onCameraSelected(Z3D::ZCameraInterface::Ptr(0));
    } else if (ui->calibrateOnlineRadioButton->isChecked()) {
        /// open the camera calibration window without a camera
        static Z3D::ZCameraSelectorWidget *cameraSelectorWidget = 0;
        if (!cameraSelectorWidget) {
            cameraSelectorWidget = new Z3D::ZCameraSelectorWidget(ui->pageCameraSelection);
            QObject::connect(cameraSelectorWidget, SIGNAL(cameraSelected(Z3D::ZCameraInterface::Ptr)),
                             this, SLOT(onCameraSelected(Z3D::ZCameraInterface::Ptr)));

            QVBoxLayout *layout = new QVBoxLayout(ui->pageCameraSelection);
            layout->setMargin(0);
            layout->addWidget(cameraSelectorWidget);

            ui->pageCameraSelection->setLayout(layout);
        }
        ui->stackedWidget->setCurrentWidget(ui->pageCameraSelection);
    } else {
        qWarning() << "something is wrong!";
    }
}
