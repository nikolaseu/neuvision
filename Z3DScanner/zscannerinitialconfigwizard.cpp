#include "zscannerinitialconfigwizard.h"
#include "ui_zscannerinitialconfigwizard.h"

#include "zcameraselectorwidget.h"

#include <QDebug>

ZScannerInitialConfigWizard::ZScannerInitialConfigWizard(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ZScannerInitialConfigWizard)
{
    ui->setupUi(this);
}

ZScannerInitialConfigWizard::~ZScannerInitialConfigWizard()
{
    delete ui;
}

void ZScannerInitialConfigWizard::on_pushButton_clicked()
{
    int currentPage =  ui->stackedWidget->currentIndex();
    switch (currentPage) {
    case 0: {
        static Z3D::ZCameraSelectorWidget *m_leftCameraSelectorWidget = 0;
        if (!m_leftCameraSelectorWidget) {
            m_leftCameraSelectorWidget = new Z3D::ZCameraSelectorWidget(ui->pageLeftCameraSelection);
            //QObject::connect(m_leftCameraSelectorWidget, SIGNAL(cameraSelected(Z3D::ZCameraInterface::Ptr)),
            //                 this, SLOT(onCameraSelected(Z3D::ZCameraInterface::Ptr)));

            QVBoxLayout *layout = new QVBoxLayout(ui->pageLeftCameraSelection);
            layout->setMargin(0);
            layout->addWidget(m_leftCameraSelectorWidget);

            ui->pageLeftCameraSelection->setLayout(layout);
        }

        ui->titleLabel->setText(tr("Left camera selection"));
        ui->stackedWidget->setCurrentIndex(currentPage+1);
        break;
    }
    case 1:{
        static Z3D::ZCameraSelectorWidget *m_rightCameraSelectorWidget = 0;
        if (!m_rightCameraSelectorWidget) {
            m_rightCameraSelectorWidget = new Z3D::ZCameraSelectorWidget(ui->pageRightCameraSelection);
            //QObject::connect(m_rightCameraSelectorWidget, SIGNAL(cameraSelected(Z3D::ZCameraInterface::Ptr)),
            //                 this, SLOT(onCameraSelected(Z3D::ZCameraInterface::Ptr)));

            QVBoxLayout *layout = new QVBoxLayout(ui->pageRightCameraSelection);
            layout->setMargin(0);
            layout->addWidget(m_rightCameraSelectorWidget);

            ui->pageRightCameraSelection->setLayout(layout);
        }

        ui->titleLabel->setText(tr("Right camera selection"));
        ui->stackedWidget->setCurrentIndex(currentPage+1);
        break;
    }
    case 2:{
        ui->titleLabel->setText(tr("Intrinsic camera calibration"));
        ui->stackedWidget->setCurrentIndex(currentPage+1);
        break;
    }
    case 3:{
        ui->titleLabel->setText(tr("Extrinsic camera calibration"));
        ui->stackedWidget->setCurrentIndex(currentPage+1);
        break;
    }
    case 4:
        ui->titleLabel->setText(tr("ZScanner configuration"));
        ui->stackedWidget->setCurrentIndex(0);
        break;
    default:
        qWarning() << "some index missing?" << currentPage;
    }
}
