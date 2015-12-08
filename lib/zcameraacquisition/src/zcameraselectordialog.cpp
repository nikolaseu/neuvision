#include "zcameraselectordialog.h"
#include "ui_zcameraselectordialog.h"

namespace Z3D
{

ZCameraSelectorDialog::ZCameraSelectorDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::ZCameraSelectorDialog)
{
    ui->setupUi(this);

    ui->continueButton->setVisible(false);

    QObject::connect(ui->cameraSelectorWidget, SIGNAL(cameraSelected(Z3D::ZCameraInterface::Ptr)),
                     this, SLOT(onCameraSelected(Z3D::ZCameraInterface::Ptr)));
}

ZCameraSelectorDialog::~ZCameraSelectorDialog()
{
    delete ui;
}

Z3D::ZCameraInterface::Ptr ZCameraSelectorDialog::getCamera() {
    Z3D::ZCameraSelectorDialog cameraSelector;
    cameraSelector.setWindowModality(Qt::WindowModal);
    cameraSelector.show();

    QEventLoop eventLoop;

    QObject::connect(cameraSelector.ui->continueButton, SIGNAL(clicked(bool)),
                     &eventLoop, SLOT(quit()));
    QObject::connect(&cameraSelector, SIGNAL(finished(int)),
                     &eventLoop, SLOT(quit()));

    eventLoop.exec();

    return cameraSelector.m_selectedCamera;
}

void ZCameraSelectorDialog::onCameraSelected(Z3D::ZCameraInterface::Ptr camera)
{
    m_selectedCamera = camera;

    ui->continueButton->setVisible(m_selectedCamera);
}

} // namespace Z3D
