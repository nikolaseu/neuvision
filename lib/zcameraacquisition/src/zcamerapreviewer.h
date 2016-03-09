#pragma once

#include "zcameraacquisition_global.h"
#include "zcamerainterface.h"

#include <QWidget>


namespace Z3D
{

namespace Ui {
class ZCameraPreviewer;
}

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraPreviewer : public QWidget
{
    Q_OBJECT
    
public:
    explicit ZCameraPreviewer(QWidget *parent = 0);
    explicit ZCameraPreviewer(Z3D::ZCameraInterface::Ptr camera, QWidget *parent = 0);
    ~ZCameraPreviewer();

public slots:
    void setCamera(Z3D::ZCameraInterface::Ptr camera);

protected slots:
    void closeEvent(QCloseEvent *);

    void onCameraRunningChanged();

private slots:
    void on_playButton_clicked();

    void on_snapshotButton_clicked();

private:
    Ui::ZCameraPreviewer *ui;

    Z3D::ZCameraInterface::Ptr m_camera;
};

} // namespace Z3D
