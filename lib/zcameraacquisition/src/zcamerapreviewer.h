#ifndef Z3D_CAMERAACQUISITION___ZCAMERAPREVIEWER_H
#define Z3D_CAMERAACQUISITION___ZCAMERAPREVIEWER_H

#include "zcameraacquisition_global.h"
#include "zcamerainterface.h"

#include <QWidget>

namespace Ui {
class CameraPreviewer;
}

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraPreviewer : public QWidget
{
    Q_OBJECT
    
public:
    explicit ZCameraPreviewer(QWidget *parent = 0);
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
    Ui::CameraPreviewer *ui;

    Z3D::ZCameraInterface::Ptr m_camera;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION___ZCAMERAPREVIEWER_H
