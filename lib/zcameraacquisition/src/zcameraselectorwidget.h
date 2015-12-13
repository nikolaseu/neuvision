#ifndef Z3D_CAMERAACQUISITION___ZCAMERASELECTORWIDGET_H
#define Z3D_CAMERAACQUISITION___ZCAMERASELECTORWIDGET_H

#include "zcameraacquisition_global.h"
#include "zcamerainterface.h"

#include <QWidget>

namespace Z3D
{

namespace Ui
{
class ZCameraSelectorWidget;
}

class ZCameraInfo;
class ZCameraPluginInterface;


class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraSelectorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ZCameraSelectorWidget(QWidget *parent = 0);
    ~ZCameraSelectorWidget();

signals:
    void cameraSelected(Z3D::ZCameraInterface::Ptr camera);

private slots:
    void onPluginIndexChanged(int index);

    void onCameraIndexChanged(int index);

private:
    Ui::ZCameraSelectorWidget *ui;

    QList<Z3D::ZCameraPluginInterface *> m_pluginList;

    QList<Z3D::ZCameraInfo *> m_currentCameraList;

    Z3D::ZCameraInterface::Ptr m_selectedCamera;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION___ZCAMERASELECTORWIDGET_H
