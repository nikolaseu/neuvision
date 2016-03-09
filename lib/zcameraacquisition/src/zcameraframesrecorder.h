#pragma once

#include "zcameraacquisition_global.h"

#include "zcameraimage.h"
#include "zcamerainterface.h"

#include <QObject>

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraFramesRecorder : public QObject
{
    Q_OBJECT

public:
    typedef QSharedPointer<Z3D::ZCameraFramesRecorder> Ptr;

    explicit ZCameraFramesRecorder(QObject *parent = 0);

    ZCameraInterface::WeakPtr getCamera() const;
    void setCamera(ZCameraInterface::WeakPtr camera);

signals:

public slots:
    void setEnabled(bool enabled);

    void onAcquisitionStarted();
    void onAcquisitionStopped();

    void onNewImageReceived(Z3D::ZImageGrayscale::Ptr image);

private:
    ZCameraInterface::WeakPtr m_camera;

    bool m_enabled;

    QString m_basePath;
    QString m_currentSavePath;
};

} // namespace Z3D
