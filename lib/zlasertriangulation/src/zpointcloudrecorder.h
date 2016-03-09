#pragma once

#include "zlasertriangulation_global.h"
#include "zcamera3dinterface.h"

#include <QObject>

namespace Z3D
{

class Z3D_LASERTRIANGULATION_SHARED_EXPORT PointCloudRecorder : public QObject
{
    Q_OBJECT

public:
    typedef QSharedPointer<Z3D::PointCloudRecorder> Ptr;

    explicit PointCloudRecorder(QObject *parent = 0);

    Camera3DInterface::WeakPtr getCamera() const;
    void setCamera(Camera3DInterface::WeakPtr camera);

signals:

public slots:
    void setEnabled(bool enabled);

    void onAcquisitionStarted();
    void onAcquisitionStopped();

    void onNewPointCloudReceived(Z3D::ZPointCloud::Ptr cloud);

private:
    Camera3DInterface::WeakPtr m_camera;

    bool m_enabled;

    QString m_basePath;
    QString m_currentSavePath;
};

} // namespace Z3D
