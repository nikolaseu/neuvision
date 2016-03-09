#ifndef Z3D_CAMERAACQUISITION_PLUGIN___ZSIMULATEDCAMERA_H
#define Z3D_CAMERAACQUISITION_PLUGIN___ZSIMULATEDCAMERA_H

#include "zcamerainterface_p.h"

#include <QCache>
#include <QDir>

namespace Z3D
{

class ZSimulatedCamera : public ZCameraBase
{
    Q_OBJECT

public:
    explicit ZSimulatedCamera(QVariantMap options, QObject *parent = 0);
    ~ZSimulatedCamera();

signals:

public slots:
    virtual bool startAcquisition();
    virtual bool stopAcquisition();

    virtual QList<ZCameraAttribute> getAllAttributes();
    virtual QVariant getAttribute(const QString &name) const;

    /// this is unique, only for simulated camera
    void loadImageFromFilename(QString fileName);

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify);

    void emitNewImage();

private:
    QString m_folder;
    QString m_currentFile;

    QDir m_dir;

    int m_currentImageNumber;

    QCache<QString, ZImageGrayscale::Ptr> m_imageCache;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___ZSIMULATEDCAMERA_H
