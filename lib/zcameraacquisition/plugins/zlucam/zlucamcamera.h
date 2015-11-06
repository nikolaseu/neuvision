#ifndef Z3D_CAMERAACQUISITION_PLUGIN___LUCAMCAMERA_H
#define Z3D_CAMERAACQUISITION_PLUGIN___LUCAMCAMERA_H

#include "zcamerainterface_p.h"

#include <windows.h> // lucamapi.h needs it
#include "lucamapi.h"

#include <vector>

namespace Z3D
{

class LuCamCamera : public ZCameraBase
{
    Q_OBJECT

public:
    static QList<ZCameraInterface::Ptr> getConnectedCameras();
    static ZCameraInterface::Ptr getCameraByName(QString name);

    explicit LuCamCamera(HANDLE cameraHandle, QObject *parent = 0);
    ~LuCamCamera();

public slots:
    virtual bool startAcquisition();
    virtual bool stopAcquisition();

    virtual QList<ZCameraAttribute> getAllAttributes();
    virtual QVariant getAttribute(const QString &name) const;

    virtual void showSettingsDialog();

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify);

private:
    static void __stdcall imageEventCallback(void *pCallbackData, BYTE *data, ULONG dataLenght);
    void imageEventCallbackInternal(BYTE *data, ULONG dataLenght);

    LONG m_snapshotCallbackId;
    LONG m_streamingCallbackId;

    HANDLE m_cameraHandle;

    LUCAM_FRAME_FORMAT m_frameFormat;

    LUCAM_SNAPSHOT m_snapshotSettings;

    LUCAM_VERSION m_lucamVersion;

    std::vector<ZImageGrayscale::Ptr> m_imagesBuffer;
    int m_currentImageBufferIndex;

    int m_acquisitionCounter;
    int m_currentAcquisitionCounter;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___LUCAMCAMERA_H
