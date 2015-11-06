#ifndef Z3D_CAMERAACQUISITION_PLUGIN___FLYCAPTURE2CAMERA_H
#define Z3D_CAMERAACQUISITION_PLUGIN___FLYCAPTURE2CAMERA_H

#include "zcamerainterface_p.h"

#include "FlyCapture2.h"
#include "FlyCapture2Gui.h"

namespace Z3D
{

class ZFlyCapture2Camera : public ZCameraBase
{
    Q_OBJECT

public:
    static QList<ZCameraInterface::Ptr> getConnectedCameras();
    static ZCameraInterface::Ptr getCameraByName(QString name);

    ~ZFlyCapture2Camera();
    
signals:
    
public slots:
    void showSettingsDialog();

protected:
    explicit ZFlyCapture2Camera(FlyCapture2::Camera *cameraHandle);

private:
    static void imageEventCallback(FlyCapture2::Image* pImage, const void *pCallbackData );
    void imageEventCallbackInternal(FlyCapture2::Image* pImage);

    FlyCapture2::Camera* m_pCamera;
    FlyCapture2::CameraInfo m_cameraInfo;
    FlyCapture2::Image m_rawImage;
    FlyCapture2::Image m_processedImage;

    FlyCapture2::CameraControlDlg m_camCtlDlg;

    long m_currentBufferNumber;


    // ZCameraBase interface
public slots:
    virtual QList<ZCameraAttribute> getAllAttributes();
    virtual QVariant getAttribute(const QString &name) const;

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify);

    // ZCameraInterface interface
public:
    virtual bool startAcquisition();
    virtual bool stopAcquisition();
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___FLYCAPTURE2CAMERA_H
