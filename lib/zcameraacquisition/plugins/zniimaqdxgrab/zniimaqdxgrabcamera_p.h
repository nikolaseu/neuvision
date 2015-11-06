#ifndef Z3D_CAMERAACQUISITION_PLUGIN___ZNIIMAQDXGRABCAMERA_P_H
#define Z3D_CAMERAACQUISITION_PLUGIN___ZNIIMAQDXGRABCAMERA_P_H

#include "zniimaqdxgrabcamera.h"

#include <QObject>

#include "NIIMAQdx.h"
#include "nivision.h"

#include <vector>

/// forward definition
class QThread;

namespace Z3D
{

class ZNIIMAQdxGrabCameraPrivate : public QObject
{
    Q_OBJECT

    ZNIIMAQdxGrabCamera *q_ptr;
    Q_DECLARE_PUBLIC(ZNIIMAQdxGrabCamera)

public:
    static ZNIIMAQdxGrabCameraPrivate *getCameraByName(QString name);

    explicit ZNIIMAQdxGrabCameraPrivate(SESSION_ID sessionId, QObject *parent = 0);
    ~ZNIIMAQdxGrabCameraPrivate();

    void attributeUpdatedEventCallbackInternal(IMAQdxSession id, const char* name, void* callbackData);

signals:
    void newImageReceived(Z3D::ZImageGrayscale::Ptr image);
    void warning(QString warningMessage);
    void error(QString errorMessage);

    void attributeChanged(QString name, QVariant value);

public slots:
    bool startAcquisition();
    bool stopAcquisition();

    QList<Z3D::ZCameraInterface::ZCameraAttribute> getAllAttributes();
    QVariant getAttribute(const QString &name) const;

    bool setBufferSize(int bufferSize);

protected slots:
    bool setAttribute(const QString &name, const QVariant &value, bool notify);

//    static uInt32 __stdcall frameDoneCallback(IMAQdxSession cbsession, uInt32 bufferNumber, void* callbackData);
//    uInt32 frameDoneCallbackInternal(IMAQdxSession cbSession, uInt32 bufferNumber);

    void grabLoop();

    void processImage();

private:
    void saveCameraAttributes() const;

    Image* m_imaqImage;
    int m_imaqBufferSize;

    uInt32 m_lastReturnedBufferNumber;

    SESSION_ID m_session;

    int m_acquisitionCounter;
    int m_currentAcquisitionCounter;

    bool m_stopThreadRequested;

    QString m_uuid;

    QMap< QString, Z3D::ZCameraInterface::ZCameraAttribute* > m_attributes;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___ZNIIMAQDXGRABCAMERA_P_H
