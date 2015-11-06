#ifndef Z3D_CAMERAACQUISITION_PLUGIN___AVTVIMBACAMERA_H
#define Z3D_CAMERAACQUISITION_PLUGIN___AVTVIMBACAMERA_H

#include "zcamerainterface_p.h"

#include <queue>

#include <VimbaCPP/Include/VimbaCPP.h>
#include "VimbaCPP/Include/IFrameObserver.h"

#include <QMutex>

namespace Z3D
{

////////////////////////////////////////////////////////////////////////////////
/// \brief The FrameObserver class
///
class FrameObserver : public QObject, virtual public AVT::VmbAPI::IFrameObserver
{
    Q_OBJECT

public:
    // We pass the camera that will deliver the frames to the constructor
    FrameObserver( AVT::VmbAPI::CameraPtr pCamera );
    ~FrameObserver();

    // This is our callback routine that will be executed on every received frame
    virtual void FrameReceived( const AVT::VmbAPI::FramePtr pFrame );

    // After the view has been notified about a new frame it can pick it up
    AVT::VmbAPI::FramePtr GetFrame();

    // Clears the double buffer frame queue
    void ClearFrameQueue();

private:
    // Since a Qt signal cannot contain a whole frame
    // the frame observer stores all FramePtr
    std::queue<AVT::VmbAPI::FramePtr> m_Frames;
    QMutex m_FramesMutex;

signals:
    // The frame received event that passes the frame directly
    void FrameReceivedSignal( int status );
};




////////////////////////////////////////////////////////////////////////////////
/// \brief The AVTVimbaCamera class
///
class AVTVimbaCamera : public ZCameraBase
{
    Q_OBJECT

public:
    explicit AVTVimbaCamera(AVT::VmbAPI::CameraPtr cameraPtr, QObject *parent = 0);
    ~AVTVimbaCamera();

    QString deviceID() const;

signals:

public slots:
    /// acquisition control
    virtual bool startAcquisition();
    virtual bool stopAcquisition();

    /// camera attributes (settings)
    virtual QList<ZCameraAttribute> getAllAttributes();
    virtual QVariant getAttribute(const QString &deviceID) const;

    virtual bool setBufferSize(int bufferSize);

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify);

private slots:
    void onFrameReady(int status);

    bool open();
    bool close();

private:
    AVT::VmbAPI::CameraPtr m_cameraHandle;

    QString m_deviceID;

    int m_vimbaBufferSize;

    // The current pixel format
    //VmbInt64_t m_nPixelFormat;
    // The maximum width
    //VmbInt64_t m_widthMax;
    // The maximum height
    //VmbInt64_t m_heightMax;

    int m_currentWidth;
    int m_currentHeight;

    int m_currentXOffset;
    int m_currentYOffset;

    QPointer<FrameObserver> m_frameObserverInternal;

    QMutex m_mutex;

    bool m_opened;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___AVTVIMBACAMERA_H
