#ifndef Z3D_CAMERAACQUISITION_PLUGIN___ZPLEORAEBUSCAMERA_P_H
#define Z3D_CAMERAACQUISITION_PLUGIN___ZPLEORAEBUSCAMERA_P_H

#include "zpleoraebuscamera.h"

#include <QObject>

#include <vector>

#include <PvDevice.h>

/// forward definition
class QThread;

class PvBuffer;
class PvStream;
class PvSystem;

/// typedefs
typedef std::list<PvBuffer *> BufferList;

namespace Z3D
{

class ZPleoraeBUSCameraPrivate : public QObject
{
    Q_OBJECT

    ZPleoraeBUSCamera *q_ptr;
    Q_DECLARE_PUBLIC(ZPleoraeBUSCamera)

public:
    static ZPleoraeBUSCameraPrivate *getCameraByMAC(QString name);

    explicit ZPleoraeBUSCameraPrivate(const PvDeviceInfo *deviceInfo, QObject *parent = 0);
    ~ZPleoraeBUSCameraPrivate();

signals:
    void newImageReceived(Z3D::ZImageGrayscale::Ptr image);
    void warning(QString warningMessage);
    void error(QString errorMessage);

    void attributeChanged(QString name, QVariant value);

public slots:
    bool startAcquisition();
    bool stopAcquisition();

    QList<Z3D::ZCameraInterface::ZCameraAttribute> getAllAttributes(PvGenParameterArray *paramsArray, QString topCategory = "");
    QList<Z3D::ZCameraInterface::ZCameraAttribute> getAllAttributes();
    QVariant getAttribute(const QString &name) const;

    bool setBufferSize(int bufferSize);

protected slots:
    bool setAttribute(const QString &name, const QVariant &value, bool notify);

    void grabLoop();

private:
    static PvSystem *lSystem;

    PvString m_deviceIpAddress;

    const PvDeviceInfo *lDeviceInfo;
    PvDevice* lDevice;

    PvGenParameterArray *lDeviceParams;
/*    PvGenInteger *lTLLocked;
    PvGenInteger *lPayloadSize;
    PvGenCommand *lStart;
    PvGenCommand *lStop;
*/
    PvStream *lStream;

    BufferList lBufferList;

    int m_internalBufferSize;

    int m_lastReturnedBufferNumber;

    int m_acquisitionCounter;
    int m_currentAcquisitionCounter;

    bool m_stopThreadRequested;

    QString m_uuid;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION_PLUGIN___ZPLEORAEBUSCAMERA_P_H
