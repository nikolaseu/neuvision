#ifndef Z3D_CAMERAACQUISITION___ZCAMERABASE_H
#define Z3D_CAMERAACQUISITION___ZCAMERABASE_H

#define CAMERA_MESSAGE(str) QString("[%1] %2").arg(uuid()).arg(str)
#define CAMERA_DEBUG(str)   qDebug()    << CAMERA_MESSAGE(str); emit message( CAMERA_MESSAGE(str) );
#define CAMERA_WARNING(str) qWarning()  << CAMERA_MESSAGE(str); emit warning( CAMERA_MESSAGE(str) );
#define CAMERA_ERROR(str)   qWarning()  << CAMERA_MESSAGE(str); emit error(   CAMERA_MESSAGE(str) );

#include "zcameraacquisition_global.h"
#include "zcamerainterface.h"

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraBase : public ZCameraInterface
{
    Q_OBJECT

public:
    explicit ZCameraBase(QObject *parent = 0);

    virtual QString uuid() override { return m_uuid; }

    virtual int bufferSize() override;

public slots:
    ///
    virtual bool requestSnapshot() override;
    virtual ZImageGrayscale::Ptr getSnapshot() override;

    /// acquisition control
    virtual bool startAcquisition() override;
    virtual bool stopAcquisition() override;
    virtual bool isRunning() override { return m_simultaneousCapturesCount > 0; }

    /// camera attributes (settings)
    virtual QList<ZCameraAttribute> getAllAttributes() override = 0;
    virtual bool setAttribute(const QString &name, const QVariant &value) override;
    virtual QVariant getAttribute(const QString &name) const override = 0;

    virtual void showSettingsDialog() override;

    /// camera configuration
    virtual bool loadConfiguration(QString configFileName) override;

    /// presets
    virtual QStringList getPresets() override;
    virtual bool loadPresetConfiguration(QString presetName) override;

    virtual bool setBufferSize(int bufferSize) override;

protected slots:
    virtual bool setAttribute(const QString &name, const QVariant &value, bool notify) = 0;

protected:
    QString m_uuid;

    QMap< QString, QList<ZCameraAttribute> > m_cameraPresets;

    ZImageGrayscale::Ptr *m_lastRetrievedImage;

    ZImageGrayscale::Ptr getNextBufferImage(int width, int height, int xOffset, int yOffset, int bytesPerPixel, void *externalBuffer = 0);

private:
    std::vector<ZImageGrayscale::Ptr> m_imagesBuffer;
    int m_currentImageBufferIndex;

    int m_simultaneousCapturesCount;

    /// camera settings
    QWidget *m_settingsWidget;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION___ZCAMERABASE_H
