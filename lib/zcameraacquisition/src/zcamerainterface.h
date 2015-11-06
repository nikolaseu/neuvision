#ifndef Z3D_CAMERAACQUISITION___ZCAMERAINTERFACE_H
#define Z3D_CAMERAACQUISITION___ZCAMERAINTERFACE_H

#include "zcameraacquisition_global.h"
#include "zcameraimage.h"

#include <QDebug>
#include <QObject>
#include <QPointer>
#include <QSharedPointer>
#include <QStringList>
#include <QVariant>

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraInterface : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QString uuid READ uuid CONSTANT)
    Q_PROPERTY(bool running READ isRunning NOTIFY runningChanged)
    Q_PROPERTY(QStringList presets READ getPresets CONSTANT)

public:
    typedef QSharedPointer<Z3D::ZCameraInterface> Ptr;
    typedef QPointer<Z3D::ZCameraInterface> WeakPtr;

    enum CameraAttributeType {
        CameraAttributeTypeUnknown = 0,
        CameraAttributeTypeBool,
        CameraAttributeTypeString,
        CameraAttributeTypeInt,
        CameraAttributeTypeLong,
        CameraAttributeTypeFloat,
        CameraAttributeTypeEnum,
        CameraAttributeTypeCommand
    };

    struct ZCameraAttribute {
        ZCameraAttribute() :
            name("UNKNOWN"),
            value("UNKNOWN"),
            readable(false),
            writable(false),
            type(CameraAttributeTypeUnknown),
            maximumValue(DBL_MAX),
            minimumValue(DBL_MIN),
            enumValue(-1)
        {

        }

        QString name;
        QString description;
        QVariant value;
        bool readable;
        bool writable;
        CameraAttributeType type;
        double maximumValue;
        double minimumValue;
        int enumValue;
        QStringList enumNames;
    };

    explicit ZCameraInterface(QObject *parent = 0) : QObject(parent) {}
    virtual ~ZCameraInterface() {}

    virtual QString uuid() = 0;

    virtual int bufferSize() = 0;

signals:
    void newImageReceived(Z3D::ZImageGrayscale::Ptr image);

    void message(QString message);
    void warning(QString warningMessage);
    void error(QString errorMessage);

    void acquisitionStarted();
    void acquisitionStopped();

    void runningChanged();

    void attributeChanged(QString name, QVariant value);

public slots:
    virtual ZImageGrayscale::Ptr getSnapshot() = 0;

    /// acquisition control
    Q_INVOKABLE virtual bool startAcquisition() = 0;
    Q_INVOKABLE virtual bool stopAcquisition() = 0;
    virtual bool isRunning() = 0;

    /// camera attributes (settings)
    virtual QList<ZCameraAttribute> getAllAttributes() = 0;
    virtual bool setAttribute(const QString &name, const QVariant &value) = 0;
    virtual QVariant getAttribute(const QString &name) const = 0;

    virtual void showSettingsDialog() = 0;

    /// camera configuration
    virtual bool loadConfiguration(QString configFileName) = 0;

    /// presets
    virtual QStringList getPresets() = 0;
    Q_INVOKABLE virtual bool loadPresetConfiguration(QString presetName) = 0;

    virtual bool setBufferSize(int bufferSize) = 0;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION___ZCAMERAINTERFACE_H
