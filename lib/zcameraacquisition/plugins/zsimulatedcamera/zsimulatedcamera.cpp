#include "zsimulatedcamera.h"

#include <QDebug>
#include <QDir>
#include <QTimer>

namespace Z3D
{

SimulatedCamera::SimulatedCamera(QVariantMap options, QObject *parent)
    : ZCameraBase(parent)
    , m_currentImageNumber(0)
    , m_imageCache(200000000) /// 200mb max
{
    m_uuid = QString("ZSIMULATED-%1").arg(options["Name"].toString());
    m_folder = options["Folder"].toString();

    loadImageFromFilename("gray_00_inv.png");

//    qDebug() << Q_FUNC_INFO << uuid();
}

SimulatedCamera::~SimulatedCamera()
{
//    qDebug() << Q_FUNC_INFO << uuid();
}

bool SimulatedCamera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition())
        return false;

    QTimer::singleShot(100, this, SLOT(emitNewImage()));

    return true;
}

bool SimulatedCamera::stopAcquisition()
{
    return ZCameraBase::stopAcquisition();
}

QList<ZCameraInterface::ZCameraAttribute> SimulatedCamera::getAllAttributes()
{
    QList<ZCameraInterface::ZCameraAttribute> attributes;

    ZCameraInterface::ZCameraAttribute folderAttr;
    folderAttr.name = "Folder";
    folderAttr.value = m_folder;
    folderAttr.type = ZCameraInterface::CameraAttributeTypeString;
    folderAttr.readable = true;
    attributes << folderAttr;

    ZCameraInterface::ZCameraAttribute fileAttr;
    fileAttr.name = "CurrentFile";
    fileAttr.value = m_currentFile;
    fileAttr.type = ZCameraInterface::CameraAttributeTypeString;
    fileAttr.readable = true;
    fileAttr.writable = true;
    attributes << fileAttr;

    return attributes;
}

QVariant SimulatedCamera::getAttribute(const QString &name) const
{
    Q_UNUSED(name)
    return QString("INVALID");
}

void SimulatedCamera::loadImageFromFilename(QString fileName)
{
    m_currentFile = fileName;

    QString file = QDir(m_folder).absoluteFilePath(m_currentFile);

    if (!m_imageCache.contains(file)) {
        qDebug() << "image not found in cache:" << file;
        ZImageGrayscale::Ptr* newImage = new ZImageGrayscale::Ptr(new ZImageGrayscale(file));
        if (!newImage->data()->bufferSize()) {
            qWarning() << "invalid image!" << fileName;
            return;
        }

        //newImage->setNumber(m_currentImageNumber++);

        m_imageCache.insert(file, newImage, newImage->data()->bufferSize());
    } else {
        //qDebug() << "image loaded from cache:" << file;
    }

    ZImageGrayscale::Ptr newImage = *m_imageCache[file];
    newImage->setNumber(m_currentImageNumber++);

    m_lastRetrievedImage->swap(newImage);

    if (isRunning())
        emit newImageReceived(*m_lastRetrievedImage);
}

bool SimulatedCamera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    if (name == "CurrentFile") {
        loadImageFromFilename(value.toString());

        if (notify)
            emit attributeChanged("CurrentFile", m_currentFile);

        return true;
    }

    return false;
}

void SimulatedCamera::emitNewImage()
{
    if (isRunning()) {
        emit newImageReceived(*m_lastRetrievedImage);

        QTimer::singleShot(100, this, SLOT(emitNewImage()));
    }
}

} // namespace Z3D
