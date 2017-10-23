//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "zsimulatedcamera.h"

#include <QDebug>
#include <QTimer>

namespace Z3D
{

ZSimulatedCamera::ZSimulatedCamera(QVariantMap options, QObject *parent)
    : ZCameraBase(parent)
    , m_currentImageNumber(0)
    , m_imageCache(200000000) /// 200mb max
{
    m_uuid = QString("ZSIMULATED-%1").arg(options["Name"].toString());
    m_folder = options["Folder"].toString();

    m_dir = QDir::current();
    if (!m_folder.isEmpty() && m_dir.exists(m_folder)) {
        m_dir.cd(m_folder);
    } else {
#if defined(Q_OS_MAC)
        if (m_dir.dirName() == "MacOS") {
            m_dir.cdUp();
            m_dir.cdUp();
            m_dir.cdUp();
        }
        if (!m_folder.isEmpty() && m_dir.exists(m_folder)) {
            m_dir.cd(m_folder);
        } else
#endif
            qWarning() << "Folder" << m_folder << "not found in" << m_dir.absolutePath();
    }

    const auto entries = m_dir.entryList(QStringList() << "*.png", QDir::Files);
    if (entries.size()) {
        loadImageFromFilename(entries.first());
    }
}

ZSimulatedCamera::~ZSimulatedCamera()
{

}

bool ZSimulatedCamera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition()) {
        return false;
    }

    QTimer::singleShot(100, this, SLOT(emitNewImage()));

    return true;
}

bool ZSimulatedCamera::stopAcquisition()
{
    return ZCameraBase::stopAcquisition();
}

QList<ZCameraInterface::ZCameraAttribute> ZSimulatedCamera::getAllAttributes()
{
    QList<ZCameraInterface::ZCameraAttribute> attributes;

    ZCameraInterface::ZCameraAttribute folderAttr;
    folderAttr.id = "Folder";
    folderAttr.path = "Folder";
    folderAttr.label = "Folder";
    folderAttr.value = m_folder;
    folderAttr.type = ZCameraInterface::CameraAttributeTypeString;
    folderAttr.readable = true;
    folderAttr.writable = true;
    attributes << folderAttr;

    ZCameraInterface::ZCameraAttribute fileAttr;
    fileAttr.id = "CurrentFile";
    fileAttr.path = "CurrentFile";
    fileAttr.label = "CurrentFile";
    fileAttr.value = m_currentFile;
    fileAttr.type = ZCameraInterface::CameraAttributeTypeString;
    fileAttr.readable = true;
    fileAttr.writable = true;
    attributes << fileAttr;

    return attributes;
}

QVariant ZSimulatedCamera::getAttribute(const QString &name) const
{
    Q_UNUSED(name)
    return QString("INVALID");
}

void ZSimulatedCamera::loadImageFromFilename(QString fileName)
{
    m_currentFile = fileName;

    QString file = m_dir.absoluteFilePath(m_currentFile);

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

    if (isRunning()) {
        emit newImageReceived(*m_lastRetrievedImage);
    }
}

bool ZSimulatedCamera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    if (name == "CurrentFile") {
        loadImageFromFilename(value.toString());

        if (notify) {
            emit attributeChanged("CurrentFile", m_currentFile);
        }

        return true;
    }

    return false;
}

void ZSimulatedCamera::emitNewImage()
{
    if (isRunning()) {
        emit newImageReceived(*m_lastRetrievedImage);

        QTimer::singleShot(100, this, SLOT(emitNewImage()));
    }
}

} // namespace Z3D
