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

#include "zcamerainterface_p.h"

#include "zcamerasettingswidget.h"

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QSettings>
#include <QStringList>

namespace Z3D
{

ZCameraBase::ZCameraBase(QObject *parent)
    : ZCameraInterface(parent)
    , m_lastRetrievedImage(nullptr)
    , m_currentImageBufferIndex(-1)
    , m_simultaneousCapturesCount(0)
    , m_settingsWidget(nullptr)
{
    QObject::connect(this, SIGNAL(acquisitionStarted()),
                     this, SIGNAL(runningChanged()));
    QObject::connect(this, SIGNAL(acquisitionStopped()),
                     this, SIGNAL(runningChanged()));

    /// 50 images in buffer
    m_imagesBuffer.resize(50);

    m_lastRetrievedImage = &m_imagesBuffer[0];
}

int ZCameraBase::bufferSize()
{
    return m_imagesBuffer.size();
}

bool ZCameraBase::requestSnapshot()
{
    /// TODO implement this
    return true;
}

ZImageGrayscale::Ptr ZCameraBase::getSnapshot()
{
    if (isRunning()) {
        /// make a copy because after acquisition is stopped, buffer data _could_ be deleted
        ZImageGrayscale::Ptr last = (*m_lastRetrievedImage);
        ZImageGrayscale::Ptr snapshot(new ZImageGrayscale(last->width(), last->height(), last->xOffset(), last->yOffset(), last->bytesPerPixel()));
        snapshot->setBuffer( last->buffer() );
        snapshot->setNumber( last->number() );
        return snapshot;
    } else {
        //! create an event loop and stop it when a image is received
        QEventLoop eventLoop;

        QObject::connect(this, SIGNAL(newImageReceived(Z3D::ZImageGrayscale::Ptr)),
                         &eventLoop, SLOT(quit()), Qt::DirectConnection);

        startAcquisition();
        eventLoop.exec();

        /// make a copy because after acquisition is stopped, buffer data _could_ be deleted
        ZImageGrayscale::Ptr last = (*m_lastRetrievedImage);
        ZImageGrayscale::Ptr snapshot(new ZImageGrayscale(last->width(), last->height(), last->xOffset(), last->yOffset(), last->bytesPerPixel()));
        snapshot->setBuffer( last->buffer() );
        snapshot->setNumber( last->number() );

        /// stop acquisition once we have a copy of the image
        stopAcquisition();

        return snapshot;
    }
}

bool ZCameraBase::startAcquisition()
{
    /// increment counter
    m_simultaneousCapturesCount++;

    if (m_simultaneousCapturesCount == 1) {
        /// notify that a new acquisition has started
        emit acquisitionStarted();

        return true;
    } else if (m_simultaneousCapturesCount > 1) {
        CAMERA_WARNING("Camera is already capturing because it's being used somewhere else...")
    } else {
        /// something is wrong, m_sameTimeCapturesCount < 1
        CAMERA_WARNING("Invalid situation: m_sameTimeCapturesCount < 1")
    }

    return false;
}

bool ZCameraBase::stopAcquisition()
{
    /// decrement counter
    m_simultaneousCapturesCount--;

    if (m_simultaneousCapturesCount == 0) {
        /// notify that the acquisition has ended
        emit acquisitionStopped();

        return true;
    } else if (m_simultaneousCapturesCount > 0) {
        CAMERA_WARNING("Camera will continue capturing because it's being used somewhere else...")
    } else {
        /// something is wrong, m_sameTimeCapturesCount < 0
        CAMERA_WARNING("Invalid situation: m_sameTimeCapturesCount < 0")
        /// we stopped more times than we started?
        m_simultaneousCapturesCount = 0;
    }

    return false;
}

bool ZCameraBase::setAttribute(const QString &name, const QVariant &value)
{
    /// set attribute with notification
    return setAttribute(name, value, true);
}

void ZCameraBase::showSettingsDialog()
{
    if (!m_settingsWidget) {
        m_settingsWidget = new ZCameraSettingsWidget(this);
    }

    m_settingsWidget->show();
    m_settingsWidget->raise();
}

bool ZCameraBase::loadConfiguration(QString configFileName)
{
    CAMERA_DEBUG("Loading configuration from " + configFileName)
    QSettings settings(configFileName, QSettings::IniFormat);

    QStringList groupsList = settings.childGroups();
    foreach (QString group, groupsList) {
        CAMERA_DEBUG("Loading settings for preset: " + group)

        settings.beginGroup(group);

        QList<ZCameraAttribute> settingsGroup;

        QStringList settingsList = settings.childGroups();

        foreach (QString settingIndex, settingsList) {
            settings.beginGroup(settingIndex);

            ZCameraAttribute currAttr;
            currAttr.id = settings.value("name").toString();
            currAttr.path = settings.value("name").toString();
            currAttr.label = settings.value("name").toString();
            currAttr.value = settings.value("value");
            settingsGroup.append(currAttr);

//            qDebug() << currAttr.name << "=" << currAttr.value;

            settings.endGroup();
        }

        m_cameraPresets[group] = settingsGroup;

        settings.endGroup();
    }

    /// load base "initial" configuration and remove from presets
    loadPresetConfiguration( "BaseConfiguration" );
    m_cameraPresets.remove( "BaseConfiguration" );

    return true;
}

QStringList ZCameraBase::getPresets()
{
    return m_cameraPresets.keys();
}

bool ZCameraBase::loadPresetConfiguration(QString presetName)
{
    bool error = false;

    if (m_cameraPresets.contains(presetName)) {
        CAMERA_DEBUG("Loading preset " + presetName);
        const QList<ZCameraAttribute> &presetAttributesList = m_cameraPresets[presetName];
        foreach (ZCameraAttribute attr, presetAttributesList) {
            if (!setAttribute(attr.id, attr.value, false)) {
                error = true;
//                qWarning() << "unable to set attribute" << attr.name << "to" << attr.value;
            }
        }

        /// notify that some attributes have changed
        emit attributeChanged("","");
    } else {
        CAMERA_WARNING("Preset not found: " + presetName)
        error = true;
    }

    if (error) {
        CAMERA_WARNING("Something failed while loading preset " + presetName)
        return false;
    }

    return true;
}

bool ZCameraBase::setBufferSize(int bufferSize)
{
    m_imagesBuffer.resize(bufferSize);

    return true;
}

ZImageGrayscale::Ptr ZCameraBase::getNextBufferImage(int width, int height, int xOffset, int yOffset, int bytesPerPixel, void *externalBuffer)
{
    m_currentImageBufferIndex++;

    const int index = m_currentImageBufferIndex  % m_imagesBuffer.size();
    //qDebug() << "bufferIndex" << index;

    /// update last retrieved image
    m_lastRetrievedImage = &m_imagesBuffer[index];

    /// if null or different size
    if (!m_lastRetrievedImage->get() ||
            (*m_lastRetrievedImage)->width() != width ||
            (*m_lastRetrievedImage)->height() != height ||
            (*m_lastRetrievedImage)->xOffset() != xOffset ||
            (*m_lastRetrievedImage)->yOffset() != yOffset ||
            (*m_lastRetrievedImage)->bytesPerPixel() != bytesPerPixel) {

        if (!m_lastRetrievedImage->get())
            qDebug() << "last image was null";
        else
            qDebug() << "lastImage"
                     << "size" << (*m_lastRetrievedImage)->width() << "x" << (*m_lastRetrievedImage)->height()
                     << "offset" << (*m_lastRetrievedImage)->xOffset() << "," << (*m_lastRetrievedImage)->yOffset()
                     << "bytesPerPixel" << (*m_lastRetrievedImage)->bytesPerPixel();

        if (externalBuffer) {
            qDebug() << "creating new image from external buffer of size:" << width << "x" << height << " bytesPerPixel:" << bytesPerPixel;
            ZImageGrayscale::Ptr newImage(new ZImageGrayscale(width, height,
                                                            xOffset, yOffset,
                                                            bytesPerPixel,
                                                            externalBuffer));
            m_lastRetrievedImage->swap( newImage );
        } else {
            qDebug() << "creating new image of size:" << width << "x" << height << " bytesPerPixel:" << bytesPerPixel;
            ZImageGrayscale::Ptr newImage(new ZImageGrayscale(width, height,
                                                            xOffset, yOffset,
                                                            bytesPerPixel));
            m_lastRetrievedImage->swap( newImage );
        }
    }

    return *m_lastRetrievedImage;
}

} // namespace Z3D
