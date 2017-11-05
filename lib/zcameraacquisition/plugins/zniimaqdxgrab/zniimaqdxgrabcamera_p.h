/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

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
    void newImageReceived(Z3D::ZCameraImagePtr image);
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
