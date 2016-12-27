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
