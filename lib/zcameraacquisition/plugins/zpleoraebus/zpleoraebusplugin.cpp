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

#include "zpleoraebusplugin.h"

#include "zpleoraebuscamera.h"

#include <QDebug>
#include <QStringList>

#include <PvBuffer.h>
//#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvDeviceU3V.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvStreamU3V.h>
#include <PvSystem.h>


/// static member initialization
PvSystem *Z3D::ZPleoraeBUSPlugin::s_pvSystem = NULL;

namespace Z3D
{

QString ZPleoraeBUSPlugin::id() const
{
    return QString("ZPleoraeBUS");
}

QString ZPleoraeBUSPlugin::name() const
{
    return QString("Pleora eBUS SDK");
}

QString ZPleoraeBUSPlugin::version() const
{
    return QString(Z3D_VERSION_STR);
}

QList<ZCameraInfo *> ZPleoraeBUSPlugin::getConnectedCameras()
{
    QList<ZCameraInfo *> camerasList;

    /// Find all devices on the network.
    PvSystem *pvSystem = getPvSystem();
    PvResult lResult = pvSystem->Find();
    if ( !lResult.IsOK() ) {
        qWarning() << "PvSystem::Find Error: " << lResult.GetCodeString().GetAscii();
        return camerasList;
    }

    /// Go through all interfaces
    uint32_t lInterfaceCount = pvSystem->GetInterfaceCount();
    for ( uint32_t x = 0; x < lInterfaceCount; x++ ) {
        qDebug() << "Interface " << x;

        /// Get pointer to the interface.
        const PvInterface* lInterface = pvSystem->GetInterface( x );

        /// Is it a PvNetworkAdapter?
        const PvNetworkAdapter* lNIC = dynamic_cast<const PvNetworkAdapter*>( lInterface );
        if ( lNIC != NULL ) {
            qDebug() << "  NETWORK ADAPTER";
            qDebug() << "  * MAC Address: " << lNIC->GetMACAddress().GetAscii();
            qDebug() << "  * IP Address: "  << lNIC->GetIPAddress().GetAscii();
            qDebug() << "  * Subnet Mask: " << lNIC->GetSubnetMask().GetAscii();
        }

        /// Is it a PvUSBHostController?
        const PvUSBHostController* lUSB = dynamic_cast<const PvUSBHostController*>( lInterface );
        if ( lUSB != NULL ) {
            qDebug() << "  USB HOST CONTROLLER";
            qDebug() << "  * Name: " << lUSB->GetName().GetAscii();
        }

        /// Go through all the devices attached to the interface
        uint32_t lDeviceCount = lInterface->GetDeviceCount();
        for ( uint32_t y = 0; y < lDeviceCount ; y++ ) {
            qDebug() << "  Device " << y;

            const PvDeviceInfo *lDeviceInfo = lInterface->GetDeviceInfo( y );
            QString displayID = QString::fromStdString(lDeviceInfo->GetDisplayID().GetAscii());
            qDebug() << "    * Display ID: " << displayID;
            qDebug() << "    * Serial number: " << lDeviceInfo->GetSerialNumber().GetAscii();

            QVariantMap extraData;
            extraData["ManufacturerInfo"] = QString::fromStdString(lDeviceInfo->GetManufacturerInfo().GetAscii());
            extraData["ModelName"] = QString::fromStdString(lDeviceInfo->GetModelName().GetAscii());
            extraData["SerialNumber"] = QString::fromStdString(lDeviceInfo->GetSerialNumber().GetAscii());
            extraData["UniqueID"] = QString::fromStdString(lDeviceInfo->GetUniqueID().GetAscii());
            extraData["UserDefinedName"] = QString::fromStdString(lDeviceInfo->GetUserDefinedName().GetAscii());
            extraData["VendorName"] = QString::fromStdString(lDeviceInfo->GetVendorName().GetAscii());

            const PvDeviceInfoGEV* lDeviceInfoGEV = dynamic_cast<const PvDeviceInfoGEV*>( lDeviceInfo );
            const PvDeviceInfoU3V *lDeviceInfoU3V = dynamic_cast<const PvDeviceInfoU3V *>( lDeviceInfo );
            const PvDeviceInfoUSB *lDeviceInfoUSB = dynamic_cast<const PvDeviceInfoUSB *>( lDeviceInfo );
            const PvDeviceInfoPleoraProtocol* lDeviceInfoPleora = dynamic_cast<const PvDeviceInfoPleoraProtocol*>( lDeviceInfo );

            if ( lDeviceInfoGEV != NULL ) { /// Is it a GigE Vision device?
                qDebug() << "    * GIGE VISION DEVICE";
                qDebug() << "      * MAC Address: " << lDeviceInfoGEV->GetMACAddress().GetAscii();
                qDebug() << "      * IP Address: " << lDeviceInfoGEV->GetIPAddress().GetAscii();

                extraData["MACAddress"]   = QString::fromStdString(lDeviceInfoGEV->GetMACAddress().GetAscii());
                extraData["IPAddress"]    = QString::fromStdString(lDeviceInfoGEV->GetIPAddress().GetAscii());
            } else if ( lDeviceInfoU3V != NULL ) { /// Is it a USB3 Vision device?
                qDebug() << "    * USB3 VISION DEVICE";
                qDebug() << "      * GUID: " << lDeviceInfoU3V->GetDeviceGUID().GetAscii();
                qDebug() << "      * S/N: " << lDeviceInfoU3V->GetSerialNumber().GetAscii();
                qDebug() << "      * Speed: " << lUSB->GetSpeed();
            } else if ( lDeviceInfoUSB != NULL ) { /// Is it an unidentified USB device?
                qDebug() << "    * UNKNOWN USB DEVICE";
            } else if ( lDeviceInfoPleora != NULL ) { /// Is it a Pleora Protocol device?
                qDebug() << "    * PLEORA PROTOCOL DEVICE";
                qDebug() << "      * MAC Address: " << lDeviceInfoPleora->GetMACAddress().GetAscii();
                qDebug() << "      * IP Address: " << lDeviceInfoPleora->GetIPAddress().GetAscii();

                extraData["MACAddress"]   = QString::fromStdString(lDeviceInfoPleora->GetMACAddress().GetAscii());
                extraData["IPAddress"]    = QString::fromStdString(lDeviceInfoPleora->GetIPAddress().GetAscii());
            }

            camerasList << new ZCameraInfo(this, displayID, extraData);
        }
    }


    return camerasList;
}

ZCameraInterface::Ptr ZPleoraeBUSPlugin::getCamera(QVariantMap options)
{
    QString cameraMACAddress = options.value("MACAddress").toString();
    return ZPleoraeBUSCamera::getCameraByMAC(cameraMACAddress);
}

PvSystem *ZPleoraeBUSPlugin::getPvSystem()
{
    if (!s_pvSystem)
        s_pvSystem = new PvSystem;

    return s_pvSystem;
}

} // namespace Z3D
