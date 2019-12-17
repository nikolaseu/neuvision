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

#include "zpyloncamera.h"

#include "ZCameraAcquisition/zcameraimage.h"

#include <QDebug>

#include <GenApi/INodeMap.h>

namespace Z3D
{

PylonCamera::BaslerImageHandler::BaslerImageHandler(PylonCamera *camera)
    : Pylon::CImageEventHandler()
    , m_camera(camera)
{
}

void PylonCamera::BaslerImageHandler::OnImageGrabbed(Pylon::CInstantCamera&, const Pylon::CGrabResultPtr& grabResult)
{
    if (grabResult->GetErrorCode()) {
        emit m_camera->error(QString(grabResult->GetErrorDescription()));
        return;
    }

    const auto width = grabResult->GetWidth();
    const auto height = grabResult->GetHeight();
    const auto offsetX = grabResult->GetOffsetX();
    const auto offsetY = grabResult->GetOffsetY();

    //! TODO properly set the correct bytesPerPixel value
    const int bytesPerPixel = 1;

    /// get image from buffer
    ZCameraImagePtr currentImage = m_camera->getNextBufferImage(width, height, offsetX, offsetY, bytesPerPixel);

    /// copy data
    currentImage->setBuffer(grabResult->GetBuffer());

    /// set image number
    currentImage->setNumber(grabResult->GetFrameNumber());

    /// notify
    emit m_camera->newImageReceived(currentImage);
}

PylonCamera::PylonCamera(Pylon::IPylonDevice *device, QObject *parent)
    : ZCameraBase(parent)
{
    m_camera.Attach(device);

    m_image_handler = new BaslerImageHandler(this);

    m_camera.RegisterImageEventHandler(m_image_handler, Pylon::RegistrationMode_Append, Pylon::Cleanup_Delete);
    m_camera.Open();

    m_uuid = QString("ZPylon-%1").arg(m_camera.GetDeviceInfo().GetSerialNumber().c_str());
}

PylonCamera::~PylonCamera()
{
    m_camera.Close();
    m_camera.DetachDevice();
}

bool PylonCamera::startAcquisition()
{
    if (!ZCameraBase::startAcquisition())
        return false;

    qDebug() << Q_FUNC_INFO;

    m_camera.StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);

    return true;
}

bool PylonCamera::stopAcquisition()
{
    if (!ZCameraBase::stopAcquisition())
        return false;

    qDebug() << Q_FUNC_INFO;

    m_camera.StopGrabbing();

    return true;
}

QList<ZCameraInterface::ZCameraAttribute> PylonCamera::getAllAttributes()
{
    QList<ZCameraInterface::ZCameraAttribute> parameters;

    GenApi::INodeMap &nodemap = m_camera.GetNodeMap();
    GenApi::NodeList_t nodelist;
    nodemap.GetNodes(nodelist);

    for (const auto &node : nodelist) {
        if (node->GetPrincipalInterfaceType() != GenApi::EInterfaceType::intfICategory) {
            continue;
        }

        QString categoryName = QString::fromLatin1(node->GetName());
        GenApi::NodeList_t childrens;
        node->GetChildren(childrens);

        for (const auto node : childrens) {
            ZCameraInterface::ZCameraAttribute attr;
            attr.readable = GenApi::IsReadable(node);
            attr.writable = GenApi::IsWritable(node);
            attr.id = node->GetName();
            attr.path = QString("%1::%2").arg(categoryName).arg(QString(node->GetName()));
            attr.description = node->GetDescription();
            attr.label = node->GetDisplayName();

            switch (node->GetPrincipalInterfaceType()) {
            case GenApi::EInterfaceType::intfIValue:       //!< IValue interface
                qWarning() << "unhandled type GenApi::EInterfaceType::intfIValue" << attr.path;
                continue;
            case GenApi::EInterfaceType::intfIBase:        //!< IBase interface
                qWarning() << "unhandled type GenApi::EInterfaceType::intfIBase" << attr.path;
                continue;
            case GenApi::EInterfaceType::intfIInteger: {    //!< IInteger interface
                GenApi::CIntegerPtr integerNode(node);
                attr.type =  ZCameraInterface::CameraAttributeTypeInt;
                attr.value = integerNode->GetValue();
                attr.minimumValue = integerNode->GetMin();
                attr.maximumValue = integerNode->GetMax();
                break;
            }
            case GenApi::EInterfaceType::intfIBoolean: {    //!< IBoolean interface
                GenApi::CBooleanPtr booleanNode(node);
                attr.type =  ZCameraInterface::CameraAttributeTypeBool;
                attr.value = booleanNode->GetValue();
                break;
            }
            case GenApi::EInterfaceType::intfICommand: {    //!< ICommand interface
                attr.type =  ZCameraInterface::CameraAttributeTypeCommand;
                break;
            }
            case GenApi::EInterfaceType::intfIFloat: {      //!< IFloat interface
                GenApi::CFloatPtr floatNode(node);
                attr.type =  ZCameraInterface::CameraAttributeTypeFloat;
                attr.value = floatNode->GetValue();
                attr.minimumValue = floatNode->GetMin();
                attr.maximumValue = floatNode->GetMax();
                break;
            }
            case GenApi::EInterfaceType::intfIString: {     //!< IString interface
                GenApi::CStringPtr stringNode(node);
                attr.type =  ZCameraInterface::CameraAttributeTypeString;
                attr.value = QString(stringNode->GetValue().c_str());
                break;
            }
            case GenApi::EInterfaceType::intfIRegister:    //!< IRegister interface
                qWarning() << "unhandled type GenApi::EInterfaceType::intfIRegister" << attr.path;
                continue;
            case GenApi::EInterfaceType::intfICategory:    //!< ICategory interface
                qWarning() << "unhandled type GenApi::EInterfaceType::intfICategory" << attr.path;
                continue;
            case GenApi::EInterfaceType::intfIEnumeration: {    //!< IEnumeration interface
                try {
                    GenApi::CEnumerationPtr enumNode(node);
                    attr.type =  ZCameraInterface::CameraAttributeTypeEnum;
                    GenApi::NodeList_t entries;
                    enumNode->GetEntries(entries);
                    for (const auto &entry : entries) {
                        attr.enumNames << QString(entry->GetDisplayName().c_str());
                    }
                    qDebug() << attr.path << enumNode->ToString().c_str();
                    attr.enumValue = enumNode->GetIntValue();
                } catch (...) {
                    qWarning() << "exception enumerating properties for" << attr.path;
                }
                break;
            }
            case GenApi::EInterfaceType::intfIEnumEntry:   //!< IEnumEntry interface
                qWarning() << "unhandled type GenApi::EInterfaceType::intfIEnumEntry" << attr.path;
                break;
            case GenApi::EInterfaceType::intfIPort:         //!< IPort interface
                qWarning() << "unhandled type GenApi::EInterfaceType::intfIPort" << attr.path;
                continue;
            default:
                qWarning() << "unknown type for" << attr.path;
            }

            parameters << attr;
        }
    }

    return parameters;
}

QVariant PylonCamera::getAttribute(const QString &name) const
{
    Q_UNUSED(name);

    //! TODO
    return QVariant("INVALID");
}

bool PylonCamera::setAttribute(const QString &name, const QVariant &value, bool notify)
{
    qDebug() << "trying to set " << name << "to" << value;

    auto node = m_camera.GetNodeMap().GetNode(qPrintable(name));
    if (!GenApi::IsWritable(node)) {
        qWarning() << "camera attribute is not writable" << name;
        return false;
    }

    bool changed = false;

    switch (node->GetPrincipalInterfaceType()) {
    case GenApi::EInterfaceType::intfIValue:       //!< IValue interface
        qWarning() << "unhandled type GenApi::EInterfaceType::intfIValue" << node->GetName(true);
        break;
    case GenApi::EInterfaceType::intfIBase:        //!< IBase interface
        qWarning() << "unhandled type GenApi::EInterfaceType::intfIBase" << node->GetName(true);
        break;
    case GenApi::EInterfaceType::intfIInteger: {    //!< IInteger interface
        GenApi::CIntegerPtr integerNode(node);
        integerNode->SetValue(value.toInt());
        changed = true;
        break;
    }
    case GenApi::EInterfaceType::intfIBoolean: {    //!< IBoolean interface
        GenApi::CBooleanPtr booleanNode(node);
        booleanNode->SetValue(value.toBool());
        changed = true;
        break;
    }
    case GenApi::EInterfaceType::intfICommand: {    //!< ICommand interface
        GenApi::CCommandPtr commandNode(node);
        commandNode->Execute();
        changed = true;
        break;
    }
    case GenApi::EInterfaceType::intfIFloat: {      //!< IFloat interface
        GenApi::CFloatPtr floatNode(node);
        floatNode->SetValue(value.toDouble());
        changed = true;
        break;
    }
    case GenApi::EInterfaceType::intfIString: {     //!< IString interface
        GenApi::CStringPtr stringNode(node);
        stringNode->SetValue(qPrintable(value.toString()));
        changed = true;
        break;
    }
    case GenApi::EInterfaceType::intfIRegister:    //!< IRegister interface
        qWarning() << "unhandled type GenApi::EInterfaceType::intfIRegister" << node->GetName(true);
        break;
    case GenApi::EInterfaceType::intfICategory:    //!< ICategory interface
        qWarning() << "unhandled type GenApi::EInterfaceType::intfICategory" << node->GetName(true);
        break;
    case GenApi::EInterfaceType::intfIEnumeration: {    //!< IEnumeration interface
        GenApi::CEnumerationPtr enumNode(node);
        GenApi::NodeList_t entries;
        enumNode->GetEntries(entries);
        QString valueString = value.toString();
        for (const auto &entry : entries) {
            GenApi::CEnumEntryPtr entryNode(entry);
            if (valueString.compare(QString(entry->GetDisplayName())) == 0) {
                enumNode->SetIntValue(entryNode->GetValue());
                changed = true;
                break;
            }
        }
        break;
    }
    case GenApi::EInterfaceType::intfIEnumEntry:   //!< IEnumEntry interface
        qWarning() << "unhandled type GenApi::EInterfaceType::intfIEnumEntry" << node->GetName(true);
        break;
    case GenApi::EInterfaceType::intfIPort:         //!< IPort interface
        qWarning() << "unhandled type GenApi::EInterfaceType::intfIPort" << node->GetName(true);
        break;
    default:
        qWarning() << "unknown type for" << node->GetName(true);
        break;
    }

    if (!changed) {
        qWarning() << "failed to change" << node->GetName(true) << "to" << value;
    } else {
        if (notify) {
            emit attributeChanged(name, value);
        }
    }

    return changed;
}

} // namespace Z3D
