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

#include "ZCameraAcquisition/zcamerasettingswidget.h"
#include "ui_zcamerasettingswidget.h"

#include "ZCameraAcquisition/zcamerainterface.h"

#include "ZCore/zlogging.h"

#include "qtpropertymanager.h"
#include "qteditorfactory.h"
#include "qttreepropertybrowser.h"

#include <QTimer>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zcameraacquisition", QtInfoMsg)

namespace Z3D
{

ZCameraSettingsWidget::ZCameraSettingsWidget(const ZCameraWeakPtr &camera, QWidget *parent)
    : ZWidget(parent)
    , ui(new Ui::ZCameraSettingsWidget)
    , m_camera(camera)
{
    ui->setupUi(this);

//    setWindowTitle(tr("Camera settings - %1").arg(m_camera->uuid()));

    m_propertyBrowser = new QtTreePropertyBrowser(this);

    m_groupManager = new QtGroupPropertyManager(this);

    m_boolPropertyManager = new QtBoolPropertyManager(this);
    auto checkBoxFactory = new QtCheckBoxFactory(this);
    m_propertyBrowser->setFactoryForManager(m_boolPropertyManager, checkBoxFactory);

    m_stringPropertyManager = new QtStringPropertyManager(this);
    auto lineEditFactory = new QtLineEditFactory(this);
    m_propertyBrowser->setFactoryForManager(m_stringPropertyManager, lineEditFactory);

    m_intPropertyManager = new QtIntPropertyManager(this);
    auto spinBoxFactory = new QtSpinBoxFactory(this);
    m_propertyBrowser->setFactoryForManager(m_intPropertyManager, spinBoxFactory);

    m_longPropertyManager = new QtStringPropertyManager(this);
    auto longEditFactory = new QtLineEditFactory(this);
    m_propertyBrowser->setFactoryForManager(m_longPropertyManager, longEditFactory);

    m_floatPropertyManager = new QtDoublePropertyManager(this);
    auto doubleSpinBoxFactory = new QtDoubleSpinBoxFactory(this);
    m_propertyBrowser->setFactoryForManager(m_floatPropertyManager, doubleSpinBoxFactory);

    m_enumPropertyManager = new QtEnumPropertyManager(this);
    auto enumEditorFactory = new QtEnumEditorFactory(this);
    m_propertyBrowser->setFactoryForManager(m_enumPropertyManager, enumEditorFactory);

    m_commandPropertyManager = new QtBoolPropertyManager(this);
    auto commandCheckBoxFactory = new QtCheckBoxFactory(this);
    m_propertyBrowser->setFactoryForManager(m_commandPropertyManager, commandCheckBoxFactory);

    ui->propertyBrowserLayout->addWidget(m_propertyBrowser);

    QObject::connect(m_camera.data(), &ZCameraInterface::attributeChanged,
                     this, &ZCameraSettingsWidget::onCameraAttributeChanged);

    QObject::connect(ui->refreshButton, &QPushButton::clicked,
                     this, &ZCameraSettingsWidget::updateProperties);

    QTimer::singleShot(0, this, &ZCameraSettingsWidget::updateProperties);
}

ZCameraSettingsWidget::~ZCameraSettingsWidget()
{
    delete ui;
}

void ZCameraSettingsWidget::propertyChanged(QtProperty *property)
{
    if (!m_camera) {
        return;
    }

    auto found = false;

    for (const auto *mapProperty : m_propertiesList) {
        if (mapProperty == property) {
            found = true;
            const auto &attrName = property->whatsThis(); // FIXME ugly hack
            const auto manager = property->propertyManager();

            if (manager == m_boolPropertyManager) {
                m_camera->setAttribute(attrName, m_boolPropertyManager->value( property ));
            } else if (manager == m_commandPropertyManager) {
                m_camera->setAttribute(attrName, m_commandPropertyManager->value( property ));
            } else if (manager == m_enumPropertyManager) {
                //m_camera->setAttribute(attrName, m_enumPropertyManager->value( property ));
                m_camera->setAttribute(attrName, property->valueText());
            } else if (manager == m_floatPropertyManager) {
                m_camera->setAttribute(attrName, m_floatPropertyManager->value( property ));
            } else if (manager == m_intPropertyManager) {
                m_camera->setAttribute(attrName, m_intPropertyManager->value( property ));
            } else if (manager == m_longPropertyManager) {
                m_camera->setAttribute(attrName, m_longPropertyManager->value( property ));
            } else if (manager == m_stringPropertyManager) {
                m_camera->setAttribute(attrName, m_stringPropertyManager->value( property ));
            } else {
                m_camera->setAttribute(attrName, property->valueText());
            }

            break;
        }
    }

    if (!found)
        zWarning() << "property changed not found in properties list:" << property->propertyName();
}

void ZCameraSettingsWidget::updateProperties()
{
    if (!m_camera) {
        return;
    }

    /// disconnect signals to avoid re-setting parameters when refreshed
    QObject::disconnect(m_boolPropertyManager, &QtBoolPropertyManager::propertyChanged,
                     this, &ZCameraSettingsWidget::propertyChanged);
    QObject::disconnect(m_stringPropertyManager, &QtStringPropertyManager::propertyChanged,
                     this, &ZCameraSettingsWidget::propertyChanged);
    QObject::disconnect(m_intPropertyManager, &QtIntPropertyManager::propertyChanged,
                     this, &ZCameraSettingsWidget::propertyChanged);
    QObject::disconnect(m_longPropertyManager, &QtStringPropertyManager::propertyChanged,
                     this, &ZCameraSettingsWidget::propertyChanged);
    QObject::disconnect(m_floatPropertyManager, &QtDoublePropertyManager::propertyChanged,
                     this, &ZCameraSettingsWidget::propertyChanged);
    QObject::disconnect(m_enumPropertyManager, &QtEnumPropertyManager::propertyChanged,
                     this, &ZCameraSettingsWidget::propertyChanged);

    /// disable all, later they will be enabled again (when available)
    for (auto *property : m_propertiesList) {
        if (property->propertyManager() != m_groupManager) {
            property->setEnabled(false);
        }
    }

    for (const auto &attribute : m_camera->getAllAttributes()) {
        QtProperty *m_currentProperty;
        if (m_propertiesList.contains(attribute.path)) {
            /// already existed
            m_currentProperty = m_propertiesList[attribute.path];
        } else {
            /// new property
            QtProperty *parentCategory = nullptr;
            auto categoriesList = attribute.path.split("::");
            QString attrName;
            if (categoriesList.size() > 1) {
                /// its a sub property, got to find/create the category
                int propertyIndex = categoriesList.size() - 1;
                QString fullCategory;
                for (int i=0; i<propertyIndex; ++i) {
                    /// get parent category
                    fullCategory = QString();
                    for (int j=0; j<=i; ++j)
                        fullCategory += QString("::%1").arg(categoriesList[j]);

                    /// add group for each category
                    if (!m_propertiesList.contains(fullCategory)) {
                        m_currentProperty = m_groupManager->addProperty(categoriesList[i]);
                        m_propertiesList[fullCategory] = m_currentProperty;
                    } else {
                        m_currentProperty = m_propertiesList[fullCategory];
                    }

                    if (parentCategory) {
                        /// sub category
                        parentCategory->addSubProperty( m_currentProperty );
                    } else {
                        /// top level category
                        m_propertyBrowser->addProperty( m_currentProperty );
                    }

                    parentCategory = m_currentProperty;
                }

                attrName = !attribute.label.isNull() ? attribute.label : categoriesList[propertyIndex];
            } else {
                attrName = !attribute.label.isNull() ? attribute.label : attribute.id;
            }

            /// create property
            switch (attribute.type) {
            case Z3D::ZCameraInterface::CameraAttributeTypeUnknown:
                //zWarning() << "Unknown attribute type" << attribute.name;
                m_currentProperty = m_groupManager->addProperty(attrName);
                break;
            case Z3D::ZCameraInterface::CameraAttributeTypeBool:
                m_currentProperty = m_boolPropertyManager->addProperty(attrName);
                break;
            case Z3D::ZCameraInterface::CameraAttributeTypeString:
                m_currentProperty = m_stringPropertyManager->addProperty(attrName);
                break;
            case Z3D::ZCameraInterface::CameraAttributeTypeInt:
                m_currentProperty = m_intPropertyManager->addProperty(attrName);
                break;
            case Z3D::ZCameraInterface::CameraAttributeTypeLong:
                m_currentProperty = m_longPropertyManager->addProperty(attrName);
                break;
            case Z3D::ZCameraInterface::CameraAttributeTypeFloat:
                m_currentProperty = m_floatPropertyManager->addProperty(attrName);
                break;
            case Z3D::ZCameraInterface::CameraAttributeTypeEnum:
                m_currentProperty = m_enumPropertyManager->addProperty(attrName);
                break;
            case Z3D::ZCameraInterface::CameraAttributeTypeCommand:
                //zWarning() << "Command attribute type not supported yet" << attribute.name;
                /*
                m_currentProperty = m_stringPropertyManager->addProperty(attrName);
                m_currentProperty->setEnabled(false);
                m_stringPropertyManager->setValue(m_currentProperty, tr("Commands not supported yet!"));
                */
                m_currentProperty = m_enumPropertyManager->addProperty(attrName);
                break;
            }

            if (!m_currentProperty) {
                zWarning() << "Unable to add property" << attribute.id;
                continue;
            }

            m_currentProperty->setToolTip(attribute.description);

            m_propertiesList[attribute.path] = m_currentProperty;

            if (parentCategory)
                parentCategory->addSubProperty( m_currentProperty );
            else
                m_propertyBrowser->addProperty( m_currentProperty );
        }

        m_currentProperty->setWhatsThis(attribute.id); // FIXME ugly hack

        /// set value
        switch (attribute.type) {
        case Z3D::ZCameraInterface::CameraAttributeTypeUnknown:
            //zWarning() << "Unknown attribute type" << attribute.name;
            //m_currentProperty = m_groupManager->addProperty(attrName);
            m_currentProperty->setEnabled(attribute.readable && attribute.writable);
            break;
        case Z3D::ZCameraInterface::CameraAttributeTypeBool:
            //m_currentProperty = m_boolPropertyManager->addProperty(attrName);
            m_boolPropertyManager->setValue(m_currentProperty, attribute.value.toBool());
            //m_boolPropertyManager->setReadOnly(m_currentProperty, !attribute.writable);
            m_currentProperty->setEnabled(attribute.readable && attribute.writable);
            break;
        case Z3D::ZCameraInterface::CameraAttributeTypeString:
            //m_currentProperty = m_stringPropertyManager->addProperty(attrName);
            m_stringPropertyManager->setValue(m_currentProperty, attribute.value.toString());
//            m_stringPropertyManager->setReadOnly(m_currentProperty, !attribute.writable);
            m_currentProperty->setEnabled(attribute.readable && attribute.writable);
            break;
        case Z3D::ZCameraInterface::CameraAttributeTypeInt:
            //m_currentProperty = m_intPropertyManager->addProperty(attrName);
            m_intPropertyManager->setValue(m_currentProperty, attribute.value.toInt());
            if (attribute.maximumValue != std::numeric_limits<double>::max())
                m_intPropertyManager->setMaximum(m_currentProperty, attribute.maximumValue);
            if (attribute.minimumValue != std::numeric_limits<double>::min())
                m_intPropertyManager->setMinimum(m_currentProperty, attribute.minimumValue);
//            m_intPropertyManager->setReadOnly(m_currentProperty, !attribute.writable);
            m_currentProperty->setEnabled(attribute.readable && attribute.writable);
            break;
        case Z3D::ZCameraInterface::CameraAttributeTypeLong:
            //m_currentProperty = m_longPropertyManager->addProperty(attrName);
            m_longPropertyManager->setValue(m_currentProperty, attribute.value.toString());
//            m_longPropertyManager->setReadOnly(m_currentProperty, !attribute.writable);
            m_currentProperty->setEnabled(attribute.readable && attribute.writable);
            break;
        case Z3D::ZCameraInterface::CameraAttributeTypeFloat:
            //m_currentProperty = m_floatPropertyManager->addProperty(attrName);
            m_floatPropertyManager->setValue(m_currentProperty, attribute.value.toDouble());
            if (attribute.maximumValue != std::numeric_limits<double>::max())
                m_floatPropertyManager->setMaximum(m_currentProperty, attribute.maximumValue);
            if (attribute.minimumValue != std::numeric_limits<double>::min())
                m_floatPropertyManager->setMinimum(m_currentProperty, attribute.minimumValue);
//            m_floatPropertyManager->setReadOnly(m_currentProperty, !attribute.writable);
            m_currentProperty->setEnabled(attribute.readable && attribute.writable);
            break;
        case Z3D::ZCameraInterface::CameraAttributeTypeEnum:
            //m_currentProperty = m_enumPropertyManager->addProperty(attrName);
            m_enumPropertyManager->setValue(m_currentProperty, attribute.enumValue);
            m_enumPropertyManager->setEnumNames(m_currentProperty, attribute.enumNames);
            //m_enumPropertyManager->setReadOnly(m_currentProperty, !attribute.writable);
            m_currentProperty->setEnabled(attribute.readable && attribute.writable);
            break;
        case Z3D::ZCameraInterface::CameraAttributeTypeCommand:
            //zWarning() << "Command attribute type not supported yet" << attribute.name;
            //m_currentProperty = m_groupManager->addProperty(attrName);
            m_enumPropertyManager->setValue(m_currentProperty, 0);
            m_enumPropertyManager->setEnumNames(m_currentProperty, QStringList() << "[COMMAND]" << "Execute command");
            m_currentProperty->setEnabled(attribute.writable);
            break;
        }

    }

    /// connect signal again
    QObject::connect(m_boolPropertyManager, &QtBoolPropertyManager::propertyChanged,
                     this, &ZCameraSettingsWidget::propertyChanged);
    QObject::connect(m_stringPropertyManager, &QtStringPropertyManager::propertyChanged,
                     this, &ZCameraSettingsWidget::propertyChanged);
    QObject::connect(m_intPropertyManager, &QtIntPropertyManager::propertyChanged,
                     this, &ZCameraSettingsWidget::propertyChanged);
    QObject::connect(m_longPropertyManager, &QtStringPropertyManager::propertyChanged,
                     this, &ZCameraSettingsWidget::propertyChanged);
    QObject::connect(m_floatPropertyManager, &QtDoublePropertyManager::propertyChanged,
                     this, &ZCameraSettingsWidget::propertyChanged);
    QObject::connect(m_enumPropertyManager, &QtEnumPropertyManager::propertyChanged,
                     this, &ZCameraSettingsWidget::propertyChanged);
}

void ZCameraSettingsWidget::onCameraAttributeChanged(const QString &name, const QVariant& value)
{
    Q_UNUSED(value)
    QtProperty *m_currentProperty;
    if (m_propertiesList.contains(name)) {
        /// already existed
        m_currentProperty = m_propertiesList[name];
    }
}

} // namespace Z3D
