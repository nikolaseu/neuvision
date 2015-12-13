#include "zcamerasettingswidget.h"
#include "ui_zcamerasettingswidget.h"

#include "qtpropertymanager.h"
#include "qteditorfactory.h"
#include "qttreepropertybrowser.h"

#include <QDebug>
#include <QTimer>

namespace Z3D
{

ZCameraSettingsWidget::ZCameraSettingsWidget(ZCameraInterface::WeakPtr camera, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ZCameraSettingsWidget),
    m_camera(camera)
{
    ui->setupUi(this);

    setWindowTitle(tr("Camera settings - %1").arg(m_camera->uuid()));

    m_propertyBrowser = new QtTreePropertyBrowser(this);

    m_groupManager = new QtGroupPropertyManager(this);

    m_boolPropertyManager = new QtBoolPropertyManager(this);
    QtCheckBoxFactory *checkBoxFactory = new QtCheckBoxFactory(this);
    m_propertyBrowser->setFactoryForManager(m_boolPropertyManager, checkBoxFactory);

    m_stringPropertyManager = new QtStringPropertyManager(this);
    QtLineEditFactory *lineEditFactory = new QtLineEditFactory(this);
    m_propertyBrowser->setFactoryForManager(m_stringPropertyManager, lineEditFactory);

    m_intPropertyManager = new QtIntPropertyManager(this);
    QtSpinBoxFactory *spinBoxFactory = new QtSpinBoxFactory(this);
    m_propertyBrowser->setFactoryForManager(m_intPropertyManager, spinBoxFactory);

    m_longPropertyManager = new QtStringPropertyManager(this);
    QtLineEditFactory *longEditFactory = new QtLineEditFactory(this);
    m_propertyBrowser->setFactoryForManager(m_longPropertyManager, longEditFactory);

    m_floatPropertyManager = new QtDoublePropertyManager(this);
    QtDoubleSpinBoxFactory *doubleSpinBoxFactory = new QtDoubleSpinBoxFactory(this);
    m_propertyBrowser->setFactoryForManager(m_floatPropertyManager, doubleSpinBoxFactory);

    m_enumPropertyManager = new QtEnumPropertyManager(this);
    QtEnumEditorFactory *enumEditorFactory = new QtEnumEditorFactory(this);
    m_propertyBrowser->setFactoryForManager(m_enumPropertyManager, enumEditorFactory);

    m_commandPropertyManager = new QtBoolPropertyManager(this);
    QtCheckBoxFactory *commandCheckBoxFactory = new QtCheckBoxFactory(this);
    m_propertyBrowser->setFactoryForManager(m_commandPropertyManager, commandCheckBoxFactory);

    ui->propertyBrowserLayout->addWidget(m_propertyBrowser);

    QObject::connect(m_camera.data(), SIGNAL(attributeChanged(QString,QVariant)),
                     this, SLOT(onCameraAttributeChanged(QString,QVariant)));

    QObject::connect(ui->refreshButton, SIGNAL(clicked()),
                     this, SLOT(updateProperties()));

    QTimer::singleShot(0, this, SLOT(updateProperties()));
}

ZCameraSettingsWidget::~ZCameraSettingsWidget()
{
    delete ui;
}

void ZCameraSettingsWidget::propertyChanged(QtProperty *property)
{
    if (!m_camera)
        return;

    bool found = false;

    foreach (const QString &attrName, m_propertiesList.keys()) {
        QtProperty *mapProperty = m_propertiesList[attrName];
        if (mapProperty == property) {
            found = true;

            const QtAbstractPropertyManager *manager = property->propertyManager();

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
        qWarning() << "property changed not found in properties list:" << property->propertyName();
}

void ZCameraSettingsWidget::updateProperties()
{
    if (!m_camera)
        return;

    /// disconnect signals to avoid re-setting parameters when refreshed
    QObject::disconnect(m_boolPropertyManager,   SIGNAL(propertyChanged(QtProperty*)),
                     this, SLOT(propertyChanged(QtProperty*)));
    QObject::disconnect(m_stringPropertyManager, SIGNAL(propertyChanged(QtProperty*)),
                     this, SLOT(propertyChanged(QtProperty*)));
    QObject::disconnect(m_intPropertyManager,    SIGNAL(propertyChanged(QtProperty*)),
                     this, SLOT(propertyChanged(QtProperty*)));
    QObject::disconnect(m_longPropertyManager, SIGNAL(propertyChanged(QtProperty*)),
                     this, SLOT(propertyChanged(QtProperty*)));
    QObject::disconnect(m_floatPropertyManager,  SIGNAL(propertyChanged(QtProperty*)),
                     this, SLOT(propertyChanged(QtProperty*)));
    QObject::disconnect(m_enumPropertyManager,   SIGNAL(propertyChanged(QtProperty*)),
                     this, SLOT(propertyChanged(QtProperty*)));

    /// disable all, later they will be enabled again (when available)
    foreach (QtProperty *property, m_propertiesList) {
        if (property->propertyManager() != m_groupManager)
            property->setEnabled(false);
    }

    QList<Z3D::ZCameraInterface::ZCameraAttribute> attributes = m_camera->getAllAttributes();

    foreach(Z3D::ZCameraInterface::ZCameraAttribute attribute, attributes) {

        QtProperty *m_currentProperty;
        if (m_propertiesList.contains(attribute.name)) {
            /// already existed
            m_currentProperty = m_propertiesList[attribute.name];
        } else {
            /// new property
            QtProperty *parentCategory = 0;
            QStringList categoriesList = attribute.name.split("::");
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

                attrName = categoriesList[propertyIndex];
            } else {
                attrName = attribute.name;
            }

            /// create property
            switch (attribute.type) {
            case Z3D::ZCameraInterface::CameraAttributeTypeUnknown:
                //qWarning() << "Unknown attribute type" << attribute.name;
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
                //qWarning() << "Command attribute type not supported yet" << attribute.name;
                /*
                m_currentProperty = m_stringPropertyManager->addProperty(attrName);
                m_currentProperty->setEnabled(false);
                m_stringPropertyManager->setValue(m_currentProperty, tr("Commands not supported yet!"));
                */
                m_currentProperty = m_enumPropertyManager->addProperty(attrName);
                break;
            }

            if (!m_currentProperty) {
                qWarning() << "Unable to add property" << attribute.name;
                continue;
            }

            m_currentProperty->setToolTip(attribute.description);

            m_propertiesList[attribute.name] = m_currentProperty;

            if (parentCategory)
                parentCategory->addSubProperty( m_currentProperty );
            else
                m_propertyBrowser->addProperty( m_currentProperty );
        }

        /// set value
        switch (attribute.type) {
        case Z3D::ZCameraInterface::CameraAttributeTypeUnknown:
            //qWarning() << "Unknown attribute type" << attribute.name;
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
            m_stringPropertyManager->setReadOnly(m_currentProperty, !attribute.writable);
            m_currentProperty->setEnabled(attribute.readable && attribute.writable);
            break;
        case Z3D::ZCameraInterface::CameraAttributeTypeInt:
            //m_currentProperty = m_intPropertyManager->addProperty(attrName);
            m_intPropertyManager->setValue(m_currentProperty, attribute.value.toInt());
            if (attribute.maximumValue != DBL_MAX)
                m_intPropertyManager->setMaximum(m_currentProperty, attribute.maximumValue);
            if (attribute.minimumValue != DBL_MIN)
                m_intPropertyManager->setMinimum(m_currentProperty, attribute.minimumValue);
            m_intPropertyManager->setReadOnly(m_currentProperty, !attribute.writable);
            m_currentProperty->setEnabled(attribute.readable && attribute.writable);
            break;
        case Z3D::ZCameraInterface::CameraAttributeTypeLong:
            //m_currentProperty = m_longPropertyManager->addProperty(attrName);
            m_longPropertyManager->setValue(m_currentProperty, attribute.value.toString());
            m_longPropertyManager->setReadOnly(m_currentProperty, !attribute.writable);
            m_currentProperty->setEnabled(attribute.readable && attribute.writable);
            break;
        case Z3D::ZCameraInterface::CameraAttributeTypeFloat:
            //m_currentProperty = m_floatPropertyManager->addProperty(attrName);
            m_floatPropertyManager->setValue(m_currentProperty, attribute.value.toDouble());
            if (attribute.maximumValue != DBL_MAX)
                m_floatPropertyManager->setMaximum(m_currentProperty, attribute.maximumValue);
            if (attribute.minimumValue != DBL_MIN)
                m_floatPropertyManager->setMinimum(m_currentProperty, attribute.minimumValue);
            m_floatPropertyManager->setReadOnly(m_currentProperty, !attribute.writable);
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
            //qWarning() << "Command attribute type not supported yet" << attribute.name;
            //m_currentProperty = m_groupManager->addProperty(attrName);
            m_enumPropertyManager->setValue(m_currentProperty, 0);
            m_enumPropertyManager->setEnumNames(m_currentProperty, QStringList() << "[COMMAND]" << "Execute command");
            m_currentProperty->setEnabled(attribute.writable);
            break;
        }

    }

    /// connect signal again
    QObject::connect(m_boolPropertyManager,   SIGNAL(propertyChanged(QtProperty*)),
                     this, SLOT(propertyChanged(QtProperty*)));
    QObject::connect(m_stringPropertyManager, SIGNAL(propertyChanged(QtProperty*)),
                     this, SLOT(propertyChanged(QtProperty*)));
    QObject::connect(m_intPropertyManager,    SIGNAL(propertyChanged(QtProperty*)),
                     this, SLOT(propertyChanged(QtProperty*)));
    QObject::connect(m_longPropertyManager, SIGNAL(propertyChanged(QtProperty*)),
                     this, SLOT(propertyChanged(QtProperty*)));
    QObject::connect(m_floatPropertyManager,  SIGNAL(propertyChanged(QtProperty*)),
                     this, SLOT(propertyChanged(QtProperty*)));
    QObject::connect(m_enumPropertyManager,   SIGNAL(propertyChanged(QtProperty*)),
                     this, SLOT(propertyChanged(QtProperty*)));
}

void ZCameraSettingsWidget::onCameraAttributeChanged(QString name, QVariant value)
{
    QtProperty *m_currentProperty;
    if (m_propertiesList.contains(name)) {
        /// already existed
        m_currentProperty = m_propertiesList[name];
    }
}

} // namespace Z3D
