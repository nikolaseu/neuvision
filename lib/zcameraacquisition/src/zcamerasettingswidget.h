#pragma once

#include "zcameraacquisition_global.h"
#include "zcamerainterface.h"

#include <QtProperty>
#include <QtStringPropertyManager>

#include <QWidget>

namespace Z3D
{

namespace Ui {
class ZCameraSettingsWidget;
}

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZCameraSettingsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ZCameraSettingsWidget(ZCameraInterface::WeakPtr camera, QWidget *parent = 0);
    ~ZCameraSettingsWidget();

protected slots:
    void propertyChanged(QtProperty *property);
    void updateProperties();

    void onCameraAttributeChanged(QString name, QVariant value);

private:
    Ui::ZCameraSettingsWidget *ui;

    ZCameraInterface::WeakPtr m_camera;

    QMap<QString, QtProperty*> m_propertiesList;

    QtAbstractPropertyBrowser *m_propertyBrowser;
    QtGroupPropertyManager *m_groupManager;
    QtBoolPropertyManager *m_boolPropertyManager;
    QtStringPropertyManager *m_stringPropertyManager;
    QtIntPropertyManager *m_intPropertyManager;
    QtStringPropertyManager *m_longPropertyManager;
    QtDoublePropertyManager *m_floatPropertyManager;
    QtEnumPropertyManager *m_enumPropertyManager;
    QtBoolPropertyManager *m_commandPropertyManager;
};

} // namespace Z3D
