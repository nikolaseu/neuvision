#pragma once

#include <QWidget>

namespace Z3D {

class ZDualCameraStereoSLS;

namespace Ui {
class ZDualCameraStereoSLSConfigWidget;
}

class ZDualCameraStereoSLSConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ZDualCameraStereoSLSConfigWidget(ZDualCameraStereoSLS *stereoSLS, QWidget *parent = 0);
    ~ZDualCameraStereoSLSConfigWidget();

private:
    Ui::ZDualCameraStereoSLSConfigWidget *ui;

    ZDualCameraStereoSLS *m_stereoSLS;
};

} // namespace Z3D
