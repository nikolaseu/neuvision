#ifndef BINARYPATTERNPROJECTIONCONFIGWIDGET_H
#define BINARYPATTERNPROJECTIONCONFIGWIDGET_H

#include <QWidget>

namespace Ui {
class ZBinaryPatternProjectionConfigWidget;
}

namespace Z3D
{

class ZBinaryPatternProjection;

class ZBinaryPatternProjectionConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ZBinaryPatternProjectionConfigWidget(ZBinaryPatternProjection *binaryPatternProjection, QWidget *parent = 0);
    ~ZBinaryPatternProjectionConfigWidget();

private slots:
    void onSelectedScreenChanged(int index);
    void updateScreens();

private:
    Ui::ZBinaryPatternProjectionConfigWidget *ui;

    ZBinaryPatternProjection *m_binaryPatternProjection;
};

} // namespace Z3D

#endif // BINARYPATTERNPROJECTIONCONFIGWIDGET_H
