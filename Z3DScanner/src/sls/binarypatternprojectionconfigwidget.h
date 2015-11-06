#ifndef BINARYPATTERNPROJECTIONCONFIGWIDGET_H
#define BINARYPATTERNPROJECTIONCONFIGWIDGET_H

#include <QWidget>

class BinaryPatternProjection;

namespace Ui {
class BinaryPatternProjectionConfigWidget;
}

class BinaryPatternProjectionConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit BinaryPatternProjectionConfigWidget(BinaryPatternProjection *binaryPatternProjection, QWidget *parent = 0);
    ~BinaryPatternProjectionConfigWidget();

private slots:
    void onSelectedScreenChanged(int index);
    void updateScreens();

private:
    Ui::BinaryPatternProjectionConfigWidget *ui;

    BinaryPatternProjection *m_binaryPatternProjection;
};

#endif // BINARYPATTERNPROJECTIONCONFIGWIDGET_H
