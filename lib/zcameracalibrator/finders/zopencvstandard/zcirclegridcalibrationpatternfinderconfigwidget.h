#ifndef Z3D_CAMERACALIBRATOR___ZCIRCLEGRIDCALIBRATIONPATTERNFINDERCONFIGWIDGET_H
#define Z3D_CAMERACALIBRATOR___ZCIRCLEGRIDCALIBRATIONPATTERNFINDERCONFIGWIDGET_H

#include <QWidget>

namespace Ui {
class ZCircleGridCalibrationPatternFinderConfigWidget;
}

namespace Z3D
{

/// forward declaration
class ZCircleGridCalibrationPatternFinder;

class ZCircleGridCalibrationPatternFinderConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ZCircleGridCalibrationPatternFinderConfigWidget(ZCircleGridCalibrationPatternFinder *patternFinder, QWidget *parent = 0);
    ~ZCircleGridCalibrationPatternFinderConfigWidget();

private:
    Ui::ZCircleGridCalibrationPatternFinderConfigWidget *ui;

    ZCircleGridCalibrationPatternFinder *m_patternFinder;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZCIRCLEGRIDCALIBRATIONPATTERNFINDERCONFIGWIDGET_H
