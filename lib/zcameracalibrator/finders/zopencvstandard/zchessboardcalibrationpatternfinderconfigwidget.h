#ifndef Z3D_CAMERACALIBRATOR___ZCHESSBOARDCALIBRATIONPATTERNFINDERCONFIGWIDGET_H
#define Z3D_CAMERACALIBRATOR___ZCHESSBOARDCALIBRATIONPATTERNFINDERCONFIGWIDGET_H

#include <QWidget>

namespace Ui {
class ZChessboardCalibrationPatternFinderConfigWidget;
}

namespace Z3D
{

/// forward declaration
class ZChessboardCalibrationPatternFinder;

class ZChessboardCalibrationPatternFinderConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ZChessboardCalibrationPatternFinderConfigWidget(ZChessboardCalibrationPatternFinder *patternFinder, QWidget *parent = 0);
    ~ZChessboardCalibrationPatternFinderConfigWidget();

private slots:

private:
    Ui::ZChessboardCalibrationPatternFinderConfigWidget *ui;

    ZChessboardCalibrationPatternFinder *m_patternFinder;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZCHESSBOARDCALIBRATIONPATTERNFINDERCONFIGWIDGET_H
