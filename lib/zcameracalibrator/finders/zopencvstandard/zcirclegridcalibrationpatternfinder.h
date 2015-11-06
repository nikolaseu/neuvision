#ifndef Z3D_CAMERACALIBRATOR___ZCIRCLEGRIDCALIBRATIONPATTERNFINDER_H
#define Z3D_CAMERACALIBRATOR___ZCIRCLEGRIDCALIBRATIONPATTERNFINDER_H

#include "zcalibrationpatternfinder.h"

namespace Z3D
{

class ZCircleGridCalibrationPatternFinder : public ZCalibrationPatternFinder
{
    Q_OBJECT

public:
    explicit ZCircleGridCalibrationPatternFinder(QObject *parent = 0);

    virtual QString name();

    virtual QWidget *configWidget();

signals:

public slots:
    virtual bool findCalibrationPattern(cv::Mat image, std::vector<cv::Point2f> &corners, std::vector<cv::Point3f> &objectPoints);

protected slots:
    virtual void updateConfigHash();

protected:
    QWidget *m_configWidget;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZCIRCLEGRIDCALIBRATIONPATTERNFINDER_H
