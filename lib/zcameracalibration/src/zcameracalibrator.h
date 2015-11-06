#ifndef Z3D_CAMERACALIBRATOR___ZCAMERACALIBRATOR_H
#define Z3D_CAMERACALIBRATOR___ZCAMERACALIBRATOR_H

#include "zcameracalibration.h"

#include <QObject>

namespace Z3D
{

class ZCameraCalibrator : public QObject
{
    Q_OBJECT

public:
    explicit ZCameraCalibrator(QObject *parent = 0) : QObject(parent) {}

    virtual ~ZCameraCalibrator() {}

    virtual QString name() = 0;

    virtual ZCameraCalibration::Ptr getCalibration(
            std::vector<std::vector<cv::Point2f> > &imagePoints,
            std::vector<std::vector<cv::Point3f> > &realWorldPoints,
            cv::Size &imageSize) = 0;

    virtual QWidget *configWidget() = 0;
};

} // namespace Z3D

#endif // Z3D_CAMERACALIBRATOR___ZCAMERACALIBRATOR_H
