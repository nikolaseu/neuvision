#pragma once

#include "zcameracalibration.h"

#include <QObject>

namespace Z3D
{

class ZMultiCameraCalibrator : public QObject
{
    Q_OBJECT

public:
    explicit ZMultiCameraCalibrator(QObject *parent = 0) : QObject(parent) {}

    virtual ~ZMultiCameraCalibrator() {}

    virtual QString name() = 0;

    virtual std::vector<ZCameraCalibration::Ptr> getCalibration(
            std::vector< Z3D::ZCameraCalibration::Ptr > &initialCameraCalibrations,
            std::vector< std::vector< std::vector< cv::Point2f > > > &imagePoints,
            std::vector< std::vector< std::vector< cv::Point3f > > > &objectPoints) = 0;

    virtual QWidget *configWidget() = 0;
};

} // namespace Z3D
