#pragma once

#include "zcameracalibrator_global.h"
#include "zcalibrationimage.h"

#include "zimageviewer.h"

namespace Z3D
{

class Z3D_CAMERACALIBRATOR_SHARED_EXPORT ZCalibrationImageViewer : public ZImageViewer
{
    Q_OBJECT

public:
    enum DisplayMode {
        NoMarkers = 0,
        ShowMarkers
    };

    ZCalibrationImageViewer(QWidget *parent = 0);
    ~ZCalibrationImageViewer();

    void setDisplayMode(DisplayMode displayMode);

public slots:
    void updateCalibrationImage(Z3D::ZCalibrationImage::Ptr image);

protected:
    DisplayMode m_displayMode;

    QPolygonF m_markerPolygon;

    std::vector<QGraphicsItem*> m_calibrationPoints;
};

} // namespace Z3D
