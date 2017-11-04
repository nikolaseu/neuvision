/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "zcameracalibrator_fwd.h"
#include "zcameracalibrator_global.h"

#include <Z3DCameraCalibration>

#include <QFutureWatcher>
#include <QObject>

namespace Z3D
{

class Z3D_CAMERACALIBRATOR_SHARED_EXPORT ZMultiCameraCalibratorWorker : public QObject
{
    Q_OBJECT

public:
    explicit ZMultiCameraCalibratorWorker(QObject *parent = nullptr);
    ~ZMultiCameraCalibratorWorker();

    Z3D::ZMultiCalibrationImageModel* imageModel();
    ZCalibrationPatternFinderPtr patternFinder();
    Z3D::ZMultiCameraCalibrator *cameraCalibrator();

    float progress() const;

signals:
    void imageModelChanged();
    void patternFinderChanged();

    void progressChanged(float progress, QString message = QString());

    void calibrationFailed(QString message);
    void calibrationChanged(Z3D::ZMultiCameraCalibrationPtr multiCameraCalibrationResult);

public slots:
    void setImageModel(Z3D::ZMultiCalibrationImageModel* imageModel);
    void setPatternFinder(Z3D::ZCalibrationPatternFinderPtr patternFinder);
    void setCameraCalibrator(Z3D::ZMultiCameraCalibrator *cameraCalibrator);

    void setProgress(float progress, QString message = QString());
    void setProgressValue(int arg);
    void setProgressRange(int min, int max);

    void findCalibrationPattern();
    void calibrate(std::vector<Z3D::ZCameraCalibrationPtr> currentCalibrations);

protected:
    void calibrateFunctionImpl(std::vector<ZCameraCalibrationPtr> currentCalibrations);

    Z3D::ZMultiCalibrationImageModelWeakPtr m_imageModel;
    Z3D::ZCalibrationPatternFinderPtr m_patternFinder;
    Z3D::ZMultiCameraCalibrator *m_cameraCalibrator;

    QFutureWatcher<void> m_patternFinderFutureWatcher;
    QFutureWatcher<void> m_calibrateFutureWatcher;

    float m_progress;
    int m_minProgressValue;
    int m_maxProgressValue;
};

} // namespace Z3D
