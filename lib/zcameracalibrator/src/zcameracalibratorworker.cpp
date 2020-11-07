//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "ZCameraCalibrator/zcameracalibratorworker.h"

#include "ZCameraCalibrator/zcalibrationimage.h"
#include "ZCameraCalibrator/zcalibrationimagemodel.h"
#include "ZCameraCalibrator/zcalibrationpatternfinder.h"

#include "ZCameraCalibration/zcameracalibration.h"
#include "ZCameraCalibration/zcameracalibrator.h"
#include "ZCore/zlogging.h"

#include <QCoreApplication>
#include <QElapsedTimer>
#include <QtConcurrentMap>
#include <QtConcurrentRun>

Z3D_LOGGING_CATEGORY_FROM_FILE("z3d.zcameracalibrator", QtInfoMsg)

namespace Z3D
{

ZCameraCalibratorWorker::ZCameraCalibratorWorker(QObject *parent)
    : QObject(parent)
    , m_patternFinder(nullptr)
    , m_cameraCalibrator(nullptr)
    , m_progress(1.f)
{
    /// connect future watcher signals to monitor progress
    QObject::connect(&m_patternFinderFutureWatcher, &QFutureWatcher<void>::progressRangeChanged,
                     this, &ZCameraCalibratorWorker::setProgressRange);
    QObject::connect(&m_patternFinderFutureWatcher, &QFutureWatcher<void>::progressValueChanged,
                     this, &ZCameraCalibratorWorker::setProgressValue);

    /// start finding calibration patterns when the pattern finder changes
    QObject::connect(this, &ZCameraCalibratorWorker::patternFinderChanged,
                     this, &ZCameraCalibratorWorker::findCalibrationPattern);
}

ZCameraCalibratorWorker::~ZCameraCalibratorWorker()
{
    if (m_calibrateFutureWatcher.isRunning()) {
        zWarning() << Q_FUNC_INFO << "canceling running calibrations task...";
        m_calibrateFutureWatcher.future().cancel();
    }

    if (m_patternFinderFutureWatcher.isRunning()) {
        zWarning() << Q_FUNC_INFO << "canceling running pattern finder tasks...";
        m_patternFinderFutureWatcher.future().cancel();
    }

    if (m_calibrateFutureWatcher.isRunning()) {
        m_calibrateFutureWatcher.waitForFinished();
    }

    if (m_patternFinderFutureWatcher.isRunning()) {
        m_patternFinderFutureWatcher.waitForFinished();
    }
}

ZCalibrationImageModel *ZCameraCalibratorWorker::imageModel()
{
    return m_imageModel;
}

ZCalibrationPatternFinderPtr ZCameraCalibratorWorker::patternFinder()
{
    return m_patternFinder;
}

ZCameraCalibrator *ZCameraCalibratorWorker::cameraCalibrator()
{
    return m_cameraCalibrator;
}

float ZCameraCalibratorWorker::progress() const
{
    return m_progress;
}

void ZCameraCalibratorWorker::setImageModel(ZCalibrationImageModel *imageModel)
{
    if (m_imageModel != imageModel) {

        if (m_imageModel) {
            /// disconnect old signals
            QObject::disconnect(m_imageModel.data(), &ZCalibrationImageModel::newImagesAdded,
                                this, &ZCameraCalibratorWorker::findCalibrationPattern);
        }

        /// update
        m_imageModel = imageModel;

        if (m_imageModel) {
            /// connect new signals
            QObject::connect(m_imageModel.data(), &ZCalibrationImageModel::newImagesAdded,
                                this, &ZCameraCalibratorWorker::findCalibrationPattern);
        }

        /// notify
        emit imageModelChanged();
    }
}

void ZCameraCalibratorWorker::setPatternFinder(ZCalibrationPatternFinderPtr patternFinder)
{
    if (m_patternFinder != patternFinder) {
        if (m_patternFinder) {
            /// disconnect old signals
            QObject::disconnect(m_patternFinder.get(), &ZCalibrationPatternFinder::configHashChanged,
                                this, &ZCameraCalibratorWorker::findCalibrationPattern);
        }

        /// update
        m_patternFinder = patternFinder;

        if (m_patternFinder) {
            /// connect new signals
            QObject::connect(m_patternFinder.get(), &ZCalibrationPatternFinder::configHashChanged,
                             this, &ZCameraCalibratorWorker::findCalibrationPattern);
        }

        /// notify
        emit patternFinderChanged();
    }
}

void ZCameraCalibratorWorker::setCameraCalibrator(ZCameraCalibrator *cameraCalibrator)
{
    if (m_cameraCalibrator != cameraCalibrator) {
        m_cameraCalibrator = cameraCalibrator;
    }
}

void ZCameraCalibratorWorker::setProgress(float progress, QString message)
{
    if (m_progress != progress) {
        m_progress = progress;
        emit progressChanged(progress, message);
    }
}

void ZCameraCalibratorWorker::setProgressValue(int arg)
{
    setProgress( (float) arg / m_maxProgressValue );
}

void ZCameraCalibratorWorker::setProgressRange(int min, int max)
{
    m_minProgressValue = min;
    m_maxProgressValue = max;
}

void ZCameraCalibratorWorker::findCalibrationPattern()
{
    if (!m_imageModel || !m_patternFinder || !m_imageModel->rowCount())
        return;

    if (m_patternFinderFutureWatcher.isRunning()) {
        zWarning() << Q_FUNC_INFO << "canceling previous pattern finder...";

        /// 0%
        setProgress(-1.f, tr("Canceling previous calibration patterns search..."));

        m_patternFinderFutureWatcher.future().cancel();
        m_patternFinderFutureWatcher.waitForFinished();
    }

    zDebug() << Q_FUNC_INFO << "using" << QThreadPool::globalInstance()->maxThreadCount() << "thread(s)...";

    /// 0%
    setProgress(0.f, tr("Finding calibration patterns..."));

    /// execution on parallel in other threads (from the thread pool)
    m_patternFinderFutureWatcher.setFuture(
                QtConcurrent::map(m_imageModel->images(),
                                  [=](const auto &image) {
                                        image->findPattern(m_patternFinder.get());
                                  }));
}

void ZCameraCalibratorWorker::calibrate()
{
    if (!m_imageModel)
        return;

    /// if for some reason the task is executed simultaneously, we need to wait
    /// to set the future to be able to exit cleanly
    if (m_calibrateFutureWatcher.isRunning()) {
        zWarning() << Q_FUNC_INFO << "canceling previous calibration...";

        /// 0%
        setProgress(-1.f, tr("Canceling previous camera calibration..."));

        m_calibrateFutureWatcher.future().cancel();
        m_calibrateFutureWatcher.waitForFinished();
    }

    /// starts execution in other thread
    m_calibrateFutureWatcher.setFuture(
            QtConcurrent::run(std::bind(&ZCameraCalibratorWorker::calibrateFunctionImpl, this)));
}

void ZCameraCalibratorWorker::calibrateFunctionImpl()
{
    if (m_patternFinderFutureWatcher.isRunning()) {
        zWarning() << Q_FUNC_INFO << "waiting for pattern finder to finish...";
        m_patternFinderFutureWatcher.waitForFinished();
    }

    zDebug() << Q_FUNC_INFO << "starting...";

    QElapsedTimer time;
    time.start();

    /// 0%
    setProgress(0.f, tr("Starting camera calibration..."));

    int imageCount = m_imageModel->rowCount();

    /// get only the images where the calibration pattern was detected
    QVector< Z3D::ZCalibrationImagePtr > validImages;
    validImages.reserve(imageCount);
    for (int i=0; i<imageCount; ++i) {
        auto image = m_imageModel->imageAt(i);
        if (image->state() == ZCalibrationImage::PatternFoundState) {
            validImages.push_back(image);
        }
    }

    int validImageCount = validImages.size();

    if (validImageCount < 2) {
        /// not enough valid images
        zWarning() << "could not calibrate camera, not enough images with calibration pattern found";

        /// calibration end
        setProgress(1.f, tr("Calibration failed. Not enough calibration patterns found"));
        emit calibrationFailed(tr("Calibration failed. Not enough calibration patterns found"));
        emit calibrationChanged(nullptr);

        return;
    }

    /// load detected calibration pattern points for each image
    std::vector<std::vector<cv::Point2f> > imagePoints;
    imagePoints.reserve(validImageCount);
    std::vector<std::vector<cv::Point3f> > realWorldPoints;
    realWorldPoints.reserve(validImageCount);
    for (int i=0; i<validImageCount; ++i) {
        auto image = validImages[i];
        /// detected calibration pattern points in image coordinates
        imagePoints.push_back( image->detectedPoints() );
        /// detected calibration pattern point in real world coordinates
        realWorldPoints.push_back( image->detectedPointsInRealCoordinates() );
    }

    /// get image size
    cv::Size imageSize(m_imageModel->imageWidth(), m_imageModel->imageHeight());

    /// 40% ¿?
    setProgress(0.4f, tr("Running camera calibration..."));

    /// get calibration
    ZCameraCalibrationPtr calibration = m_cameraCalibrator->getCalibration(imagePoints, realWorldPoints, imageSize);

    /// this function is run from a thread in the QThreadPool, we need to move
    /// the qobject to a thread that has an event loop running
    if (calibration)
        calibration->moveToThread(QCoreApplication::instance()->thread());

    /// 90%
    setProgress(0.9f, tr("Finishing camera calibration..."));

    /// calibration end
    emit calibrationChanged(calibration);

    /// 100%
    setProgress(1.f, tr("Calibration finished in %1 msecs").arg(time.elapsed()));
}

} // namespace Z3D

