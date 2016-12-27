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

#include "zmulticameracalibratorworker.h"

#include "zcalibrationimage.h"
#include "zmulticalibrationimage.h"
#include "zmulticameracalibrator.h"

#include <QCoreApplication>
#include <QTime>
#include <QtConcurrentMap>
#include <QtConcurrentRun>

namespace Z3D
{

struct ZMultiCameraCalibratorWorkerParallelFindPatternImpl
{
    explicit ZMultiCameraCalibratorWorkerParallelFindPatternImpl(Z3D::ZCalibrationPatternFinder::Ptr patternFinder)
        : m_patternFinder(patternFinder.data())
    {
        // empty
    }

    void operator()(const Z3D::ZCalibrationImage::Ptr &image)
    {
        image->findPattern(m_patternFinder);
    }

    Z3D::ZCalibrationPatternFinder::WeakPtr m_patternFinder;
};

ZMultiCameraCalibratorWorker::ZMultiCameraCalibratorWorker(QObject *parent)
    : QObject(parent)
    , m_patternFinder(0)
    , m_cameraCalibrator(0)
    , m_progress(1.f)
{
    /// connect future watcher signals to monitor progress
    QObject::connect(&m_patternFinderFutureWatcher, &QFutureWatcher<void>::progressRangeChanged,
                     this, &ZMultiCameraCalibratorWorker::setProgressRange);
    QObject::connect(&m_patternFinderFutureWatcher, &QFutureWatcher<void>::progressValueChanged,
                     this, &ZMultiCameraCalibratorWorker::setProgressValue);

    /// start finding calibration patterns when the pattern finder changes
    QObject::connect(this, &ZMultiCameraCalibratorWorker::patternFinderChanged,
                     this, &ZMultiCameraCalibratorWorker::findCalibrationPattern);
}

ZMultiCameraCalibratorWorker::~ZMultiCameraCalibratorWorker()
{
    if (m_calibrateFutureWatcher.isRunning()) {
        qWarning() << Q_FUNC_INFO << "canceling running calibrations task...";
        m_calibrateFutureWatcher.future().cancel();
    }

    if (m_patternFinderFutureWatcher.isRunning()) {
        qWarning() << Q_FUNC_INFO << "canceling running pattern finder tasks...";
        m_patternFinderFutureWatcher.future().cancel();
    }

    if (m_calibrateFutureWatcher.isRunning()) {
        m_calibrateFutureWatcher.waitForFinished();
    }

    if (m_patternFinderFutureWatcher.isRunning()) {
        m_patternFinderFutureWatcher.waitForFinished();
    }
}

ZMultiCalibrationImageModel *ZMultiCameraCalibratorWorker::imageModel()
{
    return m_imageModel;
}

ZCalibrationPatternFinder::Ptr ZMultiCameraCalibratorWorker::patternFinder()
{
    return m_patternFinder;
}

ZMultiCameraCalibrator *ZMultiCameraCalibratorWorker::cameraCalibrator()
{
    return m_cameraCalibrator;
}

float ZMultiCameraCalibratorWorker::progress() const
{
    return m_progress;
}

void ZMultiCameraCalibratorWorker::setImageModel(ZMultiCalibrationImageModel *imageModel)
{
    if (m_imageModel != imageModel) {

        if (m_imageModel) {
            /// disconnect old signals
            QObject::disconnect(m_imageModel.data(), &ZMultiCalibrationImageModel::newImagesAdded,
                                this, &ZMultiCameraCalibratorWorker::findCalibrationPattern);
        }

        /// update
        m_imageModel = imageModel;

        if (m_imageModel) {
            /// connect new signals
            QObject::connect(m_imageModel.data(), &ZMultiCalibrationImageModel::newImagesAdded,
                                this, &ZMultiCameraCalibratorWorker::findCalibrationPattern);
        }

        /// notify
        emit imageModelChanged();
    }
}

void ZMultiCameraCalibratorWorker::setPatternFinder(ZCalibrationPatternFinder::Ptr patternFinder)
{
    if (m_patternFinder != patternFinder) {
        if (m_patternFinder) {
            /// disconnect old signals
            QObject::disconnect(m_patternFinder.data(), &ZCalibrationPatternFinder::configHashChanged,
                                this, &ZMultiCameraCalibratorWorker::findCalibrationPattern);
        }

        /// update
        m_patternFinder = patternFinder;

        if (m_patternFinder) {
            /// connect new signals
            QObject::connect(m_patternFinder.data(), &ZCalibrationPatternFinder::configHashChanged,
                                this, &ZMultiCameraCalibratorWorker::findCalibrationPattern);
        }

        /// notify
        emit patternFinderChanged();
    }
}

void ZMultiCameraCalibratorWorker::setCameraCalibrator(ZMultiCameraCalibrator *cameraCalibrator)
{
    if (m_cameraCalibrator != cameraCalibrator) {
        m_cameraCalibrator = cameraCalibrator;
    }
}

void ZMultiCameraCalibratorWorker::setProgress(float progress, QString message)
{
    if (m_progress != progress) {
        m_progress = progress;
        emit progressChanged(progress, message);
    }
}

void ZMultiCameraCalibratorWorker::setProgressValue(int arg)
{
    setProgress( (float) arg / m_maxProgressValue );
}

void ZMultiCameraCalibratorWorker::setProgressRange(int min, int max)
{
    m_minProgressValue = min;
    m_maxProgressValue = max;
}

void ZMultiCameraCalibratorWorker::findCalibrationPattern()
{
    if (!m_imageModel || !m_patternFinder || !m_imageModel->rowCount())
        return;

    if (m_patternFinderFutureWatcher.isRunning()) {
        qWarning() << Q_FUNC_INFO << "canceling previous pattern finder...";

        /// 0%
        setProgress(-1.f, tr("Canceling previous calibration patterns search..."));

        m_patternFinderFutureWatcher.future().cancel();
        m_patternFinderFutureWatcher.waitForFinished();
    }

    qDebug() << Q_FUNC_INFO << "using" << QThreadPool::globalInstance()->maxThreadCount() << "thread(s)...";

    /// 0%
    setProgress(0.f, tr("Finding calibration patterns..."));

    /// execution on parallel in other threads (from the thread pool)
    m_patternFinderFutureWatcher.setFuture(
                QtConcurrent::map(m_imageModel->images(),
                                  ZMultiCameraCalibratorWorkerParallelFindPatternImpl(m_patternFinder)) );
}

void ZMultiCameraCalibratorWorker::calibrate(std::vector<ZCameraCalibration::Ptr> currentCalibrations)
{
    if (!m_imageModel)
        return;

    /// if for some reason the task is executed simultaneously, we need to wait
    /// to set the future to be able to exit cleanly
    if (m_calibrateFutureWatcher.isRunning()) {
        qWarning() << Q_FUNC_INFO << "canceling previous calibration...";

        /// 0%
        setProgress(-1.f, tr("Canceling previous camera calibration..."));

        m_calibrateFutureWatcher.future().cancel();
        m_calibrateFutureWatcher.waitForFinished();
    }

    /// starts execution in other thread
    m_calibrateFutureWatcher.setFuture(
                QtConcurrent::run(this, &ZMultiCameraCalibratorWorker::calibrateFunctionImpl, currentCalibrations) );
}

void ZMultiCameraCalibratorWorker::calibrateFunctionImpl(std::vector<ZCameraCalibration::Ptr> currentCalibrations)
{
    if (m_patternFinderFutureWatcher.isRunning()) {
        qWarning() << Q_FUNC_INFO << "waiting for pattern finder to finish...";
        m_patternFinderFutureWatcher.waitForFinished();
    }

    qDebug() << Q_FUNC_INFO << "starting...";

    QTime time;
    time.start();

    /// 0%
    setProgress(0.f, tr("Starting multi camera calibration..."));

    //std::vector<ZCameraCalibration::Ptr> currentCalibrations;
    std::vector<ZCameraCalibration::Ptr> newCalibrations;

    int imageCount = m_imageModel->rowCount();
    int cameraCount = -1;

    /// get only the images where the calibration pattern was detected
    QVector< Z3D::ZMultiCalibrationImage::Ptr > validImages;
    validImages.reserve(imageCount);
    for (int i=0; i<imageCount; ++i) {
        Z3D::ZMultiCalibrationImage::Ptr multiImage = m_imageModel->imageAt(i);
        if (multiImage->state() == ZCalibrationImage::PatternFoundState) {
            validImages.push_back(multiImage);

            if (cameraCount < 0)
                cameraCount = multiImage->images().size();
            else if (cameraCount != multiImage->images().size()) {
                qWarning() << "The number of cameras is not the same for all images";

                /// calibration end
                setProgress(1.f);
                emit calibrationFailed(tr("Calibration failed. The number of cameras is not the same for all images"));
                emit calibrationChanged(newCalibrations);

                return;
            }
        }
    }

    if (cameraCount < 2) {
        /// not enough cameras, this is for multi camera setups!
        qWarning() << "could not calibrate multi camera, not enough cameras";

        /// calibration end
        setProgress(1.f);
        emit calibrationFailed(tr("Calibration failed. Not enought cameras to calibrate"));
        emit calibrationChanged(newCalibrations);

        return;
    } else {
        qDebug() << "number of cameras:" << cameraCount;
    }

    int validImageCount = validImages.size();

    if (validImageCount < 1) {
        /// not enough valid images
        qWarning() << "could not calibrate camera, not enough images with calibration pattern found";

        /// calibration end
        setProgress(1.f);
        emit calibrationFailed(tr("Calibration failed. Not enough calibration patterns found"));
        emit calibrationChanged(newCalibrations);

        return;
    } else {
        qDebug() << "number of images:" << validImageCount;
    }



    /// load detected calibration pattern points for each image
    std::vector<std::vector<std::vector<cv::Point2f> > > allImagePoints(cameraCount);
    std::vector<std::vector<std::vector<cv::Point3f> > > allRealWorldPoints(cameraCount);

    for (int i=0; i<cameraCount; ++i) {
        auto &imagePoints = allImagePoints[i];
        imagePoints.resize(validImageCount);
        auto &realWorldPoints = allRealWorldPoints[i];
        realWorldPoints.resize(validImageCount);
    }


    for (int v=0; v<validImageCount; ++v) {
        Z3D::ZMultiCalibrationImage::Ptr multiImage = validImages[v];
        for (int i=0; i<cameraCount; ++i) {
            Z3D::ZCalibrationImage::Ptr img = multiImage->image(i);

            /// detected calibration pattern points in image coordinates
            allImagePoints[i][v] = img->detectedPoints();

            /// detected calibration pattern point in real world coordinates
            allRealWorldPoints[i][v] = img->detectedPointsInRealCoordinates();
        }
    }

    /// 40% ¿?
    setProgress(0.4f, tr("Running camera calibration..."));

    newCalibrations = m_cameraCalibrator->getCalibration(currentCalibrations, allImagePoints, allRealWorldPoints);

    /// this function is run from a thread in the QThreadPool, we need to move
    /// the qobject to a thread that has an event loop running
    for (size_t i=0; i<newCalibrations.size(); ++i) {
        ZCameraCalibration::Ptr calibration = newCalibrations[i];
        if (calibration) {
            while (!calibration->ready()) {
                qDebug() << "waiting for calibration to be ready, index:" << i;
                QThread::msleep(100);
            }
            calibration->moveToThread(QCoreApplication::instance()->thread());
        }
    }

    /// 90%
    setProgress(0.9f, tr("Finishing camera calibration..."));

    /// calibration end
    emit calibrationChanged(newCalibrations);

    /// 100%
    setProgress(1.f, tr("Calibration finished in %1 msecs").arg(time.elapsed()));
}

} // namespace Z3D

