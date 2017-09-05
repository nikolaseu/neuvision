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

ZMultiCameraCalibratorWorker::ZMultiCameraCalibratorWorker(QObject *parent)
    : QObject(parent)
    , m_patternFinder(nullptr)
    , m_cameraCalibrator(nullptr)
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
                                  [=](const Z3D::ZCalibrationImage::Ptr &image) {
                                      image->findPattern(m_patternFinder.data());
                                  }));
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

    const auto imageCount = size_t(m_imageModel->rowCount());
    auto cameraCount = size_t(0);

    /// get only the images where the calibration pattern was detected
    std::vector< Z3D::ZMultiCalibrationImage::Ptr > validImages;
    validImages.reserve(imageCount);
    for (size_t i=0; i<imageCount; ++i) {
        Z3D::ZMultiCalibrationImage::Ptr multiImage = m_imageModel->imageAt(int(i));
        if (multiImage->state() == ZCalibrationImage::PatternFoundState) {
            validImages.push_back(multiImage);

            if (cameraCount == 0)
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

    const auto validImageCount = validImages.size();

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

    /// 10% ¿?
    setProgress(0.1f, tr("Preparing calibration images..."));

    /// load detected calibration pattern points for each image
    std::vector<std::vector<std::vector<cv::Point2f> > > allImagePoints(cameraCount);
    std::vector<std::vector<cv::Point3f> > realWorldPoints(validImageCount);

    for (size_t i=0; i<cameraCount; ++i) {
        auto &imagePoints = allImagePoints[i];
        imagePoints.resize(validImageCount);
    }

    /// filter to only add points that were found in all cameras (required for incomplete patterns)
    for (size_t imageIndex=0; imageIndex<validImageCount; ++imageIndex) {
        const auto &multiImage = validImages[imageIndex];
        const auto &imgCam1 = multiImage->image(0);
        const auto &worldPointsCam1 = imgCam1->detectedPointsInRealCoordinates();
        const auto &imgPointsCam1 = imgCam1->detectedPoints();
        auto &worldPoints = realWorldPoints[imageIndex];
        worldPoints.reserve(worldPointsCam1.size());
        auto itImgPointsCam1 = imgPointsCam1.begin();
        for (auto itWorldPointsCam1 = worldPointsCam1.begin();
             itWorldPointsCam1 != worldPointsCam1.end();
             ++itWorldPointsCam1, ++itImgPointsCam1)
        {
            std::vector<cv::Point2f> imagePoints(cameraCount);
            imagePoints[0] = *itImgPointsCam1;
            bool foundInAll = true;
            for (size_t cameraIndex=1; cameraIndex<cameraCount; ++cameraIndex) {
                const auto &img = multiImage->image(cameraIndex);
                const auto &worldPointsCurrentImg = img->detectedPointsInRealCoordinates();
                const auto itWorldPointsCurrentImg = std::find(worldPointsCurrentImg.begin(), worldPointsCurrentImg.end(), *itWorldPointsCam1);
                if (itWorldPointsCurrentImg == worldPointsCurrentImg.end()) {
                    foundInAll = false;
                    break;
                }
                const auto indexWorldPointInCurrentImg = itWorldPointsCurrentImg - worldPointsCurrentImg.begin();
                const auto &detectedPoints = img->detectedPoints();
                imagePoints[cameraIndex] = detectedPoints[indexWorldPointInCurrentImg];
            }
            if (foundInAll) {
                worldPoints.push_back(*itWorldPointsCam1);
                for (size_t cameraIndex=0; cameraIndex<cameraCount; ++cameraIndex) {
                    allImagePoints[cameraIndex][imageIndex].push_back(imagePoints[cameraIndex]);
                }
            }
        }
    }

    /// 30% ¿?
    setProgress(0.3f, tr("Running calibration algorithm..."));

    newCalibrations = m_cameraCalibrator->getCalibration(currentCalibrations, allImagePoints, realWorldPoints);

    /// 99%
    const auto finishMessage = tr("Calibration finished in %1 msecs").arg(time.elapsed());
    setProgress(0.99f, finishMessage);

    /// this function is run from a thread in the QThreadPool, we need to move
    /// the qobject to a thread that has an event loop running
    for (size_t i=0; i<newCalibrations.size(); ++i) {
        ZCameraCalibration::Ptr calibration = newCalibrations[i];
        if (calibration) {
            while (!calibration->ready()) {
                QThread::msleep(100);
            }
            calibration->moveToThread(QCoreApplication::instance()->thread());
        }
    }

    /// calibration end
    emit calibrationChanged(newCalibrations);

    /// 100%
    setProgress(1.f, finishMessage);
}

} // namespace Z3D

