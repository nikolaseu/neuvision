#include "zcameracalibratorworker.h"

#include "zcalibrationimage.h"

#include "zcameracalibrator.h"

#include <QCoreApplication>
#include <QTime>
#include <QtConcurrentMap>
#include <QtConcurrentRun>

namespace Z3D
{

struct ZCameraCalibratorWorkerParallelFindPatternImpl
{
    ZCameraCalibratorWorkerParallelFindPatternImpl(Z3D::ZCalibrationPatternFinder::Ptr patternFinder)
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

ZCameraCalibratorWorker::ZCameraCalibratorWorker(QObject *parent)
    : QObject(parent)
    , m_patternFinder(0)
    , m_cameraCalibrator(0)
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

ZCalibrationImageModel *ZCameraCalibratorWorker::imageModel()
{
    return m_imageModel;
}

ZCalibrationPatternFinder::Ptr ZCameraCalibratorWorker::patternFinder()
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

void ZCameraCalibratorWorker::setPatternFinder(ZCalibrationPatternFinder::Ptr patternFinder)
{
    if (m_patternFinder != patternFinder) {
        if (m_patternFinder) {
            /// disconnect old signals
            QObject::disconnect(m_patternFinder.data(), &ZCalibrationPatternFinder::configHashChanged,
                                this, &ZCameraCalibratorWorker::findCalibrationPattern);
        }

        /// update
        m_patternFinder = patternFinder;

        if (m_patternFinder) {
            /// connect new signals
            QObject::connect(m_patternFinder.data(), &ZCalibrationPatternFinder::configHashChanged,
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
                                  ZCameraCalibratorWorkerParallelFindPatternImpl(m_patternFinder)) );
}

void ZCameraCalibratorWorker::calibrate()
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
                QtConcurrent::run(this, &ZCameraCalibratorWorker::calibrateFunctionImpl) );
}

void ZCameraCalibratorWorker::calibrateFunctionImpl()
{
    if (m_patternFinderFutureWatcher.isRunning()) {
        qWarning() << Q_FUNC_INFO << "waiting for pattern finder to finish...";
        m_patternFinderFutureWatcher.waitForFinished();
    }

    qDebug() << Q_FUNC_INFO << "starting...";

    QTime time;
    time.start();

    /// 0%
    setProgress(0.f, tr("Starting camera calibration..."));

    int imageCount = m_imageModel->rowCount();

    /// get only the images where the calibration pattern was detected
    QVector< Z3D::ZCalibrationImage::Ptr > validImages;
    validImages.reserve(imageCount);
    for (int i=0; i<imageCount; ++i) {
        Z3D::ZCalibrationImage::Ptr image = m_imageModel->imageAt(i);
        if (image->state() == ZCalibrationImage::PatternFoundState) {
            validImages.push_back(image);
        }
    }

    int validImageCount = validImages.size();

    if (validImageCount < 2) {
        /// not enough valid images
        qWarning() << "could not calibrate camera, not enough images with calibration pattern found";

        /// calibration end
        setProgress(1.f, tr("Calibration failed. Not enough calibration patterns found"));
        emit calibrationFailed(tr("Calibration failed. Not enough calibration patterns found"));
        emit calibrationChanged(ZCameraCalibration::Ptr(0));

        return;
    }

    /// load detected calibration pattern points for each image
    std::vector<std::vector<cv::Point2f> > imagePoints;
    imagePoints.reserve(validImageCount);
    std::vector<std::vector<cv::Point3f> > realWorldPoints;
    realWorldPoints.reserve(validImageCount);
    for (int i=0; i<validImageCount; ++i) {
        Z3D::ZCalibrationImage::Ptr image = validImages[i];
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
    ZCameraCalibration::Ptr calibration = m_cameraCalibrator->getCalibration(imagePoints, realWorldPoints, imageSize);

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

