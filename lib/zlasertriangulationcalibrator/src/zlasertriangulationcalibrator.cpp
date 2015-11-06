#include "zlasertriangulationcalibrator.h"

#include <Z3DPointCloud>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// we need this in linux, we use other version of pcl and vtk
//#include "vtkCamera.h"
//#include "vtkLegendScaleActor.h"
#include "vtkRenderWindow.h"

#include <QTime>
#include <QtConcurrentMap>
#include <QtConcurrentRun>

namespace Z3D
{

cv::Point3f linePlaneIntersection(const cv::Point3f &linePoint,
                                  const cv::Vec3f   &lineDirection,
                                  const cv::Point3f &planePoint,
                                  const cv::Vec3f   &planeNormal)
{
    // in our case the line will never be paralell to the plane, so there shouldn't be any problem
    // in case we want to check, dot(lineDirection,planeNormal) will be ~= zero
    cv::Vec3f pointDiff(planePoint - linePoint);
    float alpha = pointDiff.dot(planeNormal) / lineDirection.dot(planeNormal);
    return linePoint + cv::Point3f(alpha * lineDirection);
}

struct ZLaserTriangulationCalibratorWorkerParallelFindPatternImpl
{
    ZLaserTriangulationCalibratorWorkerParallelFindPatternImpl(Z3D::ZCalibrationPatternFinder::Ptr patternFinder)
        : m_patternFinder(patternFinder.data()) { }

    void operator()(const Z3D::ZCalibrationImage::Ptr &image)
    {
        image->findPattern(m_patternFinder);
    }

    Z3D::ZCalibrationPatternFinder::WeakPtr m_patternFinder;
};

struct ZLaserTriangulationCalibratorWorkerParallelPatternPositionEstimationImpl
{
    ZLaserTriangulationCalibratorWorkerParallelPatternPositionEstimationImpl(Z3D::ZCameraCalibration::WeakPtr cameraCalibration)
        : m_cameraCalibration(cameraCalibration) { }

    void operator()(const Z3D::ZCalibrationImage::Ptr &image)
    {
        image->estimateCalibrationPatternPosition(m_cameraCalibration);
    }

    Z3D::ZCameraCalibration::WeakPtr m_cameraCalibration;
};

ZLaserTriangulationCalibrator::ZLaserTriangulationCalibrator(QObject *parent) :
    QObject(parent),
    m_pclViewer(new pcl::visualization::PCLVisualizer("3D", false)),
    m_vtkWidget(new QVTKWidget())
{
    /// connect future watcher signals to monitor progress
    QObject::connect(&m_patternFinderFutureWatcher, SIGNAL(progressRangeChanged(int,int)),
                     this, SLOT(setProgressRange(int,int)));
    QObject::connect(&m_patternFinderFutureWatcher, SIGNAL(progressValueChanged(int)),
                     this, SLOT(setProgressValue(int)));

    /// start finding calibration patterns when the pattern finder changes
    QObject::connect(this, SIGNAL(patternFinderChanged(Z3D::ZCalibrationPatternFinder*)),
                     this, SLOT(findCalibrationPattern()));

    /// init qvtkwidget
    vtkSmartPointer<vtkRenderWindow> renderWindow = m_pclViewer->getRenderWindow();
    m_vtkWidget->SetRenderWindow(renderWindow);

    /// these are useful to add to make the controls more like pcd_viewer
    m_pclViewer->setupInteractor(m_vtkWidget->GetInteractor(), m_vtkWidget->GetRenderWindow());
    m_pclViewer->getInteractorStyle()->setKeyboardModifier(pcl::visualization::INTERACTOR_KB_MOD_SHIFT);

    /// use dark gray background color
    m_pclViewer->setBackgroundColor(0.1f, 0.1f, 0.1f);

    m_pclViewer->initCameraParameters();
}

ZLaserTriangulationCalibrator::~ZLaserTriangulationCalibrator()
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

ZLTSCalibrationImageModel *ZLaserTriangulationCalibrator::imageModel() const
{
    return m_imageModel;
}

ZCalibrationPatternFinder::Ptr ZLaserTriangulationCalibrator::patternFinder() const
{
    return m_patternFinder;
}

ZCameraCalibration *ZLaserTriangulationCalibrator::cameraCalibration() const
{
    return m_cameraCalibration;
}

float ZLaserTriangulationCalibrator::progress() const
{
    return m_progress;
}

QWidget *ZLaserTriangulationCalibrator::getViewerWidget() const
{
    return m_vtkWidget;
}

void ZLaserTriangulationCalibrator::setImageModel(ZLTSCalibrationImageModel *imageModel)
{
    if (m_imageModel != imageModel) {

        if (m_imageModel) {
            /// disconnect old signals
            QObject::disconnect(m_imageModel.data(), SIGNAL(newImagesAdded()),
                                this, SLOT(findCalibrationPattern()));
        }

        /// update
        m_imageModel = imageModel;

        if (m_imageModel) {
            /// connect new signals
            QObject::connect(m_imageModel.data(), SIGNAL(newImagesAdded()),
                             this, SLOT(findCalibrationPattern()));
        }

        /// notify
        emit imageModelChanged(imageModel);
    }
}

void ZLaserTriangulationCalibrator::setPatternFinder(ZCalibrationPatternFinder::Ptr patternFinder)
{
    if (m_patternFinder != patternFinder) {
        if (m_patternFinder) {
            /// disconnect old signals
            QObject::disconnect(m_patternFinder.data(), SIGNAL(configHashChanged(QString)),
                                this, SLOT(findCalibrationPattern()));
        }

        /// update
        m_patternFinder = patternFinder;

        if (m_patternFinder) {
            /// connect new signals
            QObject::connect(m_patternFinder.data(), SIGNAL(configHashChanged(QString)),
                             this, SLOT(findCalibrationPattern()));
        }

        /// notify
        emit patternFinderChanged(patternFinder);
    }
}

void ZLaserTriangulationCalibrator::setCameraCalibration(ZCameraCalibration *cameraCalibration)
{
    if (m_cameraCalibration != cameraCalibration) {
        m_cameraCalibration = cameraCalibration;
        emit cameraCalibrationChanged(cameraCalibration);
    }
}

void ZLaserTriangulationCalibrator::setProgress(float progress, QString message)
{
    if (m_progress != progress) {
        m_progress = progress;
        emit progressChanged(progress, message);
    }
}

void ZLaserTriangulationCalibrator::setProgressValue(int arg)
{
    setProgress( (float) arg / m_maxProgressValue );
}

void ZLaserTriangulationCalibrator::setProgressRange(int min, int max)
{
    m_minProgressValue = min;
    m_maxProgressValue = max;
}

void ZLaserTriangulationCalibrator::findCalibrationPattern()
{
    if (!m_imageModel || !m_patternFinder || !m_imageModel->rowCount())
        return;

    if (m_patternFinderFutureWatcher.isRunning()) {
        qWarning() << Q_FUNC_INFO << "canceling previous pattern finder...";
        m_patternFinderFutureWatcher.future().cancel();
        m_patternFinderFutureWatcher.waitForFinished();
    }

    qDebug() << Q_FUNC_INFO << "using" << QThread::idealThreadCount() << "thread(s)...";

    /// 0%
    setProgress(0.f, tr("Finding calibration patterns..."));

    /// execution on parallel in other threads (from the thread pool)
    m_patternFinderFutureWatcher.setFuture(
                QtConcurrent::map(m_imageModel->images(),
                                  ZLaserTriangulationCalibratorWorkerParallelFindPatternImpl(m_patternFinder)) );
}

void ZLaserTriangulationCalibrator::calibrate()
{
    if (!m_imageModel)
        return;

    /// if for some reason the task is executed simultaneously, we need to wait
    /// to set the future to be able to exit cleanly
    if (m_calibrateFutureWatcher.isRunning()) {
        qWarning() << Q_FUNC_INFO << "canceling previous calibration...";
        m_calibrateFutureWatcher.future().cancel();
        m_calibrateFutureWatcher.waitForFinished();
    }

    /// starts execution in other thread
//    m_calibrateFutureWatcher.setFuture(
//                QtConcurrent::run(this, &ZLaserTriangulationCalibrator::calibrateFunctionImpl) );
    /// or run from gui thread (because vtk throws some warnings...)
    calibrateFunctionImpl();
}

void ZLaserTriangulationCalibrator::saveResults(const QString &fileName)
{
    //! TODO
}

void ZLaserTriangulationCalibrator::calibrateFunctionImpl()
{
    if (m_patternFinderFutureWatcher.isRunning()) {
        qWarning() << Q_FUNC_INFO << "waiting for pattern finder to finish...";
        m_patternFinderFutureWatcher.waitForFinished();
    }

    qDebug() << Q_FUNC_INFO << "starting...";

    QTime time;
    time.start();

    /// 0% only to show message
    setProgress(0.f, tr("Starting LTS calibration..."));

    int imageCount = m_imageModel->rowCount();

    /// get only the images where the calibration pattern was detected
    QVector< Z3D::ZLTSCalibrationImage::Ptr > validImages;
    validImages.reserve(imageCount);
    for (int i=0; i<imageCount; ++i) {
        Z3D::ZLTSCalibrationImage::Ptr image = m_imageModel->imageAt(i);
        if (image->state() == ZLTSCalibrationImage::PatternFoundState) {
            validImages.push_back(image);
        }
    }

    int validImageCount = validImages.size();

    if (validImageCount < 2) {
        /// not enough valid images
        qWarning() << "could not calibrate system, not enough images with calibration pattern found";

        /// calibration end
        setProgress(1.f, tr("Calibration failed. Not enough calibration patterns found"));
        emit calibrationChanged();

        return;
    }

    //! check camera calibration
    if (!m_cameraCalibration) {
        /// camera calibration not valid
        qWarning() << "the camera calibration is invalid!";

        /// calibration end
        setProgress(1.f, tr("Calibration failed. The camera calibration is invalid."));
        emit calibrationChanged();

        return;
    }

    /// 0% only to show message
    setProgress(0.f, tr("Estimating calibration pattern positions..."));

    /// execution on parallel in other threads (from the thread pool)
    m_patternFinderFutureWatcher.setFuture(
                QtConcurrent::map(validImages, ZLaserTriangulationCalibratorWorkerParallelPatternPositionEstimationImpl(m_cameraCalibration)) );
    qWarning() << Q_FUNC_INFO << "waiting for pattern position estimation to finish...";
    m_patternFinderFutureWatcher.waitForFinished();







    setProgress(0.4f, tr("Running LTS calibration..."));

    /// remove everything from the viewer
    m_pclViewer->removeAllPointClouds();
    m_pclViewer->removeAllShapes();

    PointCloudPCLPtr cloud(new PointCloudPCL);
    int points_counter = 0;
    foreach (Z3D::ZLTSCalibrationImage::Ptr image, validImages) {
        const std::vector<cv::Point2f> &validLaserPoints = image->validLaserPoints();
        points_counter += validLaserPoints.size();
    }

    /// resize the cloud (reserve max size)
    cloud->width  = points_counter;
    cloud->height = 1;
    cloud->points.resize(points_counter);

    int i = 0;
    int level = 0;
    foreach (Z3D::ZLTSCalibrationImage::Ptr image, validImages) {
        level++;

        /// calculate 3d position of laser points over the checkerboard plane
        const std::vector<cv::Point2f> &validLaserPoints = image->validLaserPoints();
        int laserPointsSize = validLaserPoints.size();
        if (laserPointsSize>0) {
            /// Create a pcl point cloud of laser points in 3d space, where we'll fit a line
            PointCloudPCLPtr laserLineCloud(new PointCloudPCL);
            /// Resize the cloud
            laserLineCloud->width  = laserPointsSize;
            laserLineCloud->height = 1;
            laserLineCloud->points.resize(laserPointsSize);

            /// undistort laser points
            std::vector<cv::Point2f> undistortedLaserPoints;
            undistortedLaserPoints.resize(laserPointsSize);
            /// undistortedLaserPoints will already be in object space, not pixels, since we don't specify "P" matrix below
            for (int idx = 0; idx<laserPointsSize; ++idx) {
                const cv::Point2f &point = validLaserPoints[idx];
                cv::Point2f &undistortedPoint = undistortedLaserPoints[idx];
                m_cameraCalibration->getCalibratedPixel(point.x, point.y, &undistortedPoint.x, &undistortedPoint.y);
            }


            /// get R y T between camera coordinate system and checkerboard coordinate system
            cv::Matx33f r = image->rotation();
            const cv::Point3f ckeckerboardOrigin = image->translation();
            const cv::Vec3f checkerboardNormal = r * cv::Vec3f(0,0,1); // checkerboard normal (z axis from the checkerboard coordinate system)

            /// line origin (each image point correspond to a line starting at the camera coordinate origin)
            const cv::Point3f cameraProjectionOrigin(0,0,0);

            uint32_t rgb_black = (static_cast<uint32_t>(128) << 16 |
                                  static_cast<uint32_t>(128) <<  8 |
                                  static_cast<uint32_t>(128)       );

            /// calculate 3d points as intersection of camera projection line and checkerboard plane
            for (int i1=0; i1<laserPointsSize; ++i1) {
                const cv::Point3f lineDir(undistortedLaserPoints[i1].x, undistortedLaserPoints[i1].y, 1.f);
                const cv::Point3f intersec = linePlaneIntersection(cameraProjectionOrigin, lineDir, ckeckerboardOrigin, checkerboardNormal);

                laserLineCloud->points[i1].x = intersec.x;
                laserLineCloud->points[i1].y = intersec.y;
                laserLineCloud->points[i1].z = intersec.z;

                /// black color
                laserLineCloud->points[i1].intensity /*rgb*/ = *reinterpret_cast<float*>(&rgb_black);
            }

            /// segment a line from the laser points (to avoid outliers when fitting plane)
            pcl::ModelCoefficients::Ptr lineCoefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr lineInliers(new pcl::PointIndices);
            pcl::SACSegmentation<PointType> lineSeg;
            /// Optional
            bool m_lineOptimizeCoeffs = true;
            lineSeg.setOptimizeCoefficients(m_lineOptimizeCoeffs);
            /// Mandatory
            lineSeg.setModelType(pcl::SACMODEL_LINE);

            //pcl::SAC_RANSAC
            //pcl::SAC_LMEDS
            lineSeg.setMethodType(pcl::SAC_RANSAC);

            double m_lineThresholdDistance = 0.005;
            lineSeg.setDistanceThreshold(m_lineThresholdDistance);

            lineSeg.setInputCloud(laserLineCloud);
            lineSeg.segment(*lineInliers, *lineCoefficients);

            //pcl::getMinMax3D(laserLineCloud, lineInliers, line_min, line_max);
            bool m_lineShowLine = true;
            if (m_lineShowLine)
                m_pclViewer->addLine(*lineCoefficients, QString("laser_line_%1").arg(level).toStdString());

            //pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(laserLineCloud);
            pcl::visualization::PointCloudColorHandlerGenericField<PointType> rgb(laserLineCloud, "intensity");
            m_pclViewer->addPointCloud<PointType>(laserLineCloud, rgb, QString("laser_points_%1").arg(level).toStdString());

            if (lineInliers->indices.size () == 0) {
                qDebug() << "Could not estimate a line";
            } else {
                uint32_t pointColor = ( static_cast<uint32_t>(255) << 16 |
                                        static_cast<uint32_t>(255) <<  8 |
                                        static_cast<uint32_t>(  0)       );

                /// add line inliers to point cloud where we'll fit a plane
                for (unsigned int i2 = 0; i2 < lineInliers->indices.size(); ++i2) {
                    cloud->points[i] = laserLineCloud->points[lineInliers->indices[i2]];
                    cloud->points[i].intensity /*rgb*/ = *reinterpret_cast<float*>(&pointColor);
                    i++;
                }
            }
        }
    }

    /// Resize the cloud with final size
    cloud->width  = i;
    cloud->height = 1;
    cloud->points.resize(i);

    /// Segment plane
    pcl::ModelCoefficients::Ptr planeCoefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr planeInliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointType> planeSeg;
    /// Optional
    bool m_planeOptimizeCoeffs = true;
    planeSeg.setOptimizeCoefficients(m_planeOptimizeCoeffs);
    /// Mandatory
    planeSeg.setModelType(pcl::SACMODEL_PLANE);
    planeSeg.setMethodType(pcl::SAC_RANSAC);
    double m_planeThreshold = 0.005;
    planeSeg.setDistanceThreshold(m_planeThreshold);

    planeSeg.setInputCloud(cloud);
    planeSeg.segment(*planeInliers, *planeCoefficients);

    std::cout << "Model coefficients: " << planeCoefficients->values[0] << " "
                                        << planeCoefficients->values[1] << " "
                                        << planeCoefficients->values[2] << " "
                                        << planeCoefficients->values[3] << std::endl;

    if (planeInliers->indices.size () == 0) {
        qDebug() << "Could not estimate a plane";
    } else {
        /// set red color to inliers
        uint32_t pointColor = ( static_cast<uint32_t>(255) << 16 |
                                static_cast<uint32_t>(  0) <<  8 |
                                static_cast<uint32_t>(  0)       );
        for (size_t i = 0; i < planeInliers->indices.size(); ++i)
            cloud->points[planeInliers->indices[i]].intensity /*rgb*/ = *reinterpret_cast<float*>(&pointColor);
    }

    //pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(cloud);
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> rgb(cloud, "intensity");
    m_pclViewer->addPointCloud<PointType>(cloud, rgb, "laser_points");

    m_pclViewer->addPlane(*planeCoefficients, "laser_plane");

    //m_pclViewer->addPolylineFromPolygonMesh(pclOpenCVUtils::getCameraMesh());
    //m_pclViewer->addPolygonMesh(pclOpenCVUtils::getCameraMesh());


    /// 100%
    setProgress(1.f, tr("Calibration finished in %1 msecs").arg(time.elapsed()));

    /// calibration end
    emit calibrationChanged();
}

} // namespace Z3D
