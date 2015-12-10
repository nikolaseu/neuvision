#include "zpinholecameracalibration.h"

#include <opencv2/imgproc/imgproc.hpp> // undistortPoints
#include <opencv2/calib3d/calib3d.hpp> // rodrigues, solvePnP

#include <iostream>

#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QSettings>
#include <QtConcurrentRun>

namespace Z3D
{

ZPinholeCameraCalibration::ZPinholeCameraCalibration()
    : ZCameraCalibration(ZCameraCalibration::CentralCameraType)
{
    m_cvCameraMatrix.create(3, 3, CV_64F);

#if defined(CV_VERSION_EPOCH) && CV_VERSION_EPOCH < 3
    /// In OpenCV < 3, when using rational model, max size is (1x8)
    m_cvDistortionCoeffs.create(1, 8, CV_64F);
#else
    /// In OpenCV 3, when using rational model, max size seems to be (1x12)
    m_cvDistortionCoeffs.create(1, 12, CV_64F);
#endif

    //m_cvCameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    //m_cvDistortionCoeffs = cv::Mat::zeros(1, 12, CV_64F);

    for (int r=0; r<m_cvDistortionCoeffs.rows; ++r)
        for (int c=0; c<m_cvDistortionCoeffs.cols; ++c)
            m_cvDistortionCoeffs.at<double>(r,c) = 0.;

    QObject::connect(this, SIGNAL(calibrationChanged()),
                     this, SIGNAL(pinholeCalibrationChanged()));

    QObject::connect(this, SIGNAL(calibrationChanged()),
                     this, SLOT(onCalibrationChanged()));
}

ZPinholeCameraCalibration::ZPinholeCameraCalibration(cv::Mat cameraMatrix, cv::Mat distCoeffs, cv::Size imageSize)
    : ZCameraCalibration(ZCameraCalibration::CentralCameraType)
{
    m_sensorWidth = imageSize.width;
    m_sensorHeight = imageSize.height;

    m_cvCameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
    /// In OpenCV 3, when using rational model, size seems to be (1x12)
    m_cvDistortionCoeffs = cv::Mat::zeros(1, qMax(8, distCoeffs.cols), CV_64F);
/*
    m_cvCameraMatrix.create(3, 3, CV_64F);
    m_cvCameraMatrix = cv::Scalar::all(0);

    m_cvDistortionCoeffs.create(1, 12, CV_64F);
    m_cvDistortionCoeffs = cv::Scalar::all(0);
*/
    /// convert to double precision and load into class variables
    cameraMatrix.convertTo(m_cvCameraMatrix, CV_64F);

    /// If we copy directly, the distortion vector might be 1x5 if the rational
    /// was not used. In order to allow modifications, we always use the full
    /// size (1x8).
    /// In OpenCV 3, when using rational model, size seems to be (1x12)
    /// Then we need to copy one by one
    //distCoeffs.convertTo(m_cvDistortionCoeffs, CV_64F);
    cv::Mat distCoeffs64;
    distCoeffs.convertTo(distCoeffs64, CV_64F);
    for (int r=0; r<distCoeffs64.rows; ++r)
        for (int c=0; c<distCoeffs64.cols; ++c)
            m_cvDistortionCoeffs.at<double>(r,c) = distCoeffs64.at<double>(r,c);

    QObject::connect(this, SIGNAL(calibrationChanged()),
                     this, SIGNAL(pinholeCalibrationChanged()));

    QObject::connect(this, SIGNAL(calibrationChanged()),
                     this, SLOT(onCalibrationChanged()));

    emit calibrationChanged();
}

ZCameraCalibration::Ptr ZPinholeCameraCalibration::clone() const
{
    ZCameraCalibration::Ptr clon(new ZPinholeCameraCalibration(m_cvCameraMatrix.clone(), m_cvDistortionCoeffs.clone(), cv::Size(m_sensorWidth, m_sensorHeight)));
    return clon;
}

bool ZPinholeCameraCalibration::getCalibratedPixel(int x, int y, float *calibratedX, float *calibratedY)
{
    //qDebug() << Q_FUNC_INFO << "x:" << x << "y:" << y;

    if (!ready()) {
        *calibratedX = x;
        *calibratedY = y;
        return false;
    }

    /// check if the pixel is inside the calibrated range
    if (x < 0 || x >= m_sensorWidth || y < 0 || y >= m_sensorHeight) {
        qWarning() << "pixel outside the calibration range:" << x << "," << y << "calibrated size:" << m_sensorWidth << "," << m_sensorHeight;
        *calibratedX = x;
        *calibratedY = y;
        return false;
    }

    const cv::Point2f &undistortedPoint = m_undistortedPoints[ indexForPixel(x, y) ];
    *calibratedX = undistortedPoint.x;
    *calibratedY = undistortedPoint.y;

    return true;
}

bool ZPinholeCameraCalibration::getRayForPixel(int x, int y, cv::Vec3d &origin, cv::Vec3d &direction)
{
    if (!ready()) {
        return false;
    }

    if (x < 0 || x >= m_sensorWidth || y < 0 || y >= m_sensorHeight) {
        qWarning() << "pixel outside the calibration range:" << x << "," << y << "calibrated size:" << m_sensorWidth << "," << m_sensorHeight;
        return false;
    }

    origin = m_translation;

    direction = m_undistortedRays[ indexForPixel(x, y) ];

    return true;
}

bool ZPinholeCameraCalibration::getWorldRayForPixel(int x, int y, cv::Vec3d &origin, cv::Vec3d &direction)
{
    if (!ready()) {
        return false;
    }

    if (x < 0 || x >= m_sensorWidth || y < 0 || y >= m_sensorHeight) {
        qWarning() << "pixel outside the calibration range:" << x << "," << y << "calibrated size:" << m_sensorWidth << "," << m_sensorHeight;
        return false;
    }

    origin = m_translation;

    direction = m_undistortedWorldRays[ indexForPixel(x, y) ];

    return true;
}

bool ZPinholeCameraCalibration::getEstimatedTransformation(std::vector<cv::Point2f> &imageCorners, std::vector<cv::Point3f> &worldCorners, cv::Matx33d &rotation, cv::Point3d &translation)
{
    cv::Mat rotationMat;
    cv::Mat translationMat;

    /// solve system
    bool valid = cv::solvePnP(worldCorners,
                              imageCorners,
                              m_cvCameraMatrix,
                              m_cvDistortionCoeffs,
                              rotationMat,
                              translationMat);

    if (valid) {
        /// rotation was expressed as a vector, we need as matrix. Use rodrigues' formula
        cv::Vec3d rvec(rotationMat);
        cv::Rodrigues(rvec, rotation);
        /// convert translation also
        translation = cv::Point3d(translationMat);
    }

    return valid;
}

bool ZPinholeCameraCalibration::loadFromFile(const QString &fileName)
{
    if (fileName.isEmpty() || fileName.isNull())
        return false;

    QDir currentDir = QDir::current();
    if (!currentDir.exists(fileName)) {
    #if defined(Q_OS_WIN)
//        if (pluginsDir.dirName().toLower() == "debug" || pluginsDir.dirName().toLower() == "release")
//            pluginsDir.cdUp();
    #elif defined(Q_OS_MAC)
        if (currentDir.dirName() == "MacOS") {
            currentDir.cdUp();
            currentDir.cdUp();
            currentDir.cdUp();
        }
    #endif

        if (!currentDir.exists(fileName)) {
            qWarning() << "calibration file" << fileName << "not found in" << currentDir.absolutePath();
            return false;
        }
    }

    bool valid = false;

    blockSignals(true);

    QSettings settings(currentDir.absoluteFilePath(fileName), QSettings::IniFormat);

    QString calibDate;

    settings.beginGroup("CameraCalibration");
    {
        if (settings.value("Type", "UNKNOWN").toString() == PINHOLE_CALIB_NAME) {
            calibDate = settings.value("CalibrationDate", "UNKNOWN").toString();
            m_sensorWidth = settings.value("SensorWidth",  m_sensorWidth).toInt();
            m_sensorHeight = settings.value("SensorHeight",  m_sensorHeight).toInt();

            /// rotation
            cv::Mat rotationMat = cv::Mat(rotation()).reshape(0, 1).clone();
            std::vector<double> rotationVector = rotationMat;
            int size = settings.beginReadArray("Rotation");
            for (int r=0; r<size; ++r) {
                settings.setArrayIndex(r);
                rotationVector[r] = settings.value("value", rotationVector[r]).toDouble();
            }
            settings.endArray();
            setRotation( cv::Mat(rotationVector).reshape(0, 3).clone() );

            /// translation
            cv::Mat translationMat = cv::Mat(m_translation).reshape(0, 1).clone();
            std::vector<double> translationVector = translationMat;
            size = settings.beginReadArray("Translation");
            for (int t=0; t<size; ++t) {
                settings.setArrayIndex(t);
                translationVector[t] = settings.value("value", translationVector[t]).toDouble();
            }
            settings.endArray();
            setTranslation( cv::Mat(translationVector).clone() );

            settings.beginGroup(PINHOLE_CALIB_NAME);
            {
                /// camera matrix
                setCx( settings.value("cx", cx()).toDouble() );
                setCy( settings.value("cy", cy()).toDouble() );
                setFx( settings.value("fx", fx()).toDouble() );
                setFy( settings.value("fy", fy()).toDouble() );

                /// tangential distortion
                setP1( settings.value("p1", p1()).toDouble() );
                setP2( settings.value("p2", p2()).toDouble() );

                /// radial distortion
                setK1( settings.value("k1", k1()).toDouble() );
                setK2( settings.value("k2", k2()).toDouble() );
                setK3( settings.value("k3", k3()).toDouble() );
                setK4( settings.value("k4", k4()).toDouble() );
                setK5( settings.value("k5", k5()).toDouble() );
                setK6( settings.value("k6", k6()).toDouble() );
            }
            settings.endGroup();

            valid = true;
        }
    }
    settings.endGroup();

    if (valid) {
        /// show data in console
        std::cout << "Loaded pinhole camera calibration from:\n " << qPrintable(fileName) << std::endl
                  << "> calibrationDate:\n " << qPrintable(calibDate) << std::endl
                  << "> sensorWidth:\n " << m_sensorWidth << std::endl
                  << "> sensorHeight:\n " << m_sensorHeight << std::endl
                  << "> cameraMatrix:\n " << m_cvCameraMatrix << std::endl
                  << "> distCoeffs:\n " << m_cvDistortionCoeffs << std::endl
                  << "> rotation:\n " << rotation() << std::endl
                  << "> translation:\n " << translation() << std::endl;
    }

    blockSignals(false);

    emit calibrationChanged();

    return valid;
}

bool ZPinholeCameraCalibration::saveToFile(const QString &fileName)
{
    QSettings settings(fileName, QSettings::IniFormat);

    if (!settings.isWritable()) {
        qWarning() << "Error: can not open/write calibration file" << fileName;
        return false;
    }

    settings.beginGroup("CameraCalibration");
    {
        settings.setValue("Type", PINHOLE_CALIB_NAME);
        settings.setValue("CalibrationDate",  QDateTime::currentDateTime().toString(Qt::ISODate));
        settings.setValue("SensorWidth",  m_sensorWidth);
        settings.setValue("SensorHeight",  m_sensorHeight);

        /// rotation
        cv::Mat rotationMat = cv::Mat(rotation()).reshape(0, 1).clone();
        std::vector<double> rotationVector = rotationMat;
        settings.beginWriteArray("Rotation", rotationVector.size());
        for (size_t r=0; r<rotationVector.size(); ++r) {
            settings.setArrayIndex(r);
            settings.setValue("value", rotationVector[r]);
        }
        settings.endArray();

        /// translation
        cv::Mat translationMat = cv::Mat(translation()).reshape(0, 1).clone();
        std::vector<double> translationVector = translationMat;
        settings.beginWriteArray("Translation", translationVector.size());
        for (size_t t=0; t<translationVector.size(); ++t) {
            settings.setArrayIndex(t);
            settings.setValue("value", translationVector[t]);
        }
        settings.endArray();

        settings.beginGroup(PINHOLE_CALIB_NAME);
        {
            /// camera matrix
            settings.setValue("cx", cx());
            settings.setValue("cy", cy());
            settings.setValue("fx", fx());
            settings.setValue("fy", fy());

            /// tangential distortion
            settings.setValue("p1", p1());
            settings.setValue("p2", p2());

            /// radial distortion
            settings.setValue("k1", k1());
            settings.setValue("k2", k2());
            settings.setValue("k3", k3());
            settings.setValue("k4", k4());
            settings.setValue("k5", k5());
            settings.setValue("k6", k6());
        }
        settings.endGroup();
    }
    settings.endGroup();
    settings.sync();

    return true;

/*
    cv::FileStorage fs(qPrintable(fileName), cv::FileStorage::WRITE);
    if( fs.isOpened() ) {
        /// save calibration date
        fs << "calibrationDate" << qPrintable( QDateTime::currentDateTime().toString(Qt::ISODate) );

        /// save sensor (image) size
        fs << "sensorWidth" << m_sensorWidth;
        fs << "sensorHeight" << m_sensorHeight;

        /// save calibration results
        fs << "cameraMatrix" << m_cvCameraMatrix;
        fs << "distCoeffs" << m_cvDistortionCoeffs;

        fs.release();

        return true;
    }

    qWarning() << "unable to save calibration. cannot open file:" << fileName;
    return false;*/
}

double ZPinholeCameraCalibration::cx() const
{
    return m_cvCameraMatrix.at<double>(0, 2);
}

double ZPinholeCameraCalibration::cy() const
{
    return m_cvCameraMatrix.at<double>(1, 2);
}

double ZPinholeCameraCalibration::fx() const
{
    return m_cvCameraMatrix.at<double>(0, 0);
}

double ZPinholeCameraCalibration::fy() const
{
    return m_cvCameraMatrix.at<double>(1, 1);
}

double ZPinholeCameraCalibration::p1() const
{
    return m_cvDistortionCoeffs.at<double>(0, 2);
}

double ZPinholeCameraCalibration::p2() const
{
    return m_cvDistortionCoeffs.at<double>(0, 3);
}

double ZPinholeCameraCalibration::k1() const
{
    return m_cvDistortionCoeffs.at<double>(0, 0);
}

double ZPinholeCameraCalibration::k2() const
{
    return m_cvDistortionCoeffs.at<double>(0, 1);
}

double ZPinholeCameraCalibration::k3() const
{
    return m_cvDistortionCoeffs.at<double>(0, 4);
}

double ZPinholeCameraCalibration::k4() const
{
    /// only present when calibrated using rational model
    if (m_cvDistortionCoeffs.cols > 5)
        return m_cvDistortionCoeffs.at<double>(0, 5);
    else
        return 0;
}

double ZPinholeCameraCalibration::k5() const
{
    /// only present when calibrated using rational model
    if (m_cvDistortionCoeffs.cols > 6)
        return m_cvDistortionCoeffs.at<double>(0, 6);
    else
        return 0;
}

double ZPinholeCameraCalibration::k6() const
{
    /// only present when calibrated using rational model
    if (m_cvDistortionCoeffs.cols > 7)
        return m_cvDistortionCoeffs.at<double>(0, 7);
    else
        return 0;
}

QPointF ZPinholeCameraCalibration::principalPoint() const
{
    return QPointF(cx(), cy());
}

QPointF ZPinholeCameraCalibration::focalLength() const
{
    return QPointF(fx(), fy());
}

//QList<double> ZPinholeCameraCalibration::radialDistortion() const
//{
//    QList<double> m_radialDistortion;

//    m_radialDistortion << k1();
//    m_radialDistortion << k2();
//    m_radialDistortion << k3();
//    m_radialDistortion << k4();
//    m_radialDistortion << k5();
//    m_radialDistortion << k6();

//    return m_radialDistortion;
//}

//QList<double> ZPinholeCameraCalibration::tangentialDistortion() const
//{
//    QList<double> m_tangentialDistortion;

//    m_tangentialDistortion << p1();
//    m_tangentialDistortion << p2();

//    return m_tangentialDistortion;
//}

void ZPinholeCameraCalibration::setCx(double arg)
{
    if (cx() == arg)
        return;

    m_cvCameraMatrix.at<double>(0, 2) = arg;

    emit calibrationChanged();
}

void ZPinholeCameraCalibration::setCy(double arg)
{
    if (cy() == arg)
        return;

    m_cvCameraMatrix.at<double>(1, 2) = arg;

    emit calibrationChanged();
}

void ZPinholeCameraCalibration::setFx(double arg)
{
    if (fx() == arg)
        return;

    m_cvCameraMatrix.at<double>(0, 0) = arg;

    emit calibrationChanged();
}

void ZPinholeCameraCalibration::setFy(double arg)
{
    if (fy() == arg)
        return;

    m_cvCameraMatrix.at<double>(1, 1) = arg;

    emit calibrationChanged();
}

void ZPinholeCameraCalibration::setP1(double arg)
{
    if (p1() == arg)
        return;

    m_cvDistortionCoeffs.at<double>(0, 2) = arg;

    emit calibrationChanged();
}

void ZPinholeCameraCalibration::setP2(double arg)
{
    if (p2() == arg)
        return;

    m_cvDistortionCoeffs.at<double>(0, 3) = arg;

    emit calibrationChanged();
}

void ZPinholeCameraCalibration::setK1(double arg)
{
    if (k1() == arg)
        return;

    m_cvDistortionCoeffs.at<double>(0, 0) = arg;

    emit calibrationChanged();
}

void ZPinholeCameraCalibration::setK2(double arg)
{
    if (k2() == arg)
        return;

    m_cvDistortionCoeffs.at<double>(0, 1) = arg;

    emit calibrationChanged();
}

void ZPinholeCameraCalibration::setK3(double arg)
{
    if (k3() == arg)
        return;

    m_cvDistortionCoeffs.at<double>(0, 4) = arg;

    emit calibrationChanged();
}

void ZPinholeCameraCalibration::setK4(double arg)
{
    if (k4() == arg)
        return;

    m_cvDistortionCoeffs.at<double>(0, 5) = arg;

    emit calibrationChanged();
}

void ZPinholeCameraCalibration::setK5(double arg)
{
    if (k5() == arg)
        return;

    m_cvDistortionCoeffs.at<double>(0, 6) = arg;

    emit calibrationChanged();
}

void ZPinholeCameraCalibration::setK6(double arg)
{
    if (k6() == arg)
        return;

    m_cvDistortionCoeffs.at<double>(0, 7) = arg;

    emit calibrationChanged();
}

void ZPinholeCameraCalibration::setPrincipalPoint(QPointF arg)
{
    setCx(arg.x());
    setCy(arg.y());
}

void ZPinholeCameraCalibration::setFocalLength(QPointF arg)
{
    setFx(arg.x());
    setFy(arg.y());
}

void ZPinholeCameraCalibration::onCalibrationChanged()
{
    /// run in other thread
    QtConcurrent::run(this, &ZPinholeCameraCalibration::generateLookUpTable);
}

//void ZPinholeCameraCalibration::setRadialDistortion(QList<double> arg)
//{
//    if (radialDistortion() == arg)
//        return;

//    blockSignals(true);

//    setK1(arg[0]);
//    setK2(arg[1]);
//    setK3(arg[2]);
//    setK4(arg[3]);
//    setK5(arg[4]);
//    setK6(arg[5]);

//    blockSignals(false);

//    emit calibrationChanged();
//}

//void ZPinholeCameraCalibration::setTangentialDistortion(QList<double> arg)
//{
//    if (tangentialDistortion() == arg)
//        return;

//    blockSignals(true);

//    setP1(arg[0]);
//    setP2(arg[1]);

//    blockSignals(false);

//    emit calibrationChanged();
//}

void ZPinholeCameraCalibration::generateLookUpTable()
{
    setReady(false);

    std::size_t pixelCount = m_sensorWidth * m_sensorHeight;

    QTime time;
    time.start();

    qDebug() << "initializing undistorted rays lookup table with" << pixelCount << "values";

    /// fill "distorted" points for every pixel in the image
    std::vector<cv::Point2f> points;
    points.resize( pixelCount );
    for (int ix = 0; ix < m_sensorWidth; ++ix) {
        for (int iy = 0; iy < m_sensorHeight; ++iy) {
            points[ indexForPixel(ix, iy) ] = cv::Point2f(ix, iy);
        }
    }

    /// undistorted points will be returned here
    m_undistortedPoints.resize( pixelCount );

    /// undistortedPoints will already be in object space, not pixels, since we don't specify "P" matrix below
    cv::undistortPoints(points, m_undistortedPoints, m_cvCameraMatrix, m_cvDistortionCoeffs /*, InputArray R=noArray(), InputArray P=noArray()*/);

    m_undistortedRays.resize( pixelCount );
    m_undistortedWorldRays.resize( pixelCount );

    const cv::Matx33f &R = rotation();

    for (int ix = 0; ix < m_sensorWidth; ++ix) {
        for (int iy = 0; iy < m_sensorHeight; ++iy) {
            int index = indexForPixel(ix, iy);

            const cv::Point2f &undistortedPoint = m_undistortedPoints[index];

            /// store undistorted ray
            cv::Vec3f &ray = m_undistortedRays[index];
            ray[0] = undistortedPoint.x;
            ray[1] = undistortedPoint.y;
            ray[2] = 1.;

            /// store "world" ray
            /// use normalized (unit vector) to optimize intersection calculation
            m_undistortedWorldRays[index] = R * cv::normalize(ray);

        }
    }

    /// undistortedPoints will now be in pixel again
    cv::undistortPoints(points, m_undistortedPoints, m_cvCameraMatrix, m_cvDistortionCoeffs, cv::Mat(), m_cvCameraMatrix);

    qDebug() << "finished initialization of undistorted rays lookup table with" << pixelCount << "pixels in" << time.elapsed() << "msecs";

    setReady(true);
}

} // namespace Z3D
