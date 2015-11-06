#include "zltscamera3d.h"

#include "zltscamera3dcalibrationinterface.h"

#include <Z3DCameraAcquisition>

#include <QDebug>
#include <QTime>

namespace Z3D
{

LTSCamera3D::LTSCamera3D(ZCalibratedCamera::Ptr camera,
                         LTSCamera3DCalibrationInterface::Ptr calibration,
                         QObject *parent) :
    Camera3DInterface(camera, parent),
    m_ltsCalibration(calibration),
    m_acquisitionMode(UnknownAcquisitionMode),
    m_cloudsBufferSize(10),
    m_currentCloudIndex(-1),
    m_pointsStep(1),
    m_zStep(0.05f),
    m_profileDivisor(1) //! FIXME el 64 debe ser parametro de configuracion!
{
    QObject::connect(camera2d()->camera().data(), SIGNAL(newImageReceived(Z3D::ZImageGrayscale::Ptr)),
                     this, SLOT(onNewImageReceived(Z3D::ZImageGrayscale::Ptr)));

    /// prepare cloud buffer
    m_cloudsBuffer.reserve(m_cloudsBufferSize);
    for (int i=0; i<m_cloudsBufferSize; ++i) {
        m_cloudsBuffer.push_back( Z3D::ZPointCloud::Ptr(new Z3D::ZPointCloud) );
    }
}

LTSCamera3D::~LTSCamera3D()
{
    qDebug() << Q_FUNC_INFO;
}

int LTSCamera3D::profileDivisor() const
{
    return m_profileDivisor;
}

void LTSCamera3D::startAcquisition()
{
    if (!m_camera->camera()->isRunning())
        m_camera->camera()->startAcquisition();
}

void LTSCamera3D::stopAcquisition()
{
    if (m_camera->camera()->isRunning())
        m_camera->camera()->stopAcquisition();
}

int LTSCamera3D::acquisitionMode() const
{
    return m_acquisitionMode;
}

bool LTSCamera3D::setAcquisitionMode(int mode)
{
    if (m_acquisitionMode != mode) {
        /// do we allow changing mode while acquisition is running?
        if (m_camera->camera()->isRunning()) {
            qWarning() << "Unable to change acquisition mode: acquisition is running.";
            return false;
        }

        if (mode & ImageAcquisitionMode) {
            m_camera->camera()->loadPresetConfiguration("ImageAcquisitionMode");
        } else if (mode & ProfileAcquisitionMode) {
            m_camera->camera()->loadPresetConfiguration("ProfileAcquisitionMode");
        }

        m_acquisitionMode = mode;
    }

    return true;
}

void LTSCamera3D::setProfileDivisor(int arg)
{
    if (m_profileDivisor != arg) {
        m_profileDivisor = arg;
        emit profileDivisorChanged(arg);
    }
}

void LTSCamera3D::onNewImageReceived(Z3D::ZImageGrayscale::Ptr image)
{
    if ( !(m_acquisitionMode & ProfileAcquisitionMode || m_acquisitionMode & ImageAcquisitionMode) ) {
        /// we don't know the initial mode, it wasn't configured properly
        qWarning() << "unknown acquisition mode, what should we do?";
        return;
    }

    /// if in dual mode, stop acquisition right now
    if (m_acquisitionMode & DualAcquisitionMode) {
        m_camera->camera()->stopAcquisition();
    }

    /// what kink of "image" have we just received?
    if (m_acquisitionMode & ImageAcquisitionMode) {
        /// nothing to do, just emit signal
        emit newImageReceived(image);

    } else if (m_acquisitionMode & ProfileAcquisitionMode) {
        /// use profiles and convert to point cloud
        //processProfiles(image);
    }

    /// if in dual mode, switch mode and restart acquisition
    if (m_acquisitionMode & DualAcquisitionMode) {
        if (m_acquisitionMode & ImageAcquisitionMode)
            setAcquisitionMode(ProfileAcquisitionMode | DualAcquisitionMode);
        else
            setAcquisitionMode(ImageAcquisitionMode | DualAcquisitionMode);
        m_camera->camera()->startAcquisition();
    }
}

void LTSCamera3D::processProfiles(ZImageGrayscale::Ptr image)
{
//    QTime elapsedTime;
//    elapsedTime.start();

    m_currentCloudIndex = ++m_currentCloudIndex % m_cloudsBufferSize;

    const Z3D::ZPointCloud::Ptr &cloud2 = m_cloudsBuffer[m_currentCloudIndex];
    const PointCloudPCLPtr &cloud = cloud2->pclPointCloud();

    //if (cloud->width  != (image->width() / pointStep) || cloud->height != image->height()) {
    if (cloud->points.size() != ((image->width() / m_pointsStep) * image->height())) {
//        qDebug() << "resizing point cloud" << currentIndex
//                 << "(" << image->width() << "x" << image->height() << ")";

        cloud->width  = (image->width() / m_pointsStep) * image->height();
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);
    }

    std::vector<PointType, Eigen::aligned_allocator<PointType> >::iterator cloudPoint = cloud->points.begin();

    //const int startY = m_currentCloudIndex * image->height();
    const int startY = image->number() * image->height();

    float realX,
          realY,
          realZ;

    int validCloudPoints = 0;

    switch (image->bytesPerPixel()) {
    case 1: /// The image type is 8-bit
        {
            unsigned char *data = (unsigned char *) image->buffer();
            for (int iy=0; iy<image->height(); iy++) {
                for (int ix=0; ix<image->width(); ix += m_pointsStep) {
                    /// skip zero values
                    if (*data) {
                        const int absX = image->xOffset() + ix;
                        const int absY = image->yOffset() + iy;

                        getRealWorldPositionForPixel(absX, *data, &realX, &realY, &realZ);

                        cloudPoint->x = realX;
                        cloudPoint->y = realY;
                        cloudPoint->z = m_zStep * (startY + absY) + realZ;

                        ++cloudPoint; /// go to next point

                        ++validCloudPoints;
                    }

                    data += m_pointsStep; /// go to next pixel
                }
            }
        }
        break;
    case 2: /// The image type is 16-bit
        {
            unsigned short *data = (unsigned short *) image->buffer();
            for (int iy=0; iy<image->height(); iy++) {
                for (int ix=0; ix<image->width(); ix += m_pointsStep) {
                    /// skip zero values
                    if (*data) {
                        const int absX = image->xOffset() + ix;
                        const int absY = image->yOffset() + iy;

                        getRealWorldPositionForPixel(absX, *data, &realX, &realY, &realZ);

                        cloudPoint->x = realX;
                        cloudPoint->y = realY;
                        cloudPoint->z = m_zStep * (startY + absY) + realZ;

                        ++cloudPoint; /// go to next point

                        ++validCloudPoints;
                    }

                    data += m_pointsStep; /// go to next pixel
                }
            }
        }
        break;
    }

    if (validCloudPoints) {
        cloud->width = validCloudPoints;
        cloud->height = 1;
        cloud->points.resize(cloud->width * cloud->height);

        cloud2->setNumber(image->number());

        emit newPointCloudReceived(cloud2);
    } else {
        qDebug() << "empty point cloud. no laser line detected?";
    }

//    const int allFramesTime = elapsedTime.elapsed();
//    qDebug() << "cloud computation time:" << allFramesTime << "msecs (" << (float)allFramesTime/image->height() << "msecs/profile)";
}

void LTSCamera3D::getRealWorldPositionForPixel(int x, int y, float *realX, float *realY, float *realZ)
{
    /// use laser calibration (if we have it)
    if (m_ltsCalibration) {
        float y2 = ((float)y) / profileDivisor();

        float yl = floor(y2);
        float yu = yl + 1.f;

        float realXL,
              realYL,
              realXU,
              realYU,
              realZL,
              realZU;

        m_ltsCalibration->getCalibratedPointForPixel(x, yl, &realXL, &realYL, &realZL);
        m_ltsCalibration->getCalibratedPointForPixel(x, yu, &realXU, &realYU, &realZU);

        float lFactor = yu - y2;
        float uFactor = y2 - yl;

        *realX = lFactor * realXL + uFactor * realXU;
        *realY = lFactor * realYL + uFactor * realYU;
        *realZ = lFactor * realZL + uFactor * realZU;

//        qDebug() << "x" << x << "y2" << y2 << "yl" << yl << "yu" << yu;
    } else {
        /// fallback, uncalibrated camera
        *realX = x;
        *realY = y;
        *realZ = 0;
    }
}

} // namespace Z3D
