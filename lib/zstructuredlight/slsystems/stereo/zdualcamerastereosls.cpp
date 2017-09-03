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

#include "zdualcamerastereosls.h"

#include "zcalibratedcameraprovider.h"
#include "zpinhole/zpinholecameracalibration.h"

namespace Z3D
{

ZDualCameraStereoSLS::ZDualCameraStereoSLS(QObject *parent)
    : ZStereoSLS(parent)
{

}

ZDualCameraStereoSLS::~ZDualCameraStereoSLS()
{
    qDebug() << "deleting cameras...";
    m_cameras.clear();
}

ZCalibratedCamera::Ptr ZDualCameraStereoSLS::leftCamera() const
{
    return m_cameras[0];
}

ZCalibratedCamera::Ptr ZDualCameraStereoSLS::rightCamera() const
{
    return m_cameras[1];
}

void ZDualCameraStereoSLS::addCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameras)
{
    setReady(false);

    /// add cameras to list
    m_cameras.clear();

    if (cameras.size() >= 2) {
        m_cameras.resize(2);
        setLeftCamera(cameras[0]);
        setRightCamera(cameras[1]);
    }

    std::vector<Z3D::ZCameraInterface::Ptr> camerasVector;
    for (auto cam : cameras)
        camerasVector.push_back(cam->camera());
    setAcquisitionManager(new ZCameraAcquisitionManager(camerasVector));
}

void ZDualCameraStereoSLS::setLeftCamera(ZCalibratedCamera::Ptr camera)
{
    if (m_cameras[0] != camera) {
        /// check first! this only works for pinhole cameras!
        auto *calibration = static_cast<Z3D::ZPinholeCameraCalibration*>(camera->calibration().data());

        if (!calibration) {
            qWarning() << "invalid calibration! this only works for pinhole cameras!";
            return;
        }

        if (m_cameras[0]) {
            disconnect(m_cameras[0].data(), &Z3D::ZCalibratedCamera::calibrationChanged,
                       this, &ZDualCameraStereoSLS::setLeftCalibration);
        }

        m_cameras[0] = camera;

        connect(m_cameras[0].data(), &Z3D::ZCalibratedCamera::calibrationChanged,
                this, &ZDualCameraStereoSLS::setLeftCalibration);

        setLeftCalibration(camera->calibration());
    }
}

void ZDualCameraStereoSLS::setRightCamera(ZCalibratedCamera::Ptr camera)
{
    if (m_cameras[1] != camera) {
        /// check first! this only works for pinhole cameras!
        auto *calibration = static_cast<Z3D::ZPinholeCameraCalibration*>(camera->calibration().data());

        if (!calibration) {
            qWarning() << "invalid calibration! this only works for pinhole cameras!";
            return;
        }

        if (m_cameras[1]) {
            disconnect(m_cameras[1].data(), &Z3D::ZCalibratedCamera::calibrationChanged,
                       this, &ZDualCameraStereoSLS::setRightCalibration);
        }

        m_cameras[1] = camera;

        connect(m_cameras[1].data(), &Z3D::ZCalibratedCamera::calibrationChanged,
                this, &ZDualCameraStereoSLS::setRightCalibration);

        setRightCalibration(camera->calibration());
    }
}

QString ZDualCameraStereoSLS::id() const
{
    return QString("DualCamera");
}

QString ZDualCameraStereoSLS::displayName() const
{
    return QString("Dual cameras");
}

void ZDualCameraStereoSLS::init(QSettings *settings)
{
    QList<ZCalibratedCamera::Ptr> cameras;

    settings->beginGroup("Left");
    {
        cameras << CalibratedCameraProvider::getCalibratedCamera(settings);
    }
    settings->endGroup();

    settings->beginGroup("Right");
    {
        cameras << CalibratedCameraProvider::getCalibratedCamera(settings);
    }
    settings->endGroup();

    addCameras(cameras);
}

void ZDualCameraStereoSLS::onPatternProjected(ZProjectedPattern::Ptr pattern)
{
    Q_UNUSED(pattern);
}

void ZDualCameraStereoSLS::onPatternsDecoded(std::vector<ZDecodedPattern::Ptr> decodedPatterns)
{
    /// is there something to process?
    for (auto decodedPattern : decodedPatterns) {
        if (decodedPattern->estimatedCloudPoints < 1)
            return;
    }

    int estimatedCloudPoints = 0;
    for (const auto &decodedPattern : decodedPatterns)
        estimatedCloudPoints += decodedPattern->estimatedCloudPoints;

    Z3D::ZSimplePointCloud::Ptr cloud = triangulate(
                decodedPatterns[0]->intensityImg,
                decodedPatterns[0]->fringePointsList,
            decodedPatterns[1]->fringePointsList,
            estimatedCloudPoints);

    if (cloud) {
        emit scanFinished(cloud);
    }
}

} // namespace Z3D
