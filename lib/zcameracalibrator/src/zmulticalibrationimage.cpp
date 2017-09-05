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

#include "zmulticalibrationimage.h"

#include <QPainter>

namespace Z3D
{

//! FIXME esto no va acá
static int _sharedPointerMetaTypeId = qRegisterMetaType< Z3D::ZMultiCalibrationImage::Ptr >("Z3D::ZMultiCalibrationImage::Ptr");

ZMultiCalibrationImage::ZMultiCalibrationImage(QList<ZCalibrationImage::Ptr> images)
    : m_images(images)
    , m_state(ZCalibrationImage::UnknownState)
{
    /// connect to each image state changes
    foreach (ZCalibrationImage::Ptr image, m_images) {
        QObject::connect(image.data(), SIGNAL(stateChanged(Z3D::ZCalibrationImage::ImageState)),
                         this, SLOT(updateState()));
    }
    updateState();
}

ZMultiCalibrationImage::~ZMultiCalibrationImage()
{

}

bool ZMultiCalibrationImage::isValid()
{
    foreach (ZCalibrationImage::Ptr image, m_images) {
        if (!image->isValid())
            return false;
    }

    return true;
}

QString ZMultiCalibrationImage::fileName() const
{
    //! FIXME: falta implementar ZMultiCalibrationImage::fileName()
    return QString("FIXME: falta implementar ZMultiCalibrationImage::fileName()");
}

QImage ZMultiCalibrationImage::thumbnail() const
{
    int maxWidth = 0;
    int minHeight = 9999999;
    QImage thumbImage;
    foreach (ZCalibrationImage::Ptr img, m_images) {
        thumbImage = img->imageFromUrl(Z3D::ZCalibrationImage::ThumbnailImage);
        if (maxWidth < thumbImage.width())
            maxWidth = thumbImage.width();
        if (minHeight > thumbImage.height())
            minHeight = thumbImage.height();
    }

    QImage thumbnailImage(maxWidth*m_images.size(), minHeight, thumbImage.format());
    QPainter painter(&thumbnailImage);
    painter.fillRect(thumbnailImage.rect(), Qt::black);
    int lastPos = 0;
    const int separation = 2;
    for (int i=0; i<m_images.size(); ++i) {
        ZCalibrationImage::Ptr img = m_images[i];
        thumbImage = img->imageFromUrl(Z3D::ZCalibrationImage::ThumbnailImage);
        thumbImage = thumbImage.scaledToHeight(minHeight);
        painter.drawImage(lastPos, 0, thumbImage);
        lastPos += thumbImage.width() + separation;
    }
    painter.end();

    return thumbnailImage.copy(0,0,lastPos-separation,minHeight).scaledToWidth(maxWidth);
}

const QList<ZCalibrationImage::Ptr> &ZMultiCalibrationImage::images() const
{
    return m_images;
}

ZCalibrationImage::Ptr ZMultiCalibrationImage::image(int index) const
{
    if (index >= m_images.size())
        return ZCalibrationImage::Ptr(nullptr);

    return m_images[index];
}

ZCalibrationImage::ImageState ZMultiCalibrationImage::state() const
{
    return m_state;
}

void ZMultiCalibrationImage::updateState()
{
    ZCalibrationImage::ImageState worseState = ZCalibrationImage::UnknownState;
    foreach (ZCalibrationImage::Ptr image, m_images) {
        if (image->state() < worseState)
            worseState = image->state();
    }
    setState(worseState);
}

void ZMultiCalibrationImage::setState(ZCalibrationImage::ImageState arg)
{
    if (m_state != arg) {
        m_state = arg;
        emit stateChanged(arg);
    }
}

} // namespace Z3D
