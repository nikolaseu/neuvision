//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2018 Nicolas Ulrich <nikolaseu@gmail.com>
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

#include "ZPointCloud/zpointcloudreader.h"

#include "ZPointCloud/zpointcloudprovider.h"

#include <QDebug>

namespace Z3D
{

ZPointCloudReader::ZPointCloudReader()
{

}

QString ZPointCloudReader::filename() const
{
    return m_filename;
}

QUrl ZPointCloudReader::source() const
{
    return m_source;
}

ZPointCloud *ZPointCloudReader::pointCloud() const
{
    return m_pointCloud.get();
}

void ZPointCloudReader::setFilename(QString filename)
{
    if (m_filename == filename) {
        return;
    }

    m_pointCloud = ZPointCloudProvider::loadPointCloud(filename);
    m_filename = filename;

    emit filenameChanged(filename);
    emit pointCloudChanged(m_pointCloud.get());
}

void ZPointCloudReader::setSource(QUrl source)
{
    if (m_source == source) {
        return;
    }

    m_source = source;
    emit sourceChanged(m_source);

    setFilename(source.toLocalFile());
}

} // namespace Z3D
