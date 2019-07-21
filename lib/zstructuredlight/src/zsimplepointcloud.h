/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "zstructuredlight_global.h"

#include "zpointcloud.h"

#include <opencv2/core/matx.hpp>

namespace Z3D
{

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZSimplePointCloud : public ZPointCloud
{
    Q_OBJECT

public:
    typedef cv::Vec4f PointType;
    typedef std::vector<PointType> PointVector;

    explicit ZSimplePointCloud(const PointVector points, QObject *parent = nullptr);
    ~ZSimplePointCloud() override;

    // ZPointCloud interface
    virtual void updateAttributes() override;
    virtual unsigned int height() const override;
    virtual unsigned int width() const override;
    virtual unsigned int pointStep() const override;
    virtual unsigned int rowStep() const override;
    virtual QByteArray data() const override;
    virtual const std::vector<ZPointField *> &fields() const override;

public:
    const std::vector<ZPointField*> m_fields;

    const PointVector m_points;

    const unsigned int m_width;
    const unsigned int m_height;
};

} // namespace Z3D
