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

#include "ZStructuredLight/zstructuredlight_global.h"

#include "ZPointCloud/zpointcloud.h"

#include <opencv2/core/matx.hpp>

namespace Z3D
{

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZSimplePointCloud : public ZPointCloud
{
    Q_OBJECT

public:
    using PointType = cv::Vec<float, 8>; // X Y Z NX NY NZ RGBX radii
    using PointVector = std::vector<PointType>;

    explicit ZSimplePointCloud(const PointVector points, QVector<unsigned int> indices = {}, QObject *parent = nullptr);
    ~ZSimplePointCloud() override;

    // ZPointCloud interface
    QByteArray vertexData() const override Q_DECL_FINAL;
    QByteArray trianglesData() const override Q_DECL_FINAL;

private:
    const PointVector m_points;
    QVector<unsigned int> m_indices;
};

} // namespace Z3D
