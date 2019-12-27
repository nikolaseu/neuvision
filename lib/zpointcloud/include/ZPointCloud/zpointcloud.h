/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2018 Nicolas Ulrich <nikolaseu@gmail.com>
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

#include "ZPointCloud/zpointcloud_global.h"
#include "ZPointCloud/zpointcloud_fwd.h"

#include <QObject>
#include <QVector>

namespace Z3D
{

class ZPointField;

class Z3D_ZPOINTCLOUD_SHARED_EXPORT ZPointCloud : public QObject
{
    Q_OBJECT
    Q_PROPERTY(unsigned int height READ height CONSTANT)
    Q_PROPERTY(unsigned int width READ width CONSTANT)
    Q_PROPERTY(unsigned int pointStep READ pointStep CONSTANT)
    Q_PROPERTY(unsigned int rowStep READ rowStep CONSTANT)
    Q_PROPERTY(QByteArray vertexData READ vertexData CONSTANT)
    Q_PROPERTY(QByteArray trianglesData READ trianglesData CONSTANT)

    Q_PROPERTY(bool hasNormals READ hasNormals CONSTANT)
    Q_PROPERTY(bool hasColors READ hasColors CONSTANT)
    Q_PROPERTY(bool hasRadii READ hasRadii CONSTANT)
    Q_PROPERTY(bool hasTriangles READ hasTriangles CONSTANT)

public:
    explicit ZPointCloud(unsigned int height,
                         unsigned int width,
                         unsigned int pointStep,
                         unsigned int rowStep,
                         const std::vector<ZPointField *> fields,
                         QObject *parent = nullptr);

    virtual ~ZPointCloud() override;

    inline unsigned int height() const { return m_height; }
    inline unsigned int width() const { return m_width; }
    inline unsigned int pointStep() const { return m_pointStep; }
    inline unsigned int rowStep() const { return m_rowStep; }
    inline const std::vector<ZPointField *> &fields() const { return m_fields; }

    unsigned int vertexCount() const;
    unsigned int trianglesCount() const;

    virtual QByteArray vertexData() const = 0;
    virtual QByteArray trianglesData() const = 0;

    bool hasNormals() const;
    bool hasColors() const;
    bool hasRadii() const;
    bool hasTriangles() const;

private:
    const unsigned int m_height;
    const unsigned int m_width;
    const unsigned int m_pointStep;
    const unsigned int m_rowStep;
    std::vector<ZPointField *> m_fields;
};

} // namespace Z3D
