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

#include <QObject>

namespace Z3D
{

class Z3D_ZPOINTCLOUD_SHARED_EXPORT ZPointField
{
    Q_GADGET

    Q_PROPERTY(QString name READ name CONSTANT)
    Q_PROPERTY(unsigned int offset READ offset CONSTANT)
    Q_PROPERTY(PointFieldTypes dataType READ dataType CONSTANT)
    Q_PROPERTY(unsigned int count READ count CONSTANT)

public:
    enum PointFieldTypes {
        INT8,
        UINT8,
        INT16,
        UINT16,
        INT32,
        UINT32,
        FLOAT32,
        FLOAT64
    };
    Q_ENUM(PointFieldTypes)

    static ZPointField* position(unsigned int offset,
                                 PointFieldTypes type,
                                 unsigned int count);
    static ZPointField* normal(unsigned int offset,
                               PointFieldTypes type,
                               unsigned int count);
    static ZPointField* color(unsigned int offset,
                              PointFieldTypes type,
                              unsigned int count);
    static ZPointField* radii(unsigned int offset,
                              PointFieldTypes type,
                              unsigned int count);

    explicit ZPointField(const QString &name,
                         unsigned int offset,
                         PointFieldTypes type,
                         unsigned int count);
    virtual ~ZPointField();

    QString name() const;
    unsigned int offset() const;
    PointFieldTypes dataType() const;
    unsigned int count() const;

private:
    const QString m_name;
    const unsigned int m_offset;
    const PointFieldTypes m_dataType;
    const unsigned int m_count;
};

} // namespace Z3D

QDebug operator<<(QDebug debug, const Z3D::ZPointField *pointField);
