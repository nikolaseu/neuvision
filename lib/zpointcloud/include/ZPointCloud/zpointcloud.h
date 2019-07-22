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

class ZPointField;

class Z3D_ZPOINTCLOUD_SHARED_EXPORT ZPointCloud : public QObject
{
    Q_OBJECT
    Q_PROPERTY(unsigned int height READ height CONSTANT)
    Q_PROPERTY(unsigned int width READ width CONSTANT)
    Q_PROPERTY(unsigned int pointStep READ pointStep CONSTANT)
    Q_PROPERTY(unsigned int rowStep READ rowStep CONSTANT)
    Q_PROPERTY(QByteArray data READ data CONSTANT)

public:
    explicit ZPointCloud(QObject *parent = nullptr);
    virtual ~ZPointCloud() override;

    virtual void updateAttributes() = 0;

    virtual unsigned int height() const = 0;
    virtual unsigned int width() const = 0;

    virtual unsigned int pointStep() const = 0;
    virtual unsigned int rowStep() const = 0;
    virtual QByteArray data() const = 0;

    //! IMPORTANT: returns a reference, subclasses must not return temporaries!
    virtual const std::vector<ZPointField *> &fields() const = 0;
};

} // namespace Z3D
