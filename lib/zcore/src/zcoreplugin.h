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

#include "zcore_global.h"

#include <QObject>

namespace Z3D
{

class Z3D_CORE_SHARED_EXPORT ZCorePlugin : public QObject
{
    Q_OBJECT

public:
    explicit ZCorePlugin(QObject *parent = nullptr);
    virtual ~ZCorePlugin() {}

    /// plugin information
    virtual QString id() const = 0;
    virtual QString name() const = 0;
    virtual QString version() const = 0;

signals:

public slots:
};

} // namespace Z3D
