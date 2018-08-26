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

#include "zstructuredlight_fwd.h"
#include "zstructuredlight_global.h"

class QSettings;

namespace Z3D
{

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZStructuredLightSystemPlugin
{
public:
    virtual ~ZStructuredLightSystemPlugin();

    virtual ZStructuredLightSystemPtr get(QSettings *settings) = 0;
};

} // namespace Z3D

#define ZStructuredLightSystemPlugin_iid "z3d.zstructuredlight.zstructuredlightsystemplugin"

Q_DECLARE_INTERFACE(Z3D::ZStructuredLightSystemPlugin, ZStructuredLightSystemPlugin_iid)
