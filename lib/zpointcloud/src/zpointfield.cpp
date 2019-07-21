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

#include "zpointfield.h"

#include <QDebug>
#include <QLoggingCategory>

namespace Z3D
{

namespace // anonymous namespace
{

Q_LOGGING_CATEGORY(loggingCategory, "z3d.zpointcloud.zpointfield", QtInfoMsg)

} // anonymous namespace

ZPointField::ZPointField(QString name, unsigned int offset, PointFieldTypes type, unsigned int count)
    : m_name(name)
    , m_offset(offset)
    , m_dataType(type)
    , m_count(count)
{
    qDebug(loggingCategory) << "creating" << this;
}

ZPointField::~ZPointField()
{
    qDebug(loggingCategory) << "destroying" << this;
}

QString ZPointField::name() const
{
    return m_name;
}

unsigned int ZPointField::offset() const
{
    return m_offset;
}

ZPointField::PointFieldTypes ZPointField::dataType() const
{
    return m_dataType;
}

unsigned int ZPointField::count() const
{
    return m_count;
}

} // namespace Z3D

QDebug operator<< (QDebug debug, const Z3D::ZPointField *pointField)
{
    QDebugStateSaver saver(debug);
    debug.nospace() << "Z3D::ZPointField(" << (void*)(pointField)
                    << ") name: " << pointField->name()
                    << ", offset: " << pointField->offset()
                    << ", type: " << pointField->dataType()
                    << ", count: " << pointField->count();
    return debug;
}
