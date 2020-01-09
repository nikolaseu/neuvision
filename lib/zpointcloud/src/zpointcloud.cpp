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

#include "ZPointCloud/zpointcloud.h"

#include "ZPointCloud/zpointfield.h"

#include <QDebug>
#include <QLoggingCategory>

namespace Z3D {

namespace // anonymous namespace
{
Q_LOGGING_CATEGORY(loggingCategory, "z3d.zpointcloud.zpointcloud", QtInfoMsg)
} // anonymous namespace

ZPointCloud::ZPointCloud(unsigned int height,
                         unsigned int width,
                         unsigned int pointStep,
                         unsigned int rowStep,
                         const std::vector<ZPointField *> &fields,
                         QObject *parent)
    : QObject(parent)
    , m_height(height)
    , m_width(width)
    , m_pointStep(pointStep)
    , m_rowStep(rowStep)
    , m_fields(fields)
{
    qDebug(loggingCategory) << "creating point cloud with fields:";
    for (const auto *field : m_fields) {
        qDebug(loggingCategory) << field;
    }

    // check for non-standard fields and add the ones we handle, when possible
    auto findField = [this](const QString &name) -> ZPointField* {
        const auto it = std::find_if(m_fields.cbegin(), m_fields.cend(), [&name](const auto &field){
            return field->name() == name;
        });
        return it != m_fields.cend()
                   ? *it
                   : nullptr;
    };

    if (const auto *positionX = findField("x")) {
        // found non-standard field x,check if we can handle position directly
        const auto *positionY = findField("y");
        const auto *positionZ = findField("z");
        if (positionY && positionZ
            && positionY->offset() - positionX->offset() == 4
            && positionZ->offset() - positionX->offset() == 8)
        {
            auto field = ZPointField::position(positionX->offset(), positionX->dataType(), 3);
            qDebug(loggingCategory) << "adding new field from individual x y z attributes:" << field;
            m_fields.push_back(field);
        }
    }

    if (const auto *normalX = findField("normal_x")) {
        // found non-standard field x,check if we can handle position directly
        const auto *normalY = findField("normal_y");
        const auto *normalZ = findField("normal_z");
        if (normalY && normalZ
            && normalY->offset() - normalX->offset() == 4
            && normalZ->offset() - normalX->offset() == 8)
        {
            auto field = ZPointField::normal(normalX->offset(), normalX->dataType(), 3);
            qDebug(loggingCategory) << "adding new field from individual normal_x normal_y normal_z attributes:" << field;
            m_fields.push_back(field);
        }
    }
}

ZPointCloud::~ZPointCloud()
{
    for (auto field : m_fields) {
        delete field;
    }
}

unsigned int ZPointCloud::vertexCount() const
{
    return m_width * m_height;
}

unsigned int ZPointCloud::trianglesCount() const
{
    return trianglesData().size() / (3 * sizeof(float));
}

bool ZPointCloud::hasNormals() const
{
    return m_fields.cend()
           != std::find_if(m_fields.cbegin(), m_fields.cend(), [](const auto &field) {
                  return field->name() == "normal";
              });
}

bool ZPointCloud::hasColors() const
{
    return m_fields.cend()
           != std::find_if(m_fields.cbegin(), m_fields.cend(), [](const auto &field) {
                  return field->name() == "rgb";
              });
}

bool ZPointCloud::hasRadii() const
{
    return m_fields.cend()
           != std::find_if(m_fields.cbegin(), m_fields.cend(), [](const auto &field) {
                  return field->name() == "radii";
              });
}

bool ZPointCloud::hasTriangles() const
{
    return trianglesCount() > 0;
}

} // namespace Z3D
