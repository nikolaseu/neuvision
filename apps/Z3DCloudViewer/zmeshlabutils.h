#pragma once

#include <QMatrix4x4>
#include <QString>

namespace Z3D
{

namespace ZMeshlabUtils
{

struct MeshlabProjectData {
    QString filename;
    QMatrix4x4 transformation;
};

std::vector<MeshlabProjectData> parseMeshlabProject(const QString &fileName);

} // namespace ZMeshlabUtils

} // namespace Z3D
