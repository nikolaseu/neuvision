#include "zmeshlabutils.h"

#include <QDir>
#include <QDomDocument>
#include <QFile>
#include <QFileInfo>
#include <QLoggingCategory>
#include <QRegularExpression>

namespace Z3D
{

namespace ZMeshlabUtils
{

namespace // anonymous namespace
{
Q_LOGGING_CATEGORY(loggingCategory, "z3d.zcloudviewer.zmeshlabutils", QtInfoMsg);
}

std::vector<MeshlabProjectData> parseMeshlabProject(const QString &fileName)
{
    QFile file(fileName);
    if (!file.open(QIODevice::ReadOnly)) {
        //open error
        qCritical(loggingCategory) << "cannot open file" << fileName;
        throw std::runtime_error("failed to open file");
    }

    QDomDocument doc;
    QString errorStr;
    int errorLine;
    int errorColumn;
    if (!doc.setContent(&file, false, &errorStr, &errorLine, &errorColumn)) {
        file.close();
        qCritical(loggingCategory) << "cannot parse file as XML" << fileName;
        throw std::runtime_error("cannot parse file as XML");
    }

    //parse OK
    file.close();

    QDomElement docRoot = doc.documentElement();
    if (docRoot.isNull()) {
        qCritical(loggingCategory) << "file has no XML root element" << fileName;
        throw std::runtime_error("file has no root element");
    }

    QDomElement meshGroupElem = docRoot.firstChildElement("MeshGroup");
    if (meshGroupElem.isNull()) {
        qCritical(loggingCategory) << "file has no MeshGroup element" << fileName;
        throw std::runtime_error("file has no MeshGroup element");
    }

    const QFileInfo fileInfo(fileName);
    const QDir projectDir(fileInfo.absolutePath());
    std::vector<MeshlabProjectData> data;
    for (QDomElement mesh = meshGroupElem.firstChildElement("MLMesh"); !mesh.isNull(); mesh = mesh.nextSiblingElement("MLMesh")) {
        const QString mlMeshFileName = mesh.attribute("filename");

        const QDomElement matrixElem = mesh.firstChildElement("MLMatrix44");
        qDebug(loggingCategory) << "matrix:" << matrixElem.text();

        const QString matrixText = matrixElem.text()
                                       .replace("\n", "")
                                       .replace("\r", "")
                                       .replace("\t", "");

        static QRegularExpression matrixRegExp(R"(([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+) ([-+]?[0-9]*\.?[0-9]+))");
        QRegularExpressionMatch matrixMatch = matrixRegExp.match(matrixText);
        if (!matrixMatch.hasMatch()) {
            qCritical(loggingCategory) << "MLMesh has invalid MLMatrix44" << matrixText;
            throw std::runtime_error("MLMesh has invalid MLMatrix44");
        }

        QMatrix4x4 transformation(
            matrixMatch.captured(1).toDouble(),
            matrixMatch.captured(2).toDouble(),
            matrixMatch.captured(3).toDouble(),
            matrixMatch.captured(4).toDouble(),
            matrixMatch.captured(5).toDouble(),
            matrixMatch.captured(6).toDouble(),
            matrixMatch.captured(7).toDouble(),
            matrixMatch.captured(8).toDouble(),
            matrixMatch.captured(9).toDouble(),
            matrixMatch.captured(10).toDouble(),
            matrixMatch.captured(11).toDouble(),
            matrixMatch.captured(12).toDouble(),
            matrixMatch.captured(13).toDouble(),
            matrixMatch.captured(14).toDouble(),
            matrixMatch.captured(15).toDouble(),
            matrixMatch.captured(16).toDouble());

        MeshlabProjectData meshData;
        meshData.filename = projectDir.exists(mlMeshFileName)
            ? projectDir.absoluteFilePath(mlMeshFileName) // it's a relative file inside the project folder
            : mlMeshFileName; // use the file name as it is
        meshData.transformation = transformation;

        qDebug(loggingCategory) << "found file:" << meshData.filename
                                << "with transformation:" << meshData.transformation;

        data.push_back(meshData);
    }

    return data;
}

} // namespace ZMeshlabUtils

} // namespace Z3D
