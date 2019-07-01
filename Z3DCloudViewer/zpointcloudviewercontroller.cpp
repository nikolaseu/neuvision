#include "zpointcloudviewercontroller.h"

#include "zpointcloudlistitem.h"
#include "zpointcloudlistmodel.h"
#include "zpointcloudprovider.h"
#include "zmeshlabutils.h"

#include <QFileInfo>
#include <QLoggingCategory>
#include <QtConcurrentRun>
#include <QUrl>

namespace Z3D
{

namespace // anonymous namespace
{
Q_LOGGING_CATEGORY(loggingCategory, "z3d.zcloudviewer.zpointcloudviewercontroller", QtInfoMsg);
}

ZPointCloudViewerController::ZPointCloudViewerController(QObject *parent)
    : QObject(parent)
    , m_model(new ZPointCloudListModel(this))
{

}

ZPointCloudViewerController::~ZPointCloudViewerController()
{
    qDebug(loggingCategory) << "destroying" << this;
}

ZPointCloudListModel *ZPointCloudViewerController::model() const
{
    return m_model;
}

void ZPointCloudViewerController::loadFile(const QUrl &fileUrl)
{
    qInfo(loggingCategory) << "trying to load file" << fileUrl;

    QtConcurrent::run([=](){
        const QFileInfo fileInfo(fileUrl.toLocalFile());

        emit message(QString("loading %1 ...").arg(fileInfo.fileName()));

        if (fileInfo.suffix().compare("mlp", Qt::CaseInsensitive) == 0) {
            const auto meshlabProjectFiles = Z3D::ZMeshlabUtils::parseMeshlabProject(fileInfo.absoluteFilePath());
            for (size_t i=0; i<meshlabProjectFiles.size(); ++i) {
                const auto &projectFile = meshlabProjectFiles[i];

                emit message(QString("loading %1 (%2 of %3) ...")
                                 .arg(projectFile.filename)
                                 .arg(i)
                                 .arg(meshlabProjectFiles.size()));

                if (auto pc = ZPointCloudProvider::loadPointCloud(projectFile.filename)) {
                    const QFileInfo projectFileInfo(projectFile.filename);
                    auto item = new ZPointCloudListItem(pc, projectFileInfo.fileName());
                    item->setTransformation(projectFile.transformation);
                    m_model->addPointCloud(item);
                } else {
                    qWarning(loggingCategory) << "failed to load file inside the project:" << projectFile.filename;
                }
            }

            emit message(QString("finished loading %1").arg(fileInfo.fileName()));
            emit fileLoaded(fileUrl);
            return;
        }

        if (auto pc = ZPointCloudProvider::loadPointCloud(fileInfo.absoluteFilePath())) {
            auto item = new ZPointCloudListItem(pc, fileInfo.fileName());
            m_model->addPointCloud(item);

            emit message(QString("finished loading %1").arg(fileInfo.fileName()));
            emit fileLoaded(fileUrl);
            return;
        }
    });
}

} // namespace Z3D
