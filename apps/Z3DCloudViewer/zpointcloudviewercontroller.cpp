#include "zpointcloudviewercontroller.h"

#include "ZPointCloud/zpointcloud.h"
#include "ZPointCloud/zpointcloudlistitem.h"
#include "ZPointCloud/zpointcloudlistmodel.h"
#include "ZPointCloud/zpointcloudprovider.h"
#include "zmeshlabutils.h"

#include <QFileInfo>
#include <QLoggingCategory>
#include <QUrl>
#include <QtConcurrentRun>

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

            const size_t INSERT_BATCH_SIZE = 20;
            std::vector<ZPointCloudListItem *> items;
            items.reserve(INSERT_BATCH_SIZE);

            for (size_t i=0; i<meshlabProjectFiles.size(); ++i) {
                const auto &projectFile = meshlabProjectFiles[i];

                emit message(QString("loading %1 (%2 of %3) ...")
                                 .arg(projectFile.filename)
                                 .arg(i)
                                 .arg(meshlabProjectFiles.size()));

                if (auto pc = ZPointCloudProvider::loadPointCloud(projectFile.filename)) {
                    const QFileInfo projectFileInfo(projectFile.filename);
                    auto item = new ZPointCloudListItem(std::move(pc), projectFileInfo.fileName());
                    item->setTransformation(projectFile.transformation);
                    items.push_back(item);
                } else {
                    qWarning(loggingCategory) << "failed to load file inside the project:" << projectFile.filename;
                }

                if (i == 0 || items.size() >= INSERT_BATCH_SIZE) {
                    qDebug(loggingCategory) << "inserting batch of items";
                    m_model->addPointClouds(items);
                    items.clear();
                }
            }

            m_model->addPointClouds(items);

            emit message(QString("finished loading %1").arg(fileInfo.fileName()));
            emit fileLoaded(fileUrl);
            return;
        }

        if (auto pc = ZPointCloudProvider::loadPointCloud(fileInfo.absoluteFilePath())) {
            auto item = new ZPointCloudListItem(std::move(pc), fileInfo.fileName());
            m_model->addPointCloud(item);

            emit message(QString("finished loading %1").arg(fileInfo.fileName()));
            emit fileLoaded(fileUrl);
            return;
        }
    });
}

} // namespace Z3D
