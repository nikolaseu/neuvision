#pragma once

#include <QObject>

namespace Z3D
{

class ZPointCloudListModel;

class ZPointCloudViewerController : public QObject
{
    Q_OBJECT

    Q_PROPERTY(Z3D::ZPointCloudListModel* model READ model CONSTANT)

public:
    explicit ZPointCloudViewerController(QObject *parent = nullptr);
    ~ZPointCloudViewerController() override;

    Z3D::ZPointCloudListModel *model() const;

signals:
    void fileLoaded(const QUrl &fileUrl);
    void message(const QString &message);
    void error(const QString &error);

public slots:
    void loadFile(const QUrl &fileUrl);

private:
    Z3D::ZPointCloudListModel *m_model;
};

} // namespace Z3D
