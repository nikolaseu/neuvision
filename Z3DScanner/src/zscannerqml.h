#pragma once

#include "zstructuredlight_fwd.h"
#include "zpointcloud_fwd.h"

#include <QObject>

class ZScannerQML : public QObject
{
    Q_OBJECT

    Q_PROPERTY(Z3D::ZPointCloud* cloud READ cloud NOTIFY cloudChanged)

public:
    explicit ZScannerQML(Z3D::ZStructuredLightSystemPtr structuredLightSystem, QObject *parent = nullptr);

    Z3D::ZPointCloud* cloud() const;

signals:
    void cloudChanged(Z3D::ZPointCloud* cloud);

public slots:
    void scan();
    Q_DECL_DEPRECATED void openPatternsDialog() const;
    Q_DECL_DEPRECATED void openStructuredLightSystemDialog() const;

private slots:
    void setCloud(Z3D::ZPointCloudPtr cloud);

private:
    const Z3D::ZStructuredLightSystemPtr m_structuredLightSystem;

    Z3D::ZPointCloudPtr m_cloud;
};
