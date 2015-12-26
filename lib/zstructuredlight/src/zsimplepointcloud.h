#ifndef ZSIMPLEPOINTCLOUD_H
#define ZSIMPLEPOINTCLOUD_H

#include "zstructuredlight_global.h"
#include "opencv2/core/types.hpp"

#include <QObject>
#include <QSharedPointer>

namespace Z3D
{

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZSimplePointCloud : public QObject
{
    Q_OBJECT

public:
    typedef cv::Vec4f PointType;
    typedef std::vector<PointType> PointVector;
    typedef QSharedPointer<Z3D::ZSimplePointCloud> Ptr;

    explicit ZSimplePointCloud(QObject *parent = 0);

signals:

public slots:

public:
    PointVector points;
    int width;
    int height;
};

} // namespace Z3D

#endif // ZSIMPLEPOINTCLOUD_H
