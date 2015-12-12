#ifndef ZSIMPLEPOINTCLOUD_H
#define ZSIMPLEPOINTCLOUD_H

#include "opencv2/core/types.hpp"

#include <QObject>

typedef cv::Vec4f PointType;

namespace Z3D
{

class ZSimplePointCloud : public QObject
{
    Q_OBJECT

public:
    typedef QSharedPointer<Z3D::ZSimplePointCloud> Ptr;

    explicit ZSimplePointCloud(QObject *parent = 0);

signals:

public slots:

public:
    std::vector<PointType> points;
    int width;
    int height;
};

} // namespace Z3D

#endif // ZSIMPLEPOINTCLOUD_H
