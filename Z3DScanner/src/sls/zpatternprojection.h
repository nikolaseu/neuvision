#ifndef ZPATTERNPROJECTION_H
#define ZPATTERNPROJECTION_H

#include "zcameraimage.h"
#include "zdecodedpattern.h"

#include <QObject>

namespace Z3D
{

class ZPatternProjection : public QObject
{
    Q_OBJECT

public:
    explicit ZPatternProjection(QObject *parent = 0);

signals:
    void prepareAcquisition(QString acquisitionId);
    void acquireSingle(QString id);
    void finishAcquisition();

    void patternDecoded(Z3D::ZDecodedPattern::Ptr pattern);

public slots:
    virtual QWidget *configWidget() = 0;
    virtual void beginScan() = 0;
    virtual void processImages(std::vector< std::vector<Z3D::ZImageGrayscale::Ptr> > acquiredImages, QString acquisitionId) = 0;
};

} // namespace Z3D

#endif // ZPATTERNPROJECTION_H
