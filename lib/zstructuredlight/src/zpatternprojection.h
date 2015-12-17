#ifndef Z3D_STRUCTUREDLIGHT___ZPATTERNPROJECTION_H
#define Z3D_STRUCTUREDLIGHT___ZPATTERNPROJECTION_H

#include "zstructuredlight_global.h"
#include "zcameraimage.h"
#include "zdecodedpattern.h"

#include <QObject>

namespace Z3D
{

class Z3D_STRUCTUREDLIGHT_SHARED_EXPORT ZPatternProjection : public QObject
{
    Q_OBJECT

public:
    explicit ZPatternProjection(QObject *parent = 0);

    virtual QString displayName() = 0;

signals:
    void prepareAcquisition(QString acquisitionId);
    void acquireSingle(QString id);
    void finishAcquisition();

    void patternsDecoded(std::vector<Z3D::ZDecodedPattern::Ptr> pattern);

public slots:
    virtual QWidget *configWidget() = 0;
    virtual void beginScan() = 0;
    virtual void processImages(std::vector< std::vector<Z3D::ZImageGrayscale::Ptr> > acquiredImages, QString acquisitionId) = 0;
};

} // namespace Z3D

#endif // Z3D_STRUCTUREDLIGHT___ZPATTERNPROJECTION_H
