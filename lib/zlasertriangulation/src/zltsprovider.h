#ifndef Z3D_LASERTRIANGULATION___LTSPROVIDER_H
#define Z3D_LASERTRIANGULATION___LTSPROVIDER_H

#include "zlasertriangulation_global.h"
#include "zltscamera3d.h"

#include <QList>

namespace Z3D
{

class Z3D_LASERTRIANGULATION_SHARED_EXPORT LTSProvider
{
public:
    static QList<Z3D::LTSCamera3D::Ptr> loadCameras(QString folder = QString());

private:
    LTSProvider() {}
};

} // namespace Z3D

#endif // Z3D_LASERTRIANGULATION___LTSPROVIDER_H
