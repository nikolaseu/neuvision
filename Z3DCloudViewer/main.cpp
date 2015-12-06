#include "zapplication.h"
#include "zcloudview.h"

int main(int argc, char *argv[])
{
    Z3D::ZApplication a(argc, argv);

    Z3D::ZCloudView w;
    w.show();

    return a.exec();
}
