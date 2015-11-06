#ifndef Z3D_CAMERAACQUISITION___ZCAMERAIMAGE_H
#define Z3D_CAMERAACQUISITION___ZCAMERAIMAGE_H

#include "zcameraacquisition_global.h"

#include <QSharedPointer>

#include "opencv2/opencv.hpp"

namespace Z3D
{

class Z3D_CAMERAACQUISITION_SHARED_EXPORT ZImageGrayscale
{

public:
    typedef QSharedPointer<Z3D::ZImageGrayscale> Ptr;

    /// create new image
    ZImageGrayscale(int width, int height, int xOffset = 0, int yOffset = 0, int bytesPerPixel = 1);

    /// create new image from image file
    ZImageGrayscale(QString filename);

    /// create new image from externally managed  buffer
    ZImageGrayscale(int width, int height, int xOffset, int yOffset, int bytesPerPixel, void *externalBuffer);

    ~ZImageGrayscale();

    /// creates a complete copy of the instance
    ZImageGrayscale::Ptr clone();

    inline cv::Mat cvMat() const { return m_cvMat; }

    unsigned char *buffer() const;
    bool setBuffer(void *otherBuffer);

    inline long number() const { return m_number; }
    inline void setNumber(long number) { m_number = number; }

    inline int width() const { return m_width; }

    inline int height() const { return m_height; }

    inline int xOffset() const { return m_xOffset; }

    inline int yOffset() const { return m_yOffset;  }

    inline int bytesPerPixel() const { return m_bytesPerPixel; }

    inline int bufferSize() const { return m_width * m_height * m_bytesPerPixel; }

    bool save(QString fileName);

private:
    long m_number;
    int m_width;
    int m_height;
    int m_xOffset;
    int m_yOffset;
    int m_bytesPerPixel;

    cv::Mat m_cvMat;
};

} // namespace Z3D

#endif // Z3D_CAMERAACQUISITION___ZCAMERAIMAGE_H
