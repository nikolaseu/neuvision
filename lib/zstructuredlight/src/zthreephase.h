#pragma once

#include <set>

#include <opencv2/core/core.hpp>

//#include <pcl/common/common_headers.h>

struct WrappedPixel {
    unsigned int x, y;
    float distance, phase;
    WrappedPixel(unsigned int x, unsigned int y, float distance, float phase) {
        this->x = x;
        this->y = y;
        this->distance = distance;
        this->phase = phase;
    }
    bool operator<(const WrappedPixel& w) const {
        return w.distance < distance;
    }
};

class ThreePhase
{
public:
    ThreePhase(unsigned int width, unsigned int height);
    ~ThreePhase();

    void calculateDepth(cv::Mat data1, cv::Mat data2, cv::Mat data3);

    //! FIXME: que no sea publico!
    //    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_ptr;
    unsigned char* wrappedPhase;
    unsigned char* unwrappedPhase;
    unsigned char* colorPositionImage;

    //public slots:
    void setWidth(unsigned int width) { inputWidth = width; }
    void setHeight(unsigned int height) { inputHeight = height; }
    void setZSkew(float zskew) { this->zskew = zskew; }
    void setZScale(float zscale) { this->zscale = zscale; }
    void setNoiseThreshold(float noiseThreshold) { this->noiseThreshold = noiseThreshold; }
    void setMakeDepth(bool enable) { this->doMakeDepth = enable; }

private:
    void phaseWrap(cv::Mat data1, cv::Mat data2, cv::Mat data3);
    void phaseUnwrap();
    void phaseUnwrap(int x, int y, float d, float r);
    void makeDepth();
    unsigned int index(unsigned int x, unsigned int y);

    unsigned int inputWidth;
    unsigned int inputHeight;

    unsigned int renderDetail;
    float zskew;
    float zscale;
    float noiseThreshold;
    bool doMakeDepth;

    bool *mask;
    bool *process;
    float *distance;
    float *phase;
    float *depth;
    float *colors;

    //unsigned int bitsPerPixel;

    std::set<WrappedPixel> toProcess;

    //    Eigen::Matrix<float, 3, 3> m_camIntrinsicParam;
    //    Eigen::Matrix<float, 3, 4> m_camExtrinsicParam;
    //    Eigen::Matrix<float, 3, 4> m_camMatrix;

    //    Eigen::Matrix<float, 3, 3> m_dlpIntrinsicParam;
    //    Eigen::Matrix<float, 3, 4> m_dlpExtrinsicParam;
    //    Eigen::Matrix<float, 3, 4> m_dlpMatrix;
};
