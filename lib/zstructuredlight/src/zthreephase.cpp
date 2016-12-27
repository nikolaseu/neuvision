//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "zthreephase.h"
#include "math.h"

#include <iostream>

#include <QDebug>

#include <opencv2/highgui/highgui.hpp> // debug

float max3(float a, float b, float c) {
    if (a > b) {
        return a > c ? a : c;
    } else {
        return b > c ? b : c;
    }
}

float min3(float a, float b, float c) {
    if (a < b) {
        return a < c ? a : c;
    } else {
        return b < c ? b : c;
    }
}

float diff(float a, float b) {
    float d = a < b ? b-a : a-b;
    return d < .5 ? d : 1-d;
}

#define min(a,b) ((a) < (b) ? (a) : (b))
#define max(a,b) ((a) > (b) ? (a) : (b))
#define clamp(a, low, high) ((a > low) ? (a < high ? a : high) : low)

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Returns the appropriate color from the "jet" colormap
///
/// Implementing a Continuous "Jet" Colormap Function in GLSL
/// http://www.metastine.com/?p=7
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void setColor(unsigned char* pix, float value)
{
    //    if (value < 0 || value > 1) {
    //        qWarning() << "¿por que no esta entre 0 y 1? " << value;
    //    }

    float valuex4 = value * 4.f;
    float r = min(valuex4 - 1.5f, -valuex4 + 4.5f);
    float g = min(valuex4 - 0.5f, -valuex4 + 3.5f);
    float b = min(valuex4 + 0.5f, -valuex4 + 2.5f);

    pix[0] = (unsigned char) 255.f * clamp(r, 0.f, 1.f); //R
    pix[1] = (unsigned char) 255.f * clamp(g, 0.f, 1.f); //G
    pix[2] = (unsigned char) 255.f * clamp(b, 0.f, 1.f); //B
}



ThreePhase::ThreePhase(unsigned int width, unsigned int height) :
    inputWidth(width),
    inputHeight(height)
  //    point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>)
{
    renderDetail = 1;
    zskew = 64;
    zscale = 100;
    noiseThreshold = 10;

    unsigned int size = inputWidth * inputHeight;
    printf("Initializing for %d x %d = %d\n", inputWidth, inputHeight, size);
    mask     = new bool [size];
    process  = new bool [size];
    distance = new float[size];
    phase    = new float[size];
    depth    = new float[size];
    colors   = new float[size];

    wrappedPhase   = new unsigned char[3*size];
    unwrappedPhase = new unsigned char[3*size];
    colorPositionImage = new unsigned char[3*size];

    //    for (unsigned int i=0; i<size; ++i) {
    //        pcl::PointXYZI point;
    //        point_cloud_ptr->points.push_back(point);
    //    }
    //    point_cloud_ptr->width = (int) point_cloud_ptr->points.size ();
    //    point_cloud_ptr->height = 1;


    //    /** intrinsic camera parameters from calibration with checkerboard */
    //    m_camIntrinsicParam <<  1, 0, 0,
    //                            0, 1, 0,
    //                            0, 0, 1;

    //    /** camera extrinsic parameters
    //     * (world coordinate transformation */
    //    m_camExtrinsicParam <<  1, 0, 0, 0,
    //                            0, 1, 0, 0,
    //                            0, 0, 1, 0;

    //    /** intrinsic projector parameters from calibration */
    //    m_dlpIntrinsicParam <<  1, 0, 0,
    //                            0, 1, 0,
    //                            0, 0, 1;

    //    /** projector extrinsic parameters
    //     * (world coordinate transformation */
    //    m_dlpExtrinsicParam <<  1, 0, 0, 0,
    //                            0, 1, 0, 0,
    //                            0, 0, 1, 0;

    //    m_camMatrix = m_camIntrinsicParam * m_camExtrinsicParam;
    //    //std::cout << m_camMatrix;

    //    m_dlpMatrix = m_dlpIntrinsicParam * m_dlpExtrinsicParam;
    //    //std::cout << m_dlpMatrix;
}

ThreePhase::~ThreePhase()
{
    delete[] mask;
    delete[] process;
    delete[] distance;
    delete[] phase;
    delete[] depth;
    delete[] colors;

    delete[] wrappedPhase;
    delete[] unwrappedPhase;
    delete[] colorPositionImage;
}

void ThreePhase::calculateDepth(cv::Mat data1, cv::Mat data2, cv::Mat data3)
{
    printf( "Calculating depth...\n");

    phaseWrap(data1, data2, data3);
    phaseUnwrap();
    if (doMakeDepth)
        makeDepth();
    printf( "Finished depth calculation...\n");
}

void ThreePhase::phaseWrap(cv::Mat data1, cv::Mat data2, cv::Mat data3)
{
    const float sqrt3 = sqrtf(3.f);
    const float two_pi = 2.f * M_PI;

    for (unsigned int y = 0; y < inputHeight; y++) {
        for (unsigned int x = 0; x < inputWidth; x++) {
            int i = index(x,y);

            float phase1 = data1.at<unsigned char>(y, x);
            float phase2 = data2.at<unsigned char>(y, x);
            float phase3 = data3.at<unsigned char>(y, x);

            float phaseRange = max3(phase1, phase2, phase3) - min3(phase1, phase2, phase3);

            mask[i]     = phaseRange <= noiseThreshold;
            process[i]  = !mask[i];
            distance[i] = phaseRange;

            // this equation can be found in Song Zhang's
            // "Recent progresses on real-time 3D shape measurement..."
            // and it is the "bottleneck" of the algorithm
            // it can be sped up with a look up table, which has the benefit
            // of allowing for simultaneous gamma correction.
            phase[i] = atan2(sqrt3 * (phase1 - phase3), 2 * phase2 - phase1 - phase3) / two_pi;

            // build color based on the lightest channels from all three images
            colors[i] = max3(phase1, phase2, phase3);

            if (!mask[i]) {
                // Este queda muy colorido, no ayuda y destruye ojos
                //setColor(&wrappedPhase[3*i], phase[i] + 0.5f);
                // Usamos escala de grises mejor
                wrappedPhase[3*i  ] = wrappedPhase[3*i+1] = wrappedPhase[3*i+2] = 255 * phase[i] + 0.5f;
            } else {
                wrappedPhase[3*i  ] = 0; //black
                wrappedPhase[3*i+1] = 0;
                wrappedPhase[3*i+2] = 0;
            }

            /*
        if (x==1)
            qDebug() << colors[i] << phase1 << phase2 << phase3;
  */

            //      // point cloud
            //      pcl::PointXYZI &point = point_cloud_ptr->points[i];
            //      if (process[i]) {
            //          point.x = x;
            //          point.y = y;
            //          point.z = 0;
            //      } else {
            //          point.x = 0;
            //          point.y = 0;
            //          point.z = 0;
            //      }

            //      if (doMakeDepth)
            //        point.intensity = colors[i];
            //      else
            //        point.intensity = phase[i];
        }
    }

    for (unsigned int y = 1; y < inputHeight - 1; y++) {
        for (unsigned int x = 1; x < inputWidth - 1; x++) {
            int i = index(x, y);
            if (!mask[i]) {
                distance[i] = (
                            diff(phase[i], phase[index(x,y - 1)]) +
                        diff(phase[i], phase[index(x,y + 1)]) +
                        diff(phase[i], phase[index(x - 1,y)]) +
                        diff(phase[i], phase[index(x + 1,y)])) / distance[i];
            }
        }
    }
}

void ThreePhase::phaseUnwrap() {
    int startX = inputWidth / 2;
    int startY = inputHeight / 2;
    toProcess.insert(WrappedPixel(startX, startY, 0, phase[index(startX, startY)]));

    float maxPhase = -9e9f;
    float minPhase =  9e9f;

    while(!toProcess.empty()) {
        WrappedPixel cur = *(toProcess.begin());
        toProcess.erase(toProcess.begin());
        unsigned int x = cur.x;
        unsigned int y = cur.y;
        unsigned int i = index(x,y);
        if(process[i]) {
            phase[i] = cur.phase;

            if (phase[i] < minPhase)
                minPhase = phase[i];
            if (phase[i] > maxPhase)
                maxPhase = phase[i];

            process[i] = false;
            float d = cur.distance;
            float r = phase[i];
            if (y > 0)
                phaseUnwrap(x, y-1, d, r);
            if (y < inputHeight-1)
                phaseUnwrap(x, y+1, d, r);
            if (x > 0)
                phaseUnwrap(x-1, y, d, r);
            if (x < inputWidth-1)
                phaseUnwrap(x+1, y, d, r);
        }
    }

    //
    float phaseRange = maxPhase - minPhase;

    qDebug() << "phase min:" << minPhase << "max:" << maxPhase << "=> range:" << phaseRange;

    for (unsigned int y = 0; y < inputHeight; y++) {
        for (unsigned int x = 0; x < inputWidth; x++) {
            int i = index(x, y);
            if (!mask[i]) {
                setColor(&unwrappedPhase[3*i], ((phase[i] - minPhase) / phaseRange));
            } else {
                unwrappedPhase[3*i  ] = 0; //black
                unwrappedPhase[3*i+1] = unwrappedPhase[3*i];
                unwrappedPhase[3*i+2] = unwrappedPhase[3*i];
            }
        }
    }
}

void ThreePhase::phaseUnwrap(int x, int y, float d, float r) {
    int i = index(x,y);
    if(process[i]) {
        float diff = phase[i] - (r - (int) r);
        if (diff > .5)
            diff--;
        if (diff < -.5)
            diff++;
        toProcess.insert(WrappedPixel(x, y, d + distance[i], r + diff));
    }
}

void ThreePhase::makeDepth() {/*
    Eigen::Matrix3f eqMatrix;
    eqMatrix(0,0) = m_camMatrix(0,0);
    eqMatrix(0,1) = m_camMatrix(0,1);
    eqMatrix(0,2) = m_camMatrix(0,2);
    eqMatrix(1,0) = m_camMatrix(1,0);
    eqMatrix(1,1) = m_camMatrix(1,1);
    eqMatrix(1,2) = m_camMatrix(1,2);
    eqMatrix(2,0) = m_dlpMatrix(1,0);
    eqMatrix(2,1) = m_dlpMatrix(1,1);
    eqMatrix(2,2) = m_dlpMatrix(1,2);

    Eigen::Matrix3f invMatrix = eqMatrix.inverse();
    Eigen::Vector3f t;
    t << m_camMatrix(0,3),
         m_camMatrix(1,3),
         m_dlpMatrix(1,3);

    for (unsigned int x = 0; x < inputWidth; x++) {
        for (unsigned int y = 0; y < inputHeight; y++) {
            int i = index(x,y);
            if (!mask[i]) {
                Eigen::Vector3f v;
                v << x,
                     y,
                     phase[i];
                Eigen::Vector3f coord = invMatrix * (v + t);
                depth[i] = coord(2);

                std::cout << coord << std::endl;

                colorPositionImage[3*i  ] = coord(0);
                colorPositionImage[3*i+1] = coord(1);
                colorPositionImage[3*i+2] = coord(2);
            } else {
                depth[i] = -999;
            }
        }
    }

    /*
    for (unsigned int x = 0; x < inputWidth; x += renderDetail) {
        float planephase = 0.5 - (x - (inputWidth / 2)) / zskew;
        //float planephase = ((inputWidth / 2) - x) / zskew;
        //float planephase = inputWidth - x / zskew;
        for (unsigned int y = 0; y < inputHeight; y += renderDetail) {
            int i = index(x,y);
            if (!mask[i]) {
                depth[i] = (phase[i] - planephase) * zscale;
            } else {
                depth[i] = -999;
            }
//            pcl::PointXYZI &point = point_cloud_ptr->points[i];
//            point.z = depth[i];
        }
    }
    */
}

unsigned int ThreePhase::index(unsigned int x, unsigned int y)
{
    return (x + y * inputWidth);
}
