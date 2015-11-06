#include "binarypatternprojection.h"

#include "binarypatternprojectionconfigwidget.h"

#include <Z3DCameraAcquisition>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream> // std::cout

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QDebug>
#include <QSignalMapper>
#include <QThread>




#if QT_VERSION < 0x050000
#include <QDeclarativeView>
#include <QDeclarativeContext>
#include <QDeclarativeEngine>
#include <QtOpenGL>

class SleeperThread : public QThread
{
public:
    static void msleep(unsigned long msecs)
    {
        QThread::msleep(msecs);
    }
};
#else
#include <QQmlEngine>
#include <QQmlContext>
#include <QQuickView>
#endif









unsigned int binaryToGray(unsigned int num)
{
    return (num >> 1) ^ num;
}


unsigned int grayToBinary(unsigned int num)
{
    unsigned int len = 8 * sizeof(num);
    unsigned int binary = 0;

    /// mask the MSB.
    unsigned int grayBit = 1 << ( len - 1 );
    /// copy the MSB.
    binary = num & grayBit;
    /// store the bit we just set.
    unsigned int binBit = binary;
    /// traverse remaining Gray bits.
    while( grayBit >>= 1 ) {
        /// shift the current binary bit
        /// to align with the Gray bit.
        binBit >>= 1;
        /// XOR the two bits.
        binary |= binBit ^ ( num & grayBit );
        /// store the current binary bit
        binBit = binary & grayBit;
    }

    return binary;
}


cv::Mat decodeBinaryPatternImages(const std::vector<cv::Mat> &images, const std::vector<cv::Mat> &invImages, cv::Mat maskImg, bool isGrayCode = true)
{
    unsigned int imgCount = images.size();

    const cv::Size &imgSize = images[0].size();

    /// use 16 bits, it's more than enough
    cv::Mat decodedImg(imgSize, CV_16U, cv::Scalar(0));

    const int &imgHeight = imgSize.height;
    const int &imgWidth = imgSize.width;

    for (unsigned int i=0; i<imgCount; ++i) {
        cv::Mat regImg, invImg;

        if (false) {
            /// apply a median filter of 3x3
            cv::medianBlur(images[i], regImg, 3);
            cv::medianBlur(invImages[i], invImg, 3);
        } else {
            regImg = images[i];
            invImg = invImages[i];
        }

        //unsigned short bit = 1 << ( imgCount - i );
        unsigned short bit = 1 << ( i+1 );
        for (int y=0; y<imgHeight; ++y) {
            /// get pointers to first item of the row
            const unsigned char* maskImgData = maskImg.ptr<unsigned char>(y);
            const unsigned char* imgData = regImg.ptr<unsigned char>(y);
            const unsigned char* invImgData = invImg.ptr<unsigned char>(y);
            unsigned short* decodedImgData = decodedImg.ptr<unsigned short>(y);
            for (int x=0; x<imgWidth; ++x) {
                if (*maskImgData) {
                    unsigned short &value = *decodedImgData;
                    if (*imgData > *invImgData) {
                        /// enable bit
                        value |= bit;
                    } else {
                        /// disable bit
                        /// not neccessary, it starts with all bits in zero
                        //value &= ~bit;
                    }
                }

                /// don't forget to advance pointers!!
                maskImgData++;
                imgData++;
                invImgData++;
                decodedImgData++;
            }
        }
    }

    //const unsigned short multiplier = 1; //pow(2., 6); solo para ver mejor las imagenes cuando no tengo colormap (ie: visor de imagenes de windows)

    for (int y=0; y<imgHeight; ++y) {
        /// get pointers to first item of the row
        const unsigned char* maskImgData = maskImg.ptr<unsigned char>(y);
        unsigned short* decodedImgData = decodedImg.ptr<unsigned short>(y);
        for (int x=0; x<imgWidth; ++x) {
            if (*maskImgData) {
                unsigned short &value = *decodedImgData;
                if (isGrayCode) {
                    value = grayToBinary(value); // * multiplier;
                //} else {
                //    value *= multiplier;
                }

                /// debug
                //if (value % 2)
                //    value += 64;
            }

            /// don't forget to advance pointers!!
            maskImgData++;
            decodedImgData++;
        }
    }

    return decodedImg;
}


cv::Mat simplifyBinaryPatternData(cv::Mat image, cv::Mat maskImg, std::map<int, std::vector<cv::Vec2f> > &fringePoints)
{
    if (false) {
        /// apply a median filter of 3x3
        //cv::medianBlur(image, image, 3);

        /// values near irrelevant pixels should not be changed
        cv::Mat erodedMask(maskImg.size(), maskImg.type());
        //maskImg = maskImg > 0;
        //maskImg = maskImg * 255;
        cv::erode(maskImg, erodedMask, cv::Mat());
        cv::Mat bluredImage;
        cv::blur(image, bluredImage, cv::Size(3, 3));
        cv::Mat bluredMinusImage = bluredImage - image;

        cv::Mat erodedMask2(image.size(), image.type());
        erodedMask.convertTo(erodedMask2, image.type());
        //erodedMask2 = erodedMask2 * ((2^16)-1);

        /*qDebug() << "image" << image.type() << "size" << image.size().width << "x" << image.size().height << "\n"
                 << "bluredImage" << bluredImage.type() << "size" << bluredImage.size().width << "x" << bluredImage.size().height << "\n"
                 << "bluredMinusImage" << bluredMinusImage.type() << "size" << bluredMinusImage.size().width << "x" << bluredMinusImage.size().height << "\n"
                 << "erodedMask" << erodedMask.type() << "size" << erodedMask.size().width << "x" << erodedMask.size().height << "\n"
                 ;*/

        image = image + (bluredMinusImage & erodedMask2);
    }

    const cv::Size &imgSize = image.size();

    /// use 16 bits, it's enough
    cv::Mat decodedImg(imgSize, CV_16U, cv::Scalar(0));

    const int &imgHeight = imgSize.height;
    const int &imgWidth = imgSize.width;

    for (int y=0; y<imgHeight; ++y) {
        /// get pointers to first item of the row
        const unsigned char* maskImgData = maskImg.ptr<unsigned char>(y);
        const unsigned short* imgData = image.ptr<unsigned short>(y);
        unsigned short* decodedImgData = decodedImg.ptr<unsigned short>(y);
        for (int x=0; x<imgWidth-1; ++x) {
            /// get mask pixels
            const unsigned char &maskValue     = *maskImgData;
            maskImgData++; /// always advance pointer
            const unsigned char &nextMaskValue = *maskImgData;

            /// get image pixels
            const unsigned short &value     = *imgData;
            imgData++; /// always advance pointer
            const unsigned short &nextValue = *imgData;

            if (maskValue && nextMaskValue && nextValue != value) {
                *decodedImgData = value;
                //decodedImgData[x+1] = nextValue; // not both borders, it's useless. x => x+0.5
                unsigned short imin = qMin(value, nextValue),
                               imax = qMax(value, nextValue);
                cv::Vec2f point(0.5f+x, y); // x => x+0.5
                if (false) {
                    /// todos?
                    for (int i = imin; i < imax; ++i) { // not "<=" because we use the left border only
                        fringePoints[i].push_back(point);
                    }
                } else {
                    /// solo "bordes"
                    fringePoints[imin].push_back(point);
                    if (imax - imin > 1)
                        fringePoints[imax-1].push_back(point);
                }
            }

            /// always advance pointer
            decodedImgData++;
        }
    }

    /// smooth
    for (std::map<int, std::vector<cv::Vec2f> >::iterator it = fringePoints.begin(); it != fringePoints.end(); ++it) {
        if (false) {
            const double approxEps = 1.0;
            const std::vector<cv::Vec2f> &fringePointsVectorOrig = it->second;
            std::vector<cv::Vec2f> fringePointsVector;
            size_t origSize = fringePointsVectorOrig.size();
            cv::approxPolyDP(fringePointsVectorOrig, fringePointsVector, approxEps, false);

            if (origSize != fringePointsVector.size())
                qDebug() << "reduced fringe points from" << origSize << "to" << fringePointsVector.size();
        }

        if (false) {
            std::vector<cv::Vec2f> &fringePointsVectorOrig = it->second;
            if (fringePointsVectorOrig.size() > 2) {
                //int counter = 0;
                std::vector<cv::Vec2f>::iterator leftIt = fringePointsVectorOrig.begin();
                std::vector<cv::Vec2f>::iterator midIt = leftIt + 1;
                std::vector<cv::Vec2f>::iterator rightIt = midIt + 1;
                cv::Vec2f leftPoint = *leftIt;
                cv::Vec2f midPoint;

                while (rightIt != fringePointsVectorOrig.end()) {
                    midPoint = *midIt;

                    /// get (norm L2) ^ 2
                    /// don't use norm_L2 because its useless to calculate the sqrt
                    float sqrNorm = cv::norm(leftPoint - *rightIt, cv::NORM_L2SQR);
                    /// if it is <= 4 it means they are on the same line
                    /// if it is more than 10 they are too far away, don't use
                    if ( sqrNorm > 4. && sqrNorm < 10. ) {
                        *midIt = 0.5f * (leftPoint + *rightIt);
                        //counter++;
                    }

                    leftPoint = midPoint;

                    /// advance iterators
                    leftIt = midIt;
                    midIt = rightIt;
                    rightIt++;
                }

                //qDebug() << "modifyed" << counter << "of" << fringePointsVectorOrig.size();
            }

        }

    }

    return decodedImg;

    /*
    // Reduce noise with a kernel 3x3
    //cv::Mat denoised;
    //cv::blur( image, denoised, cv::Size(3,3) );

    cv::Mat dx(image.rows, image.cols, image.type());

    //void Sobel(InputArray src, OutputArray dst, int ddepth, int dx, int dy, int ksize=3, double scale=1, double delta=0, int borderType=BORDER_DEFAULT )¶
    cv::Sobel(image, dx, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_REPLICATE);
    return dx;


    /*cv::Mat detectedEdges;

    int lowThreshold;
    int ratio = 3;
    int kernel_size = 3;

    // Reduce noise with a kernel 3x3
    cv::blur( image, detectedEdges, cv::Size(3,3) );

    // Canny detector
    cv::Canny( detectedEdges, detectedEdges, lowThreshold, lowThreshold*ratio, kernel_size );

    // Using Canny's output as a mask, we display our result
    cv::Mat dst;
    dst = cv::Scalar::all(0);

    image.copyTo( dst, detectedEdges);

    return dst;*/
}









BinaryPatternProjection::BinaryPatternProjection(QObject *parent)
    : QObject(parent)
    , m_dlpview(0)
    , m_configWidget(0)
    , m_delayMs(400)
    , m_noiseThreshold(15)
    , m_automaticPatternCount(true)
    , m_numPatterns(11)
    , m_intensity(1.)
    , m_currentPattern(0)
    , m_inverted(false)
    , m_vertical(true)
    , m_useGrayBinary(true)
    , m_debugMode(false)
    , m_previewEnabled(false)
{
#if QT_VERSION < 0x050000
    //! Qt4: Creo QDeclarativeView con lo que se debe mostrar por el proyector
    m_dlpview = new QDeclarativeView();

    //    QGLFormat format = QGLFormat::defaultFormat();
    //    format.setSampleBuffers(false);
    //    format.setSwapInterval(1);
    //    QGLWidget* glWidget = new QGLWidget(format);
    //    glWidget->setAutoFillBackground(false);
    //    m_dlpview->setViewport(glWidget);

    //m_dlpview->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
    //m_dlpview->setAttribute(Qt::WA_OpaquePaintEvent);
    //m_dlpview->setAttribute(Qt::WA_NoSystemBackground);
    //QObject::connect(m_dlpview->engine(), SIGNAL(quit()), m_dlpview, SLOT(close()));

    m_dlpview->setSource(QUrl("qrc:///qml/binarypatternprojection.qml"));
    m_dlpview->setResizeMode(QDeclarativeView::SizeRootObjectToView);
#else
    //! Qt5: Creo QQuickView con lo que se debe mostrar por el proyector
    m_dlpview = new QQuickView();
    m_dlpview->setSource(QUrl("qrc:///qml/binarypatternprojection.qml"));
    m_dlpview->setResizeMode(QQuickView::SizeRootObjectToView);
#endif

    //! Exporto esta ventana, se usa para leer la "configuracion" de la ventana actual
    m_dlpview->rootContext()->setContextProperty("config", this);

    //! Para poder maximizar y manejar esas cosas desde qml
    m_dlpview->rootContext()->setContextProperty("window", m_dlpview);
}

BinaryPatternProjection::~BinaryPatternProjection()
{
    qDebug() << "deleting projector window...";
    if (m_dlpview)
        m_dlpview->deleteLater();
}

void BinaryPatternProjection::showProjectionWindow()
{
    m_dlpview->showFullScreen();
}

void BinaryPatternProjection::hideProjectionWindow()
{
    m_dlpview->hide();
}

void BinaryPatternProjection::setProjectionWindowGeometry(const QRect &geometry)
{
    m_dlpview->setGeometry(geometry);

    /// to skip useless patterns
    /// log2(number) == log(number)/log(2)
    m_maxUsefulPatterns = ceil(log((double)geometry.width())/log(2.));

    qDebug() << "geometry:" << geometry << "maxUsefulPatterns:" << m_maxUsefulPatterns;

    setAutomaticPatternCount(m_automaticPatternCount);
}

QWidget *BinaryPatternProjection::configWidget()
{
    if (!m_configWidget)
        m_configWidget = new BinaryPatternProjectionConfigWidget(this);

    return m_configWidget;
}

void BinaryPatternProjection::scan()
{
    bool previewWasEnabled = setPreviewEnabled(true);

    /// to skip useless patterns
    int firstPatternToShow = 0;

    //! FIXME mejorar esto
    if (!m_automaticPatternCount) {
        firstPatternToShow = m_maxUsefulPatterns - m_numPatterns;
    }

    qDebug() << "maxUsefulPatterns:" << m_maxUsefulPatterns;
    qDebug() << "numPatterns:" << m_numPatterns << "firstPatternToShow:" << firstPatternToShow;

    int numPatterns = qMax(m_numPatterns, m_maxUsefulPatterns) - firstPatternToShow + 1;

    Z3D::DecodedPattern::Ptr decodedPattern(new Z3D::DecodedPattern(m_camList.size()));

    decodedPattern->scanTmpFolder = QString("tmp/dlpscans/%1").arg(QDateTime::currentDateTime().toString("yyyy.MM.dd_hh.mm.ss"));

    foreach (Z3D::ZCalibratedCamera::Ptr calibratedCam, m_camList) {
        Z3D::ZCameraInterface::Ptr cam = calibratedCam->camera();

        if (m_debugMode) {
            QString cameraFolder = QString("%1/%2").arg(decodedPattern->scanTmpFolder).arg(cam->uuid());
            if (!QDir::current().mkpath(cameraFolder)) {
                qWarning() << "Unable to create folder:" << cameraFolder;
                return;
            }
        }

        /// start acquisition
        cam->startAcquisition();
    }

    /// 1st index: camera
    /// 2nd index: normal/inverted
    /// 3rd index: image number
    std::vector< std::vector< std::vector<cv::Mat> > > allImages;

    setVertical(m_vertical);

    /// 1st index: camera
    /// 2nd index: normal/inverted
    /// 3rd index: image number
    allImages.resize(m_camList.size());
    for (int iCam=0; iCam<m_camList.size(); ++iCam) {
        allImages[iCam].resize(2);
        allImages[iCam][0].resize(numPatterns); // images
        allImages[iCam][1].resize(numPatterns); // inverted images
    }

    /// acquisition time
    QTime acquisitionTime;
    acquisitionTime.start();

    int realIndex = -1;
    for (int iPattern=0; iPattern <= m_maxUsefulPatterns; ++iPattern) {
        /// always show first (all white, used to stablish the noise threshold)
        /// but skip all up to firstPatternToShow
        if (iPattern && iPattern<=firstPatternToShow)
            continue;

        realIndex++;

        setCurrentPattern(iPattern);

        for (unsigned int inverted=0; inverted<2; ++inverted) {
            setInverted(inverted != 0);

            QCoreApplication::processEvents();
            QCoreApplication::processEvents();

#if QT_VERSION < 0x050000
            SleeperThread::msleep(m_delayMs);
#else
            QThread::msleep(m_delayMs);
#endif

            QCoreApplication::processEvents();
            QCoreApplication::processEvents();

            for (int iCam=0; iCam<m_camList.size(); ++iCam) {
                Z3D::ZCameraInterface::Ptr cam = m_camList[iCam]->camera();

                QString fileName = QString("%1_%2%3.png")
                        .arg(m_useGrayBinary?"gray":"binary")
                        .arg(iPattern, 2, 10, QLatin1Char('0'))
                        .arg(inverted?"_inv":"");

                /// if the camera is simulated, set which image should use now
                if (cam->uuid().startsWith("ZSIMULATED")) {
                    qDebug() << "camera is simulated. using image" << fileName;
                    cam->setAttribute("CurrentFile", fileName);
                    QCoreApplication::processEvents();
                    QCoreApplication::processEvents();
                }

                /// Retrieve an image
                Z3D::ZImageGrayscale::Ptr imptr = cam->getSnapshot();

                if (!imptr) {
                    qCritical() << "error obtaining snapshot for camera" << cam->uuid();
                    return;
                }

                cv::Mat image = imptr->cvMat().clone();

                if (decodedPattern->intensityImg.empty()) {
                    decodedPattern->intensityImg = image;
                }

                allImages[iCam][inverted][realIndex] = image;

                if (m_debugMode) {
                    /// save image
                    cv::imwrite(qPrintable(QString("%1/%2/%3")
                                           .arg(decodedPattern->scanTmpFolder)
                                           .arg(cam->uuid())
                                           .arg(fileName)),
                                image);
                }
            }
        }
    }

    qDebug() << "acquisition finished in" << acquisitionTime.elapsed() << "msecs";


    /// FIXME esto es por las dudas pero deberia solucionarlo antes
    for (int iCam=0; iCam<m_camList.size(); ++iCam) {
        allImages[iCam][0].resize(realIndex+1); // images
        allImages[iCam][1].resize(realIndex+1); // inverted images
    }


    /// stop acquisition
    foreach (Z3D::ZCalibratedCamera::Ptr calibratedCam, m_camList) {
        Z3D::ZCameraInterface::Ptr cam = calibratedCam->camera();
        cam->stopAcquisition();
    }

    /// decodification time
    QTime decodeTime;
    decodeTime.start();

    for (int iCam=0; iCam<m_camList.size(); ++iCam) {
        /// decode images

        std::vector< std::vector<cv::Mat> > &cameraImages = allImages[iCam];
        std::vector<cv::Mat> &whiteImages = cameraImages[0];
        std::vector<cv::Mat> &inverseImages = cameraImages[1];

        /// the first images are the all white, all black
        /// they are used to create the mask of valid pixels
        const cv::Mat &whiteImg = whiteImages.front();
        const cv::Mat &inverseImg = inverseImages.front();

        cv::Mat maskImg = whiteImg - inverseImg;
        /// set mask to keep only values greater than threshold
        maskImg = maskImg > m_noiseThreshold;

        /// the first images are useless to decode pattern
        whiteImages.erase(whiteImages.begin());
        inverseImages.erase(inverseImages.begin());

        /// decode binary pattern using the rest of the images
        cv::Mat decoded = decodeBinaryPatternImages(whiteImages, inverseImages, maskImg, m_useGrayBinary);

        if (m_debugMode) {
            cv::imwrite(qPrintable(QString("%1/%2/decoded.tiff")
                                   .arg(decodedPattern->scanTmpFolder)
                                   .arg(m_camList[iCam]->camera()->uuid())),
                        decoded);

//            /// test
//            cv::Mat blurredDecoded;

//            cv::blur(decoded, blurredDecoded, cv::Size(3,3));
//            cv::imwrite(qPrintable(QString("%1/%2/decoded_blur3x3.tiff")
//                                   .arg(decodedPattern->scanTmpFolder)
//                                   .arg(m_camList[iCam]->camera()->uuid())),
//                        blurredDecoded);

//            cv::blur(decoded, blurredDecoded, cv::Size(5,5));
//            cv::imwrite(qPrintable(QString("%1/%2/decoded_blur5x5.tiff")
//                                   .arg(decodedPattern->scanTmpFolder)
//                                   .arg(m_camList[iCam]->camera()->uuid())),
//                        blurredDecoded);

//            cv::medianBlur(decoded, blurredDecoded, 3);
//            cv::imwrite(qPrintable(QString("%1/%2/decoded_median3x3.tiff")
//                                   .arg(decodedPattern->scanTmpFolder)
//                                   .arg(m_camList[iCam]->camera()->uuid())),
//                        blurredDecoded);

//            cv::medianBlur(decoded, blurredDecoded, 5);
//            cv::imwrite(qPrintable(QString("%1/%2/decoded_median5x5.tiff")
//                                   .arg(decodedPattern->scanTmpFolder)
//                                   .arg(m_camList[iCam]->camera()->uuid())),
//                        blurredDecoded);
        }

        /// vector of points for every fringe
        std::map<int, std::vector<cv::Vec2f> > &fringePoints = decodedPattern->fringePointsList[iCam];
        cv::Mat decodedBorders = simplifyBinaryPatternData(decoded, maskImg, fringePoints);

        /// to know how many point we could have (to reserve memory)
        int totalPoints = 0;
        for (std::map<int, std::vector<cv::Vec2f> >::iterator it = fringePoints.begin(); it != fringePoints.end(); ++it) {
            totalPoints += it->second.size();
        }
        decodedPattern->estimatedCloudPoints += totalPoints;

        /// debug info
        decodedPattern->maskImg[iCam] = maskImg;
        decodedPattern->decodedImage[iCam] = decoded;
        decodedPattern->fringeImage[iCam] = decodedBorders;
    }

    qDebug() << "pattern decodification finished in" << decodeTime.elapsed() << "msecs";

    /// notify result
    emit patternDecoded(decodedPattern);

    /// return previous state
    setPreviewEnabled(previewWasEnabled);
}

void BinaryPatternProjection::setCameras(QList<Z3D::ZCalibratedCamera::Ptr> cameraList)
{
    m_camList = cameraList;
}

double BinaryPatternProjection::intensity()
{
    return m_intensity;
}

void BinaryPatternProjection::setIntensity(double arg)
{
    if (m_intensity != arg) {
        m_intensity = arg;
        emit intensityChanged(arg);
    }
}

int BinaryPatternProjection::currentPattern()
{
    return m_currentPattern;
}

void BinaryPatternProjection::setCurrentPattern(int arg)
{
    if (m_currentPattern != arg) {
        m_currentPattern = arg;
        emit currentPatternChanged(arg);
    }
}

bool BinaryPatternProjection::inverted()
{
    return m_inverted;
}

void BinaryPatternProjection::setInverted(bool arg)
{
    if (m_inverted != arg) {
        m_inverted = arg;
        emit invertedChanged(arg);
    }
}

bool BinaryPatternProjection::vertical()
{
    return m_vertical;
}

void BinaryPatternProjection::setVertical(bool arg)
{
    if (m_vertical != arg) {
        m_vertical = arg;
        emit verticalChanged(arg);
    }
}

bool BinaryPatternProjection::useGrayBinary()
{
    return m_useGrayBinary;
}

void BinaryPatternProjection::setUseGrayBinary(bool arg)
{
    if (m_useGrayBinary != arg) {
        m_useGrayBinary = arg;
        emit useGrayBinaryChanged(arg);
    }
}

int BinaryPatternProjection::delayMs() const
{
    return m_delayMs;
}

void BinaryPatternProjection::setDelayMs(int arg)
{
    if (m_delayMs != arg) {
        m_delayMs = arg;
        emit delayMsChanged(arg);
    }
}

int BinaryPatternProjection::noiseThreshold() const
{
    return m_noiseThreshold;
}

void BinaryPatternProjection::setNoiseThreshold(int arg)
{
    if (m_noiseThreshold != arg) {
        m_noiseThreshold = arg;
        emit noiseThresholdChanged(arg);
    }
}

int BinaryPatternProjection::numPatterns() const
{
    return m_numPatterns;
}

void BinaryPatternProjection::setNumPatterns(int arg)
{
    if (m_numPatterns != arg) {
        m_numPatterns = arg;
        emit numPatternsChanged(arg);

        if (!m_automaticPatternCount) {
            int firstPatternToShow = m_maxUsefulPatterns - m_numPatterns;
            qDebug() << "not using automatic pattern count. maxUsefulPatterns:" << m_maxUsefulPatterns;
            qDebug() << "numPatterns:" << m_numPatterns << "firstPatternToShow:" << firstPatternToShow;
        }
    }
}

bool BinaryPatternProjection::debugMode() const
{
    return m_debugMode;
}

void BinaryPatternProjection::setDebugMode(bool arg)
{
    if (m_debugMode != arg) {
        m_debugMode = arg;
        emit debugModeChanged(arg);
    }
}

bool BinaryPatternProjection::previewEnabled() const
{
    return m_previewEnabled;
}

bool BinaryPatternProjection::setPreviewEnabled(bool arg)
{
    bool previousPreviewState = m_previewEnabled;

    if (m_previewEnabled != arg) {
        m_previewEnabled = arg;
        emit previewEnabledChanged(arg);
    }

    if (m_previewEnabled)
        showProjectionWindow();
    else
        hideProjectionWindow();

    return previousPreviewState;
}

bool BinaryPatternProjection::automaticPatternCount() const
{
    return m_automaticPatternCount;
}

void BinaryPatternProjection::setAutomaticPatternCount(bool arg)
{
    if (m_automaticPatternCount != arg) {
        m_automaticPatternCount = arg;
        emit automaticPatternCountChanged(arg);
    }

    if (m_automaticPatternCount) {
        setNumPatterns(m_maxUsefulPatterns);
    }
}


