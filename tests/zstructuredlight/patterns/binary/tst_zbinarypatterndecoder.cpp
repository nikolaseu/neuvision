#include "zbinarypatterndecoder.h"

#include "ZStructuredLight/zdecodedpattern.h"

#include <QtTest/QTest>
#include <opencv2/imgcodecs.hpp>

namespace
{

// taken from https://stackoverflow.com/questions/9905093/how-to-check-whether-two-matrices-are-identical-in-opencv
bool matIsEqual(const cv::Mat &Mat1, const cv::Mat &Mat2)
{
    if (Mat1.dims == Mat2.dims &&
        Mat1.size == Mat2.size &&
        Mat1.elemSize() == Mat2.elemSize())
    {
        if (Mat1.isContinuous() && Mat2.isContinuous()) {
            return 0 == memcmp(Mat1.ptr(), Mat2.ptr(), Mat1.total() * Mat1.elemSize());
        }
        else {
            const cv::Mat* arrays[] = {&Mat1, &Mat2, nullptr};
            uchar* ptrs[2];
            cv::NAryMatIterator it(arrays, ptrs, 2);
            for (unsigned int p = 0; p < it.nplanes; p++, ++it) {
                if (0 != memcmp(it.ptrs[0], it.ptrs[1], it.size * Mat1.elemSize())) {
                    return false;
                }
            }

            return true;
        }
    }

    return false;
}

}

class ZBinaryPatternDecoderTests : public QObject
{
    Q_OBJECT

public:

private Q_SLOTS:
    void tst_decodeGrayCode()
    {
        std::vector<cv::Mat> images;
        std::vector<cv::Mat> invImages;

        for (size_t i=0; i<=11; ++i) {
            const QString imgFilename = QFINDTESTDATA(QString("data_binary_gray/gray_%1.png").arg(i, 2, 10, QChar('0')));
            const QString imgInvFilename = QFINDTESTDATA(QString("data_binary_gray/gray_%1_inv.png").arg(i, 2, 10, QChar('0')));

            const cv::Mat img = cv::imread(imgFilename.toStdString(), cv::IMREAD_UNCHANGED);
            QVERIFY2(!img.empty(), QString("Couldn't read image %1").arg(imgFilename).toLatin1());

            const cv::Mat imgInv = cv::imread(imgInvFilename.toStdString(), cv::IMREAD_UNCHANGED);
            QVERIFY2(!imgInv.empty(), QString("Couldn't read inv image %1").arg(imgInvFilename).toLatin1());

            images.push_back(img);
            invImages.push_back(imgInv);
        }

        // apply threshold for computing mask
        cv::Mat whiteImg = images[0];
        cv::Mat blackImg = invImages[0];
        const cv::Mat maskImg = (whiteImg - blackImg) > 20;

        // remove all black/white images only used for threshold
        images.erase(images.begin());
        invImages.erase(invImages.begin());

        const bool isGrayCode = true;

        const cv::Mat decodedPattern = Z3D::ZBinaryPatternDecoder::decodeBinaryPatternImages(images, invImages, maskImg, isGrayCode);

        const QString groundTruthFilename = QFINDTESTDATA("data_binary_gray/decoded.exr");
        const cv::Mat groundTruthPattern = cv::imread(groundTruthFilename.toStdString(), cv::IMREAD_UNCHANGED);

        QCOMPARE(decodedPattern.type(), groundTruthPattern.type());
        QCOMPARE(decodedPattern.type(), CV_32FC1);
        QCOMPARE(decodedPattern.cols, groundTruthPattern.cols);
        QCOMPARE(decodedPattern.rows, groundTruthPattern.rows);

        QVERIFY(matIsEqual(decodedPattern, groundTruthPattern));
    }
};

QTEST_APPLESS_MAIN(ZBinaryPatternDecoderTests)
#include "tst_zbinarypatterndecoder.moc"
