/* * Z3D - A structured light 3D scanner
 * Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
 *
 * This file is part of Z3D.
 *
 * Z3D is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Z3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "zmulticameracalibrator.h"

namespace Z3D
{

class ZOpenCVStereoMultiCameraCalibrator : public ZMultiCameraCalibrator
{
    Q_OBJECT

    /// stereo calibration flags
    Q_PROPERTY(bool fixIntrinsic             READ fixIntrinsic              WRITE setFixIntrinsic              NOTIFY fixIntrinsicChanged)
    Q_PROPERTY(bool sameFocalLength          READ sameFocalLength           WRITE setSameFocalLength           NOTIFY sameFocalLengthChanged)

    /// calibration flags
    Q_PROPERTY(bool useIntrinsicGuess        READ useIntrinsicGuess         WRITE setUseIntrinsicGuess         NOTIFY useIntrinsicGuessChanged)
    Q_PROPERTY(bool fixPrincipalPoint        READ fixPrincipalPoint         WRITE setFixPrincipalPoint         NOTIFY fixPrincipalPointChanged)
    Q_PROPERTY(bool fixAspectRatio           READ fixAspectRatio            WRITE setFixAspectRatio            NOTIFY fixAspectRatioChanged)
    Q_PROPERTY(bool zeroTangentialDistortion READ zeroTangentialDistortion  WRITE setZeroTangentialDistortion  NOTIFY zeroTangentialDistortionChanged)
    Q_PROPERTY(bool fixK1                    READ fixK1                     WRITE setFixK1                     NOTIFY fixK1Changed)
    Q_PROPERTY(bool fixK2                    READ fixK2                     WRITE setFixK2                     NOTIFY fixK2Changed)
    Q_PROPERTY(bool fixK3                    READ fixK3                     WRITE setFixK3                     NOTIFY fixK3Changed)
    Q_PROPERTY(bool fixK4                    READ fixK4                     WRITE setFixK4                     NOTIFY fixK4Changed)
    Q_PROPERTY(bool fixK5                    READ fixK5                     WRITE setFixK5                     NOTIFY fixK5Changed)
    Q_PROPERTY(bool fixK6                    READ fixK6                     WRITE setFixK6                     NOTIFY fixK6Changed)
    Q_PROPERTY(bool useRationalModel         READ useRationalModel          WRITE setUseRationalModel          NOTIFY useRationalModelChanged)

    /// termination criteria
    Q_PROPERTY(int termCriteriaMaxIterations READ termCriteriaMaxIterations WRITE setTermCriteriaMaxIterations NOTIFY termCriteriaMaxIterationsChanged)
    Q_PROPERTY(double termCriteriaEpsilon    READ termCriteriaEpsilon       WRITE setTermCriteriaEpsilon       NOTIFY termCriteriaEpsilonChanged)

    Q_PROPERTY(bool isDebugMode READ isDebugMode WRITE setDebugMode NOTIFY debugModeChanged)

public:
    explicit ZOpenCVStereoMultiCameraCalibrator(QObject *parent = 0);

    virtual ~ZOpenCVStereoMultiCameraCalibrator();

    /// ZMultiCameraCalibrator interface
    virtual QString name() override;

    virtual std::vector<ZCameraCalibration::Ptr> getCalibration(
            std::vector<ZCameraCalibration::Ptr> &initialCameraCalibrations,
            std::vector<std::vector<std::vector<cv::Point2f> > > &imagePoints,
            std::vector<std::vector<cv::Point3f> > &objectPoints) override;

    /// stereo calibration flags
    bool fixIntrinsic() const;
    bool sameFocalLength() const;

    /// calibration flags
    bool useIntrinsicGuess() const;
    bool fixPrincipalPoint() const;
    bool fixAspectRatio() const;
    bool zeroTangentialDistortion() const;
    bool fixK1() const;
    bool fixK2() const;
    bool fixK3() const;
    bool fixK4() const;
    bool fixK5() const;
    bool fixK6() const;
    bool useRationalModel() const;

    /// termination criteria
    int termCriteriaMaxIterations() const;
    double termCriteriaEpsilon() const;

    bool isDebugMode() const;

signals:
    void fixIntrinsicChanged(bool arg);
    void sameFocalLengthChanged(bool arg);

    void useIntrinsicGuessChanged(bool arg);
    void fixPrincipalPointChanged(bool arg);
    void fixAspectRatioChanged(bool arg);
    void zeroTangentialDistortionChanged(bool arg);
    void fixK1Changed(bool arg);
    void fixK2Changed(bool arg);
    void fixK3Changed(bool arg);
    void fixK4Changed(bool arg);
    void fixK5Changed(bool arg);
    void fixK6Changed(bool arg);
    void useRationalModelChanged(bool arg);

    void termCriteriaMaxIterationsChanged(int arg);
    void termCriteriaEpsilonChanged(double arg);

    void debugModeChanged(bool isDebugMode);

public slots:
    void setFixIntrinsic(bool arg);
    void setSameFocalLength(bool arg);

    void setUseIntrinsicGuess(bool arg);
    void setFixPrincipalPoint(bool arg);
    void setFixAspectRatio(bool arg);
    void setZeroTangentialDistortion(bool arg);
    void setFixK1(bool arg);
    void setFixK2(bool arg);
    void setFixK3(bool arg);
    void setFixK4(bool arg);
    void setFixK5(bool arg);
    void setFixK6(bool arg);
    void setUseRationalModel(bool arg);

    void setTermCriteriaMaxIterations(int arg);
    void setTermCriteriaEpsilon(double arg);

    void setDebugMode(bool isDebugMode);

protected:
    bool m_fixIntrinsic;

    bool m_sameFocalLength;

    /// cv::calibrateCamera flags
    /// Different flags that may be zero or a combination of the following
    /// values:
    /// - CV_CALIB_USE_INTRINSIC_GUESS cameraMatrix contains valid initial
    /// values of fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy)
    /// is initially set to the image center ( imageSize is used), and focal
    /// distances are computed in a least-squares fashion. Note, that if
    /// intrinsic parameters are known, there is no need to use this function
    /// just to estimate extrinsic parameters. Use solvePnP() instead.
    bool m_useIntrinsicGuess;

    /// - CV_CALIB_FIX_PRINCIPAL_POINT The principal point is not changed during
    /// the global optimization. It stays at the center or at a different
    /// location specified when CV_CALIB_USE_INTRINSIC_GUESS is set too.
    bool m_fixPrincipalPoint;

    /// - CV_CALIB_FIX_ASPECT_RATIO The functions considers only fy as a free
    /// parameter. The ratio fx/fy stays the same as in the input cameraMatrix.
    /// When CV_CALIB_USE_INTRINSIC_GUESS is not set, the actual input values of
    /// fx and fy are ignored, only their ratio is computed and used further.
    bool m_fixAspectRatio;

    /// - CV_CALIB_ZERO_TANGENT_DIST Tangential distortion coefficients
    /// (p_1, p_2) are set to zeros and stay zero.
    bool m_zeroTangentialDistortion;

    /// - CV_CALIB_FIX_K1,...,CV_CALIB_FIX_K6 The corresponding radial
    /// distortion coefficient is not changed during the optimization.
    /// If CV_CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
    /// supplied distCoeffs matrix is used. Otherwise, it is set to 0.
    bool m_fixK1;
    bool m_fixK2;
    bool m_fixK3;
    bool m_fixK4;
    bool m_fixK5;
    bool m_fixK6;

    /// - CV_CALIB_RATIONAL_MODEL Coefficients k4, k5, and k6 are enabled. To
    /// provide the backward compatibility, this extra flag should be explicitly
    /// specified to make the calibration function use the rational model and
    /// return 8 coefficients. If the flag is not set, the function computes and
    /// returns only 5 distortion coefficients.
    bool m_useRationalModel;

    /// cv::calibrateCamera termination criteria
    int m_termCriteriaMaxIterations;
    double m_termCriteriaEpsilon;

    /// when debug mode is enabled we save debug information
    bool m_isDebugMode;
};

} // namespace Z3D
