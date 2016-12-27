#!/bin/sh
copyright-header  --license GPL3  \
                  --add-path lib/:Z3DCameraViewer/:Z3DScanner/:LTSAcquisition/:QQCameraCalibration/:Z3DCameraCalibration/:Z3DCloudViewer/ \
                  --guess-extension \
                  --copyright-holder 'Nicolas Ulrich <nikolaseu@gmail.com>' \
                  --copyright-software 'Z3D' \
                  --copyright-software-description "A structured light 3D scanner" \
                  --copyright-year 2013-2016 \
                  --word-wrap 100 \
                  --output-dir ./
