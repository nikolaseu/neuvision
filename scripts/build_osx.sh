#!/bin/bash
export Qt5_DIR=$(brew --prefix qt)
export OpenCV_DIR=$(brew --prefix opencv)

BUILD_TYPE=Release
STARTING_DIR=$(pwd)
BUILD_DIR=$STARTING_DIR/build/$BUILD_TYPE
INSTALL_DIR=$STARTING_DIR/build/install/$BUILD_TYPE

mkdir "$BUILD_DIR"
cd "$BUILD_DIR"

cmake $STARTING_DIR \
	-DCMAKE_OSX_SYSROOT="/Library/Developer/CommandLineTools/SDKs/MacOSX.sdk" \
	-DCMAKE_BUILD_TYPE=$BUILD_TYPE \
	-DCMAKE_INSTALL_PREFIX=$INSTALL_DIR

cmake --build . --config $BUILD_TYPE

cmake --build . --target install --config $BUILD_TYPE
