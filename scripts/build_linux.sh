#!/bin/bash

BUILD_TYPE=Release
STARTING_DIR=$(pwd)
BUILD_DIR=$STARTING_DIR/build/$BUILD_TYPE
INSTALL_DIR=$STARTING_DIR/build/install/$BUILD_TYPE

cmake \
	-S $STARTING_DIR \
	-B $BUILD_DIR \
	-G Ninja \
	-DCMAKE_BUILD_TYPE=$BUILD_TYPE \
	-DCMAKE_INSTALL_PREFIX=$INSTALL_DIR

cmake --build $BUILD_DIR --config $BUILD_TYPE

cmake --build $BUILD_DIR --target install --config $BUILD_TYPE
