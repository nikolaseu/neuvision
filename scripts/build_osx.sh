#!/bin/bash
export Qt5_DIR=$(brew --prefix qt)
export OpenCV_DIR=$(brew --prefix opencv)

./build_linux.sh
