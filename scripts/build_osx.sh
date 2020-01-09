#!/bin/bash
export Qt5_DIR=$(brew --prefix qt)
export OpenCV_DIR=$(brew --prefix opencv)

. $(dirname "$0")/build_linux.sh
