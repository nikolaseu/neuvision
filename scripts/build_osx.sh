
export Qt5_DIR=$(brew --prefix qt)
export OpenCV_DIR=$(brew --prefix opencv)

export BUILD_TYPE=Release
export INSTALL_DIR=bin/%BUILD_TYPE%

cd build

cmake .. \
	-DCMAKE_BUILD_TYPE=%BUILD_TYPE% \
	-DCMAKE_INSTALL_PREFIX="%INSTALL_DIR%"
	
# cmake --build . --target install --config %BUILD_TYPE%
cmake --build . --config %BUILD_TYPE%
