
set Qt5_DIR=C:/Qt/5.14.0/msvc2017_64
set OpenCV_DIR=C:/Users/nikol/Projects/opencv-4.1.0-vc14_vc15/build

set BUILD_TYPE=Release
set INSTALL_DIR=bin/%BUILD_TYPE%

cd build

cmake -GNinja ^
	.. ^
	-DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
	-DCMAKE_INSTALL_PREFIX="%INSTALL_DIR%"
	
REM cmake --build . --target install --config %BUILD_TYPE%
cmake --build . --config %BUILD_TYPE%
