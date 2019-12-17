
set Qt5_DIR=C:/Qt/5.12/msvc2017_64
set OpenCV_DIR=C:/Tools/vcpkg/installed/x64-windows

set BUILD_TYPE=Release
set INSTALL_DIR=bin/%BUILD_TYPE%

cd build

cmake -GNinja ^
	.. ^
	-DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
	-DCMAKE_INSTALL_PREFIX="%INSTALL_DIR%"
	
REM cmake --build . --target install --config %BUILD_TYPE%
cmake --build . --config %BUILD_TYPE%
