set QT_DIR=C:/Qt/5.15/msvc2019_64
set Qt5_DIR=%QT_DIR%
set OpenCV_DIR=C:/Tools/vcpkg/installed/x64-windows

set BUILD_TYPE=Release
set STARTING_DIR=%cd%
set BUILD_DIR=%STARTING_DIR%\build\%BUILD_TYPE%
set INSTALL_DIR=%STARTING_DIR%\build\install\%BUILD_TYPE%

md "%BUILD_DIR%"
cd "%BUILD_DIR%"

cmake %STARTING_DIR% ^
	-GNinja ^
	-DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
	-DCMAKE_INSTALL_PREFIX="%INSTALL_DIR%"
if %errorlevel% neq 0 goto :cmFail

cmake --build . --config %BUILD_TYPE%
if %errorlevel% neq 0 goto :cmFail

cmake --build . --target install --config %BUILD_TYPE%
if %errorlevel% neq 0 goto :cmFail


REM ===========================================================================
REM everything went fine, just go to end
REM ===========================================================================
echo --------------------------------------------------------------------------
echo Build SUCCEEDED!
goto :cmEnd

REM ===========================================================================
REM something failed, notify
REM ===========================================================================
:cmFail
echo --------------------------------------------------------------------------
echo Build FAILED!

:cmEnd
cd %STARTING_DIR%