REM set QT_DIR=C:/Qt/6.4/msvc2019_64
REM set OpenCV_DIR=C:/Tools/vcpkg/installed/x64-windows

set BUILD_TYPE=Release
set STARTING_DIR=%cd%
set BUILD_DIR=%STARTING_DIR%\build\%BUILD_TYPE%
set INSTALL_DIR=%STARTING_DIR%\build\install\%BUILD_TYPE%

cmake ^
	-S %STARTING_DIR% ^
	-B %BUILD_DIR% ^
	-G Ninja ^
	-DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
	-DCMAKE_INSTALL_PREFIX="%INSTALL_DIR%"
if %errorlevel% neq 0 goto :cmFail

cmake --build %BUILD_DIR% --config %BUILD_TYPE%
if %errorlevel% neq 0 goto :cmFail

cmake --build %BUILD_DIR% --target install --config %BUILD_TYPE%
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
exit /b 12345

:cmEnd
cd %STARTING_DIR%
