echo off

:: Make sure that script is run as administrator.  This is more of
:: a hack than a real check, but it seems to work
at > nul
if ERRORLEVEL 1 GOTO not-admin
echo Executing as administrator... good :)

:: Checking for cmake, Python, and mingw
:prereqs
where /usr/bin/python 0>nul 
IF ERRORLEVEL 1 GOTO no-python

where /usr/bin/cmake 0>nul 
IF ERRORLEVEL 1 GOTO no-cmake

where mingw32-make 0>nul 
IF ERRORLEVEL 1 GOTO no-mingw

echo.
echo Found all dependencies!  :D
echo.
echo This batch file will download, build and install GCCXML from source using
echo MinGW to C:\gccxml, and install Pygccxml and Py++ as well.  The PATH 
echo variable will also be amended to include the path to the gccxml executable,
echo and a new variable, GCCXML_COMPILER will be created to instruct gccxml 
echo which compiler to emulate.
PAUSE
GOTO download-deps

:not-admin
ECHO ERROR: You must execute this file with administrator privileges!
GOTO END

:no-python
ECHO Python is not found.  Install Python 2.x (32-bit) and add python.exe to the PATH, then re-run this batch file.
GOTO END

:no-cmake
ECHO CMake is not found.  Install CMake and add the executable to the PATH, then re-run this batch file.
GOTO END

:no-mingw
ECHO mingw32-make is not found.  Install MinGW and add it to the PATH, then re-run this batch file.
GOTO END

:download-deps
del /S /Q "/wg/stor5/mpomarlan/ompl/build/tempdir"
del /S /Q "/wg/stor5/mpomarlan/ompl/src/external/gccxml"
del /S /Q "/wg/stor5/mpomarlan/ompl/src/external/pygccxml"
del /S /Q "/wg/stor5/mpomarlan/ompl/src/external/pyplusplus"
/usr/bin/cmake -DTEMPDIR="/wg/stor5/mpomarlan/ompl/build/tempdir" -P /wg/stor5/mpomarlan/ompl/src/external/downloadPyPlusPlus.cmake
/usr/bin/cmake -E tar xfz "/wg/stor5/mpomarlan/ompl/build/tempdir/gccxml-2012-11-06.tgz"
/usr/bin/cmake -E tar xfz "/wg/stor5/mpomarlan/ompl/build/tempdir/pygccxml-r575.tgz"
/usr/bin/cmake -E tar xfz "/wg/stor5/mpomarlan/ompl/build/tempdir/pyplusplus-r1238.tgz"

:install-gccxml
cd gccxml
/usr/bin/cmake -G "MinGW Makefiles" -DCMAKE_INSTALL_PREFIX="C:\gccxml"
/usr/bin/cmake --build . --target install
cd ..

SET GCCXMLPATH=C:\gccxml\bin
:: ADD gccxml to path and create the GCCXML_COMPILER variable

set PATH=%PATH%;%GCCXMLPATH%
:: Set the PATH and GCCXML_COMPILER variables permanently
setx PATH "%PATH%" /M 0>nul
setx GCCXML_COMPILER g++ /M 0>nul

:install-pygccxml
cd pygccxml
/usr/bin/python setup.py build
/usr/bin/python setup.py install
cd ..

:install-pyplusplus
cd pyplusplus
/usr/bin/python setup.py build
/usr/bin/python setup.py install
cd ..

:END

