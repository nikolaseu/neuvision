# Structured Light based 3D scanner

[![Build Status](https://travis-ci.com/nikolaseu/neuvision.svg?branch=master)](https://travis-ci.com/nikolaseu/neuvision)

Engineering degree thesis/project for the title of "Ingeniero en Informatica", Universidad Nacional del Litoral (Santa Fe, Argentina).

Originally used two cameras + one DLP projector to get more precission, but I'm trying to add a calibration wizard to use it with one camera + one projector, to make it easier/simpler for home-use.

## Camera support

Includes a lot of plugins that used to make it work with a lot of different cameras, mostly industrial GigE or USB cameras, but since I don't have access to them anymore, they are not tested/enabled anymore.

There's also a plugin for OpenCV cameras, so any webcam that works with OpenCV should work too.

## Requirements

You'll need some libraries installed:

- [Qt](https://www.qt.io) at least version 5.6
- [OpenCV](http://opencv.org) version 3.3, but should work with older versions too

If you have macOS and install the libraries with homebrew this should be enough. For Linux/Windows you might have to update the paths so Qt Creator can find each library to compile/link/run the application.

### Other optional requirements

For the cloud viewer you'll also need:

- [PCL](http://www.pointclouds.org) version 1.8.1
- [VTK](http://www.vtk.org) version 7

## Applications

### 3D scanner

To run the 3D scanner app, just open the Qt Creator project file `Z3DScanner.pro` and run the application.

### Multi camera calibration

To run the stereo calibration app, open the Qt Creator project file `Z3DMultiCameraCalibration.pro` and run the application.
