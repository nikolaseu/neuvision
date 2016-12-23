# Structured Light based 3D scanner

Engineering degree thesis/project for the title of "Ingeniero en Informatica", Universidad Nacional del Litoral (Santa Fe, Argentina).

Originally used two cameras + one DLP projector to get more precission, but I'm trying to add a calibration wizard to use it with one camera + one projector, to make it easier/simpler for home-use.

## Camera support

Includes a lot of plugins that used to make it work with a lot of different cameras, mostly industrial GigE or USB cameras, but since I don't have access to them anymore, they are not tested/enabled anymore.

There's also a plugin for OpenCV cameras, so any webcam that works with OpenCV should work too.

## Installation

First you'll need some libraries installed:
- OpenCV
- PCL
- VTK
- Qt

Then open the Qt Creator project file `NEUVision.pro`

> If you have macOS and install the libraries with homebrew, you should be ready to use. For Linux/Windows you might need to update the path so Qt Creator can find each library.

If everything is fine, select Z3DScanner and hit `play`

