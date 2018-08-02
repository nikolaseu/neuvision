# A framework for Structured Light based 3D scanning projects

[![Build Status](https://travis-ci.com/nikolaseu/neuvision.svg?branch=master)](https://travis-ci.com/nikolaseu/neuvision)
[![Build status](https://ci.appveyor.com/api/projects/status/q3vr5tbjbh3v8jh5/branch/master?svg=true)](https://ci.appveyor.com/project/nikolaseu/neuvision/branch/master)

Engineering degree thesis/project for the title of "Ingeniero en Informatica", Universidad Nacional del Litoral (Santa Fe, Argentina).

Originally used two cameras + one DLP projector to get more precission, but I'm trying to add a calibration wizard to use it with one camera + one projector, to make it easier/simpler for home-use.

## Camera support

There's a plugin for OpenCV, so any webcam that works with OpenCV should work too.
On Linux & macOS you can also use the libgphoto2 plugin.

There are also [a lot of plugins](./lib/zcameraacquisition/plugins) that (used to) make it work with
a lot of different cameras, mostly industrial GigE or USB cameras, but since I don't have access to
them anymore, they are not tested/enabled anymore. Contact me if you have a camera from:
- [Basler](https://www.baslerweb.com)
- [AVT / Allied Vision](https://www.alliedvision.com)
- [Point Grey](https://www.ptgrey.com)
- [Pleora Technologies](https://www.pleora.com)

## Requirements

We assume you already have a suitable C++ compiler and [Qt](https://www.qt.io) installed (at least
version 5.6).

You'll also need some libraries:

- [OpenCV](http://opencv.org) (tested with version 3.4.1)

For the point cloud viewer you might also want to include, optionally:

- [PCL](http://www.pointclouds.org) (tested with version 1.8.1)

### Apple macOS

All the dependencies can be obtained via homebrew. A [Brewfile](./Brewfile) is included so just run:

```
brew update && brew bundle
```

### Windows 64 bits

Since there are no official binaries from VTK and PCL you can build everything using
[vcpkg](https://github.com/Microsoft/vcpkg). It greatly simplifies the task, trust me.

```
vcpkg install opencv:x64-windows
vcpkg install pcl:x64-windows
```

If you have `vcpkg` installed at `C:\Tools\vcpkg\` then there's nothing else to do. If you have it
somewhere else you'll have to update all the include and lib paths.

To run the applications you also need to add the binaries (*.dll) to the `PATH`.
Open the _Projects_ tab in the left toolbar of QtCreator and go to the _Build_ (or _Run_) configurations.
In the environment variables, add to the `PATH` the folder where you have the third party binaries.
You should use the _Batch Edit_ button to make it easier. In that case:
- for a _release_ (or _profile_) build you would add:
  - ```PATH=${PATH};C:\Tools\vcpkg\installed\x64-windows\bin;```
- for a _debug_ build you would add:
  - ```PATH=${PATH};C:\Tools\vcpkg\installed\x64-windows\debug\bin;```

> It's really important that you add only the ones corresponding to the build configuration you're
> currently trying to build/run!

### Others

For Linux and other Windows setups you will have to update the paths so Qt can find each library to
compile/link/run the application. Check the `*.pri` files in the [3rdparty](./3rdparty/) folder.

## Applications

### 3D scanner

To run the 3D scanner app, just open the Qt Creator project file [Z3DScanner.pro](./Z3DScanner.pro)

> You'll need a configuration file, there's none included right now so it will NOT just work.
> Contact me if you need help.

### Multi camera calibration

To run the stereo calibration app, open the Qt Creator project file [Z3DMultiCameraCalibration.pro](./Z3DMultiCameraCalibration.pro)
