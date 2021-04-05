# A framework for Structured Light based 3D scanning projects

[![Windows](https://ci.appveyor.com/api/projects/status/q3vr5tbjbh3v8jh5/branch/master?svg=true)](https://ci.appveyor.com/project/nikolaseu/neuvision/branch/master)
[![Windows](https://github.com/nikolaseu/neuvision/actions/workflows/windows.yml/badge.svg)](https://github.com/nikolaseu/neuvision/actions/workflows/windows.yml)
[![Apple macOS](https://github.com/nikolaseu/neuvision/actions/workflows/macos.yml/badge.svg)](https://github.com/nikolaseu/neuvision/actions/workflows/macos.yml)
[![Ubuntu](https://github.com/nikolaseu/neuvision/actions/workflows/ubuntu.yml/badge.svg)](https://github.com/nikolaseu/neuvision/actions/workflows/ubuntu.yml)

Engineering degree thesis/project for the title of "Ingeniero en Informatica", Universidad Nacional del Litoral (Santa Fe, Argentina).

Originally used two cameras + one DLP projector to get more precission, but I'm trying to add a calibration wizard to use it with one camera + one projector, to make it easier/simpler for home-use.

## Camera support

There's a plugin for OpenCV, so any webcam that works with OpenCV should work too.
On Linux & macOS you can also use the libgphoto2 plugin.

There are also [a lot of plugins](./lib/zcameraacquisition/plugins) that (used to) make it work with
a lot of different cameras, mostly industrial GigE or USB cameras, but since I don't have access to
them anymore, they are not tested/enabled anymore.

The windows release should have working plugins for:
- [Basler](https://www.baslerweb.com)
- [AVT / Allied Vision](https://www.alliedvision.com)

There are also some old plugins (might require fixes/changes) for:
- [Point Grey](https://www.ptgrey.com)
- [Pleora Technologies](https://www.pleora.com)

## Requirements

We assume you already have a suitable C++ compiler and [Qt](https://www.qt.io) installed (at least
version 5.15).

You'll also need some libraries:

- [OpenCV](http://opencv.org) (mostly tested with version 4.2.x or higher)

For the point cloud viewer you might also want to include, optionally (just to open PCD files):

- [PCL](http://www.pointclouds.org) (tested with version 1.10.x or higher)

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

### Linux

For Linux it should be easy, just use the official package manager to install the dependencies.

## Building and running

For building you can simply run:

```
mkdir build
cd build
cmake ..
cmake --build .
```

Check the [CI scripts](./scripts) for more details.

You can also open the CMake project with QtCreator (version 4.11 has good support for CMake projects) and just run the applications from there.
