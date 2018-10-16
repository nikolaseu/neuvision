# Basler pylon SDK plugin

A plugin for making [Basler pylon SDK](https://www.baslerweb.com) available for camera acquisition.

Implements not only acquisition of images but also reading & setting of all camera properties. Check
it with the camera viewer example.

## Requirements

The plugin requires having the Basler pylon SDK available. The plugin will be enabled & built whenever the environment variable `PYLON_DEV_DIR` is set.

### Microsoft Windows

> It is tested to work on version 5.0.12.

`PYLON_DEV_DIR` should point to the `Development` folder of the pylon SDK installation.

For example:

```
set PYLON_DEV_DIR="C:\Program Files\Basler\pylon 5\Development"
```

> If you're building the plugin yourself, you need to have the Basler pylon SDK installed including
the development/SDK features, not only the runtime!

### Apple macOS

For example, assuming the pylon SDK was installed as a framework (with the official package):

```
set PYLON_DEV_DIR=/Library
```

And for any application that uses this plugin, this is required in the run environment to be able to load the pylon plugin at runtime (again assuming we have the official pylon SDK package):

```
LD_LIBRARY_PATH=/Library/Frameworks/pylon.framework/Libraries
```
