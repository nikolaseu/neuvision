# Basler pylon SDK plugin

A plugin for making [Basler pylon SDK](https://www.baslerweb.com) available for camera acquisition.

Implements not only acquisition of images but also reading & setting of all camera properties. Check
it with the camera viewer example.

## Requirements

The plugin requires having the Basler pylon SDK available. It is tested to work on version 5.0.12.

The plugin will be enabled & built whenever the environment variable `PYLON_DEV_DIR` is set.
It should point to the `Development` folder of the pylon SDK installation.

An example for Windows:

```
set PYLON_DEV_DIR="C:\Program Files\Basler\pylon 5\Development"
```

> If you're building the plugin yourself, you need to have the Basler pylon SDK installed including
the development/SDK features, not only the runtime!
