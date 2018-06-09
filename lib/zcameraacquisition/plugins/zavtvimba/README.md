# Allied Vision Vimba SDK plugin

A plugin for making [AVT Vimba SDK](https://www.alliedvision.com) available for camera acquisition.

Implements not only acquisition of images but also reading & setting of all camera properties. Check
it with the camera viewer example.

## Requirements

The plugin requires having the AVT Vimba SDK available. It is tested to work on version 2.1.3.

The plugin will be enabled & built whenever the environment variable `AVT_VIMBA_DIR` is set.

An example for Windows:

```
set AVT_VIMBA_DIR="C:\Program Files\Allied Vision\Vimba_2.1"
```

> If you're building the plugin yourself, you need to have the AVT Vimba SDK installed including
the C++ development/SDK features, not only the runtime!

## FAQ

### Windows

If the plugin fails to load with the error message `The specified module could not be found.` check
that both `VimbaC.dll` and `VimbaCPP.dll` files are available in the `PATH` or copy them to the
application folder manually. You can find them in the SDK installation path (for example in
`C:\Program Files\Allied Vision\Vimba_2.1\VimbaCPP\Bin\Win64`).
