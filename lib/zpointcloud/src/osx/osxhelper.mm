#include <Cocoa/Cocoa.h>
#include "osxhelper.h"

// http://public.kitware.com/pipermail/vtkusers/2015-February/090117.html
void disableGLHiDPI( long id )
{
    NSView* view = reinterpret_cast<NSView*>( id );
    [view setWantsBestResolutionOpenGLSurface:NO];
}
