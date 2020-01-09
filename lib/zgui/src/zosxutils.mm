//
// Z3D - A structured light 3D scanner
// Copyright (C) 2013-2016 Nicolas Ulrich <nikolaseu@gmail.com>
//
// This file is part of Z3D.
//
// Z3D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Z3D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Z3D.  If not, see <http://www.gnu.org/licenses/>.
//

#include "ZGui/zosxutils.h"

#include <Cocoa/Cocoa.h>

namespace Z3D {
namespace ZOSXUtils {

void hideTitleBar(unsigned long long winid)
{
    NSView *nativeView = reinterpret_cast<NSView *>(winid);
    NSWindow* nativeWindow = [nativeView window];

    [nativeWindow setTitleVisibility:NSWindowTitleHidden];

    [nativeWindow setStyleMask:
        [nativeWindow styleMask] | NSWindowStyleMaskFullSizeContentView | NSWindowTitleHidden];

    [nativeWindow setTitlebarAppearsTransparent:YES];
    //[nativeWindow setMovableByWindowBackground:YES];
}

void transparentTitleBar(unsigned long long winid)
{
    NSView *nativeView = reinterpret_cast<NSView *>(winid);
    NSWindow* nativeWindow = [nativeView window];
    [nativeWindow setTitlebarAppearsTransparent:YES];
}

void applyTranslucentBackgroundEffect(unsigned long long winid)
{
    NSView *nativeView = reinterpret_cast<NSView *>(winid);
    NSWindow* nativeWindow = [nativeView window];

    /// credits to https://bugreports.qt.io/browse/QTBUG-39463
    /// FIXME it makes app crash on maximize
    /// FIXME it does not close the app anymore when window is closed
    NSVisualEffectView * vibrant = [[NSVisualEffectView alloc] init];
    if (@available(macOS 10.14, *)) {
        vibrant.material = NSVisualEffectMaterialUnderWindowBackground;
    } else {
        vibrant.material = NSVisualEffectMaterialMediumLight;
    }
    vibrant.blendingMode = NSVisualEffectBlendingModeBehindWindow;
    vibrant.state = NSVisualEffectStateFollowsWindowActiveState;
    [vibrant setAutoresizingMask:NSViewWidthSizable|NSViewHeightSizable];

    NSView* qtView = nativeWindow.contentView;
    [qtView setAutoresizingMask:NSViewWidthSizable|NSViewHeightSizable];
    nativeWindow.contentView = vibrant;
    [vibrant addSubview:qtView positioned:NSWindowBelow relativeTo:nil];
}

} // namespace ZOSXUtils
} // namespace Z3D
