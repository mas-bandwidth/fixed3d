// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "sokol_gfx.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

// One-shot end-state screenshot support for the samples app's --capture flag
// (macOS/Metal only). The final simulated frame is rendered a second time via
// RenderFrameOffscreen into a private BGRA8 target owned here; the app then
// runs a few more (paused) frames so sokol's in-flight rotation guarantees the
// GPU finished with it, and CaptureWritePng blits the texture to CPU memory on
// a private queue and encodes a PNG with ImageIO.

// True when the active backend supports the readback path.
bool CaptureSupported( void );

// Create (or recreate) the offscreen color target and its attachment view.
void CaptureCreateTarget( int width, int height );

// Pass these to RenderFrameOffscreen.
const sg_attachments* CaptureAttachments( void );
sg_pixel_format CaptureFormat( void );

// Blit the target to CPU memory and write a PNG. Call only after the GPU is
// done with the capture render (wait >= SG_NUM_INFLIGHT_FRAMES frames).
bool CaptureWritePng( const char* path, int width, int height );

#ifdef __cplusplus
}
#endif
