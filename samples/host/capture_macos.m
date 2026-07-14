// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// End-state screenshot readback for --capture. See capture.h for the contract.
//
// The render target lives on sokol's device/queue; readback happens on a
// private queue created here. Cross-queue ordering is safe because the caller
// waits SG_NUM_INFLIGHT_FRAMES+ frames between the capture render and the
// readback, and sokol's frame rotation blocks on command-buffer completion.

#include "capture.h"

#import <CoreGraphics/CoreGraphics.h>
#import <Foundation/Foundation.h>
#import <ImageIO/ImageIO.h>
#import <Metal/Metal.h>

static sg_image s_captureImage;
static sg_view s_captureView;
static sg_attachments s_captureAttachments;
static int s_captureWidth;
static int s_captureHeight;

bool CaptureSupported( void )
{
	return sg_query_backend() == SG_BACKEND_METAL_MACOS;
}

void CaptureCreateTarget( int width, int height )
{
	if ( s_captureImage.id != SG_INVALID_ID && width == s_captureWidth && height == s_captureHeight )
	{
		return;
	}
	if ( s_captureView.id != SG_INVALID_ID )
	{
		sg_destroy_view( s_captureView );
	}
	if ( s_captureImage.id != SG_INVALID_ID )
	{
		sg_destroy_image( s_captureImage );
	}

	sg_image_desc idesc = { 0 };
	idesc.usage.color_attachment = true;
	idesc.width = width;
	idesc.height = height;
	idesc.pixel_format = SG_PIXELFORMAT_BGRA8;
	idesc.sample_count = 1;
	idesc.label = "capture_color";
	s_captureImage = sg_make_image( &idesc );

	sg_view_desc vdesc = { 0 };
	vdesc.color_attachment.image = s_captureImage;
	vdesc.label = "capture_color_attach";
	s_captureView = sg_make_view( &vdesc );

	memset( &s_captureAttachments, 0, sizeof( s_captureAttachments ) );
	s_captureAttachments.colors[0] = s_captureView;

	s_captureWidth = width;
	s_captureHeight = height;
}

const sg_attachments* CaptureAttachments( void )
{
	return &s_captureAttachments;
}

sg_pixel_format CaptureFormat( void )
{
	return SG_PIXELFORMAT_BGRA8;
}

bool CaptureWritePng( const char* path, int width, int height )
{
	if ( !CaptureSupported() || s_captureImage.id == SG_INVALID_ID )
	{
		fprintf( stderr, "capture: unsupported backend or no target\n" );
		return false;
	}

	sg_mtl_image_info info = sg_mtl_query_image_info( s_captureImage );
	id<MTLTexture> tex = (__bridge id<MTLTexture>)info.tex[info.active_slot];
	id<MTLDevice> device = (__bridge id<MTLDevice>)sg_mtl_device();
	if ( tex == nil || device == nil )
	{
		fprintf( stderr, "capture: no metal texture\n" );
		return false;
	}

	const NSUInteger bytesPerRow = (NSUInteger)width * 4;
	const NSUInteger byteCount = bytesPerRow * (NSUInteger)height;
	id<MTLBuffer> readback = [device newBufferWithLength:byteCount options:MTLResourceStorageModeShared];
	id<MTLCommandQueue> queue = [device newCommandQueue];
	id<MTLCommandBuffer> cmd = [queue commandBuffer];
	id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
	[blit copyFromTexture:tex
					 sourceSlice:0
					 sourceLevel:0
					sourceOrigin:MTLOriginMake( 0, 0, 0 )
					  sourceSize:MTLSizeMake( (NSUInteger)width, (NSUInteger)height, 1 )
						toBuffer:readback
			   destinationOffset:0
		  destinationBytesPerRow:bytesPerRow
		destinationBytesPerImage:byteCount];
	[blit endEncoding];
	[cmd commit];
	[cmd waitUntilCompleted];

	// BGRA8 in memory = 32-bit little-endian ARGB words; alpha carries tonemap
	// garbage, so skip it.
	CGColorSpaceRef colorSpace = CGColorSpaceCreateWithName( kCGColorSpaceSRGB );
	CGDataProviderRef provider = CGDataProviderCreateWithData( NULL, readback.contents, byteCount, NULL );
	CGImageRef image = CGImageCreate( (size_t)width, (size_t)height, 8, 32, bytesPerRow, colorSpace,
									  (CGBitmapInfo)kCGBitmapByteOrder32Little | (CGBitmapInfo)kCGImageAlphaNoneSkipFirst,
									  provider, NULL, false, kCGRenderingIntentDefault );

	bool ok = false;
	if ( image != NULL )
	{
		NSURL* url = [NSURL fileURLWithPath:[NSString stringWithUTF8String:path]];
		CGImageDestinationRef dest = CGImageDestinationCreateWithURL( (__bridge CFURLRef)url, CFSTR( "public.png" ), 1, NULL );
		if ( dest != NULL )
		{
			CGImageDestinationAddImage( dest, image, NULL );
			ok = CGImageDestinationFinalize( dest );
			CFRelease( dest );
		}
		CGImageRelease( image );
	}
	CGDataProviderRelease( provider );
	CGColorSpaceRelease( colorSpace );

	if ( !ok )
	{
		fprintf( stderr, "capture: failed to write %s\n", path );
	}
	return ok;
}
