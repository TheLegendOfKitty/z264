/*****************************************************************************
 * butteraugli.h: x86 butteraugli SIMD optimizations
 *****************************************************************************
 * Copyright (C) 2025 x264 project
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at licensing@x264.com.
 *****************************************************************************/

#ifndef X264_X86_BUTTERAUGLI_H
#define X264_X86_BUTTERAUGLI_H

#include "common/butteraugli.h"

/* x86 butteraugli functions - not templated by bit depth */
void x264_butteraugli_yuv_to_rgb_sse2( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                       int y_stride, int uv_stride,
                                       uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                       int width, int height, x264_butteraugli_colorspace_t colorspace );

void x264_butteraugli_yuv_to_rgb_avx2( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                       int y_stride, int uv_stride,
                                       uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                       int width, int height, x264_butteraugli_colorspace_t colorspace );

void x264_butteraugli_srgb_to_linear_sse2( const uint8_t *srgb, float *linear,
                                           int width, int height, int stride );

void x264_butteraugli_srgb_to_linear_avx2( const uint8_t *srgb, float *linear,
                                           int width, int height, int stride );

void x264_butteraugli_create_image_planar_sse2( const uint8_t *rgb_r, const uint8_t *rgb_g, const uint8_t *rgb_b,
                                                int rgb_stride, float *planar_r, float *planar_g, float *planar_b,
                                                int width, int height );

void x264_butteraugli_create_image_planar_avx2( const uint8_t *rgb_r, const uint8_t *rgb_g, const uint8_t *rgb_b,
                                                int rgb_stride, float *planar_r, float *planar_g, float *planar_b,
                                                int width, int height );

void x264_butteraugli_compute_visual_mask_sse2( const float *ref_r, const float *ref_g, const float *ref_b,
                                                int ref_stride, float *mask, int mask_stride,
                                                int width, int height );

void x264_butteraugli_compute_visual_mask_avx2( const float *ref_r, const float *ref_g, const float *ref_b,
                                                int ref_stride, float *mask, int mask_stride,
                                                int width, int height );

void x264_butteraugli_heatmap_to_uint8_sse2( const float *heatmap, int heatmap_stride,
                                             uint8_t *output, int output_stride,
                                             int width, int height, float scale );

void x264_butteraugli_heatmap_to_uint8_avx2( const float *heatmap, int heatmap_stride,
                                             uint8_t *output, int output_stride,
                                             int width, int height, float scale );

/* AVX-512 butteraugli functions */
void x264_butteraugli_yuv_to_rgb_avx512( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                         int y_stride, int uv_stride,
                                         uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                         int width, int height, x264_butteraugli_colorspace_t colorspace );

void x264_butteraugli_srgb_to_linear_avx512( const uint8_t *srgb, float *linear,
                                             int width, int height, int stride );

void x264_butteraugli_create_image_planar_avx512( const uint8_t *rgb_r, const uint8_t *rgb_g, const uint8_t *rgb_b,
                                                  int rgb_stride, float *planar_r, float *planar_g, float *planar_b,
                                                  int width, int height );

void x264_butteraugli_compute_visual_mask_avx512( const float *ref_r, const float *ref_g, const float *ref_b,
                                                  int ref_stride, float *mask, int mask_stride,
                                                  int width, int height );

void x264_butteraugli_heatmap_to_uint8_avx512( const float *heatmap, int heatmap_stride,
                                               uint8_t *output, int output_stride,
                                               int width, int height, float scale );

#endif