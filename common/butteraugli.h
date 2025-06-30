/*****************************************************************************
 * butteraugli.h: butteraugli perceptual metric integration
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

#ifndef X264_BUTTERAUGLI_H
#define X264_BUTTERAUGLI_H

#include "common.h"

typedef struct x264_butteraugli_t x264_butteraugli_t;

/* Butteraugli context creation and destruction */
x264_butteraugli_t *x264_butteraugli_create( int width, int height, int num_threads );
void x264_butteraugli_destroy( x264_butteraugli_t *ba );

/* Compute butteraugli distance between two images
 * Returns the butteraugli distance score (0 = identical, 1 = just noticeable difference)
 * Higher values indicate more perceptual difference */
float x264_butteraugli_compute( x264_butteraugli_t *ba,
                                const uint8_t *ref_yuv[3], int ref_stride[3],
                                const uint8_t *enc_yuv[3], int enc_stride[3] );

/* Compute butteraugli heatmap showing perceptual importance of different regions
 * Output is a single-channel map with values 0-255 indicating importance */
void x264_butteraugli_heatmap( x264_butteraugli_t *ba,
                               const uint8_t *ref_yuv[3], int ref_stride[3],
                               const uint8_t *enc_yuv[3], int enc_stride[3],
                               uint8_t *heatmap, int heatmap_stride );

/* Get visual masking map for adaptive quantization
 * Output values range from 0.5 to 2.0 (multiplicative QP adjustment) */
void x264_butteraugli_visual_mask( x264_butteraugli_t *ba,
                                   const uint8_t *ref_yuv[3], int ref_stride[3],
                                   float *mask, int mask_stride );

/* Compute perceptual weight for a specific macroblock
 * Returns weight value 0.0-1.0 indicating perceptual importance */
float x264_butteraugli_mb_weight( x264_butteraugli_t *ba,
                                  const uint8_t *ref_yuv[3], int ref_stride[3],
                                  const uint8_t *enc_yuv[3], int enc_stride[3],
                                  int mb_x, int mb_y );

/* Configuration structure for butteraugli parameters */
typedef struct
{
    float target_distance;    /* Target butteraugli distance (0.5-3.0) */
    float strength;          /* Influence on RD decisions (0.0-1.0) */
    int   aq_mode;          /* 0=off, 1=variance, 2=autovariance */
    int   norm;             /* 3=L3, 12=L12, 99=max */
    int   enable_heatmap;   /* Generate heatmaps for analysis */
} x264_butteraugli_config_t;

/* Apply butteraugli configuration */
void x264_butteraugli_configure( x264_butteraugli_t *ba, const x264_butteraugli_config_t *config );

/* Thread-safe tile processing for parallel encoding */
float x264_butteraugli_compute_tile( x264_butteraugli_t *ba,
                                     const uint8_t *ref_yuv[3], int ref_stride[3],
                                     const uint8_t *enc_yuv[3], int enc_stride[3],
                                     int tile_x, int tile_y, int tile_width, int tile_height );

#endif /* X264_BUTTERAUGLI_H */