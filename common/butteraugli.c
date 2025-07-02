/*****************************************************************************
 * butteraugli.c: butteraugli perceptual metric integration
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

#include "common.h"
#include "butteraugli.h"

#if HAVE_MMX
#include "x86/butteraugli.h"
#endif
#if HAVE_ARMV6 || HAVE_AARCH64
#include "aarch64/butteraugli.h"
#endif

#ifdef HAVE_BUTTERAUGLI

/* Forward declarations for C++ butteraugli functions */
#ifdef __cplusplus
extern "C" {
#endif

void* butteraugli_create_context( int width, int height, int num_threads );
void butteraugli_destroy_context( void* context );
float butteraugli_compute_distance( void* context,
                                   const uint8_t* ref_r, const uint8_t* ref_g, const uint8_t* ref_b,
                                   int ref_stride,
                                   const uint8_t* enc_r, const uint8_t* enc_g, const uint8_t* enc_b,
                                   int enc_stride );
void butteraugli_compute_heatmap( void* context,
                                 const uint8_t* ref_r, const uint8_t* ref_g, const uint8_t* ref_b,
                                 int ref_stride,
                                 const uint8_t* enc_r, const uint8_t* enc_g, const uint8_t* enc_b,
                                 int enc_stride,
                                 float* heatmap, int heatmap_stride );
void butteraugli_compute_mask( void* context,
                              const uint8_t* ref_r, const uint8_t* ref_g, const uint8_t* ref_b,
                              int ref_stride,
                              float* mask, int mask_stride );
double butteraugli_fuzzy_class( double score );
double butteraugli_fuzzy_inverse( double seek );
int butteraugli_adaptive_quantization( int width, int height,
                                      const uint8_t* rgb_r, const uint8_t* rgb_g, const uint8_t* rgb_b,
                                      int rgb_stride, float* quant_map, int quant_stride );

#ifdef __cplusplus
}
#endif

struct x264_butteraugli_t
{
    void *context;          /* Butteraugli C++ context */
    int width;
    int height;
    int num_threads;
    
    /* Configuration */
    x264_butteraugli_config_t config;
    
    /* Global buffers for full-frame operations (protected by mutex) */
    uint8_t *rgb_ref;
    uint8_t *rgb_enc;
    int rgb_stride;
    
    /* Cache for visual masking */
    float *visual_mask_cache;
    int mask_valid;
    
    /* Thread synchronization */
    x264_pthread_mutex_t mutex;
};

/* YUV to RGB conversion coefficients for different color spaces */
typedef struct
{
    int16_t coeff_r_v;      /* R = Y + coeff_r_v * V */
    int16_t coeff_g_u;      /* G = Y - coeff_g_u * U - coeff_g_v * V */
    int16_t coeff_g_v;
    int16_t coeff_b_u;      /* B = Y + coeff_b_u * U */
} yuv_to_rgb_coeffs_t;

/* Pre-computed coefficients scaled by 256 for fixed-point arithmetic */
static const yuv_to_rgb_coeffs_t yuv_coeffs[3] = {
    /* BT.601 */
    { 360, -88, -184, 455 },
    /* BT.709 */
    { 404, -48, -120, 475 },
    /* BT.2020 */
    { 377, -42, -133, 482 }
};

/* Scalar YUV to RGB conversion with planar output */
static void butteraugli_yuv_to_rgb_c( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                      int y_stride, int uv_stride,
                                      uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                      int width, int height, x264_butteraugli_colorspace_t colorspace )
{
    const yuv_to_rgb_coeffs_t *coeffs = &yuv_coeffs[colorspace];
    
    for( int j = 0; j < height; j++ )
    {
        for( int i = 0; i < width; i++ )
        {
            int yi = y[j * y_stride + i];
            int ui = u[(j/2) * uv_stride + (i/2)] - 128;
            int vi = v[(j/2) * uv_stride + (i/2)] - 128;
            
            int r = yi + ((coeffs->coeff_r_v * vi) >> 8);
            int g = yi - ((coeffs->coeff_g_u * ui + coeffs->coeff_g_v * vi) >> 8);
            int b = yi + ((coeffs->coeff_b_u * ui) >> 8);
            
            rgb_r[j * rgb_stride + i] = x264_clip3( r, 0, 255 );
            rgb_g[j * rgb_stride + i] = x264_clip3( g, 0, 255 );
            rgb_b[j * rgb_stride + i] = x264_clip3( b, 0, 255 );
        }
    }
}

/* sRGB to linear conversion - scalar implementation */
static void butteraugli_srgb_to_linear_c( const uint8_t *srgb, float *linear,
                                          int width, int height, int stride )
{
    for( int j = 0; j < height; j++ )
    {
        for( int i = 0; i < width; i++ )
        {
            float val = srgb[j * stride + i] / 255.0f;
            if( val <= 0.04045f )
                linear[j * width + i] = val / 12.92f;
            else
                linear[j * width + i] = powf( (val + 0.055f) / 1.055f, 2.4f );
        }
    }
}

/* Create planar float image from RGB - scalar implementation */
static void butteraugli_create_image_planar_c( const uint8_t *rgb_r, const uint8_t *rgb_g, const uint8_t *rgb_b,
                                               int rgb_stride, float *planar_r, float *planar_g, float *planar_b,
                                               int width, int height )
{
    for( int j = 0; j < height; j++ )
    {
        for( int i = 0; i < width; i++ )
        {
            planar_r[j * width + i] = (float)rgb_r[j * rgb_stride + i];
            planar_g[j * width + i] = (float)rgb_g[j * rgb_stride + i];
            planar_b[j * width + i] = (float)rgb_b[j * rgb_stride + i];
        }
    }
}

/* Compute visual mask - scalar implementation */
static void butteraugli_compute_visual_mask_c( const float *ref_r, const float *ref_g, const float *ref_b,
                                              int ref_stride, float *mask, int mask_stride,
                                              int width, int height )
{
    /* Set borders to neutral masking value */
    for( int y = 0; y < height; y++ )
    {
        mask[y * mask_stride + 0] = 1.0f;
        mask[y * mask_stride + width - 1] = 1.0f;
    }
    for( int x = 0; x < width; x++ )
    {
        mask[0 * mask_stride + x] = 1.0f;
        mask[(height - 1) * mask_stride + x] = 1.0f;
    }
    
    /* Compute variance-based masking for inner pixels */
    for( int y = 1; y < height - 1; y++ )
    {
        for( int x = 1; x < width - 1; x++ )
        {
            float sum = 0.0f;
            float sum_sq = 0.0f;
            
            /* Compute statistics over 3x3 neighborhood */
            for( int dy = -1; dy <= 1; dy++ )
            {
                for( int dx = -1; dx <= 1; dx++ )
                {
                    int offset = (y + dy) * ref_stride + (x + dx);
                    /* BT.709 luma coefficients */
                    float luma = 0.2126f * ref_r[offset] + 0.7152f * ref_g[offset] + 0.0722f * ref_b[offset];
                    sum += luma;
                    sum_sq += luma * luma;
                }
            }
            
            float mean = sum / 9.0f;
            float variance = (sum_sq / 9.0f) - (mean * mean);
            
            /* Map variance to masking strength */
            float masking = 1.0f + expf(-variance * 100.0f);
            mask[y * mask_stride + x] = fminf(2.0f, fmaxf(0.5f, masking));
        }
    }
}

/* Convert heatmap to uint8 - scalar implementation */
static void butteraugli_heatmap_to_uint8_c( const float *heatmap, int heatmap_stride,
                                            uint8_t *output, int output_stride,
                                            int width, int height, float scale )
{
    for( int y = 0; y < height; y++ )
    {
        for( int x = 0; x < width; x++ )
        {
            float val = heatmap[y * heatmap_stride + x] * scale;
            output[y * output_stride + x] = (uint8_t)x264_clip3f( val, 0.0f, 255.0f );
        }
    }
}

/* YUV to RGB conversion for butteraugli input (legacy - interleaved output) */
static void yuv_to_rgb( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                       int y_stride, int uv_stride,
                       uint8_t *rgb, int rgb_stride,
                       int width, int height )
{
    for( int j = 0; j < height; j++ )
    {
        for( int i = 0; i < width; i++ )
        {
            int yi = y[j * y_stride + i];
            int ui = u[(j/2) * uv_stride + (i/2)] - 128;
            int vi = v[(j/2) * uv_stride + (i/2)] - 128;
            
            int r = yi + ((360 * vi) >> 8);
            int g = yi - ((88 * ui + 184 * vi) >> 8);
            int b = yi + ((455 * ui) >> 8);
            
            rgb[j * rgb_stride + i * 3 + 0] = x264_clip3( r, 0, 255 );
            rgb[j * rgb_stride + i * 3 + 1] = x264_clip3( g, 0, 255 );
            rgb[j * rgb_stride + i * 3 + 2] = x264_clip3( b, 0, 255 );
        }
    }
}

x264_butteraugli_t *x264_butteraugli_create( int width, int height, int num_threads )
{
    x264_butteraugli_t *ba = x264_malloc( sizeof(x264_butteraugli_t) );
    if( !ba )
        return NULL;
    
    ba->width = width;
    ba->height = height;
    ba->num_threads = num_threads;
    
    /* Create butteraugli context */
    ba->context = butteraugli_create_context( width, height, num_threads );
    if( !ba->context )
    {
        x264_free( ba );
        return NULL;
    }
    
    /* Allocate RGB conversion buffers */
    ba->rgb_stride = ALIGN( width * 3, 16 );
    ba->rgb_ref = x264_malloc( ba->rgb_stride * height );
    ba->rgb_enc = x264_malloc( ba->rgb_stride * height );
    if( !ba->rgb_ref || !ba->rgb_enc )
    {
        butteraugli_destroy_context( ba->context );
        x264_free( ba->rgb_ref );
        x264_free( ba->rgb_enc );
        x264_free( ba );
        return NULL;
    }
    
    /* Allocate visual mask cache */
    ba->visual_mask_cache = x264_malloc( width * height * sizeof(float) );
    if( !ba->visual_mask_cache )
    {
        butteraugli_destroy_context( ba->context );
        x264_free( ba->rgb_ref );
        x264_free( ba->rgb_enc );
        x264_free( ba );
        return NULL;
    }
    ba->mask_valid = 0;
    
    /* Initialize configuration with defaults */
    ba->config.target_distance = 1.0f;
    ba->config.strength = 0.5f;
    ba->config.aq_mode = 2;
    ba->config.norm = 3;
    ba->config.enable_heatmap = 0;
    
    x264_pthread_mutex_init( &ba->mutex, NULL );
    
    return ba;
}

void x264_butteraugli_destroy( x264_butteraugli_t *ba )
{
    if( !ba )
        return;
    
    x264_pthread_mutex_destroy( &ba->mutex );
    butteraugli_destroy_context( ba->context );
    x264_free( ba->rgb_ref );
    x264_free( ba->rgb_enc );
    x264_free( ba->visual_mask_cache );
    x264_free( ba );
}

float x264_butteraugli_compute( x264_butteraugli_t *ba,
                               const uint8_t *ref_yuv[3], int ref_stride[3],
                               const uint8_t *enc_yuv[3], int enc_stride[3] )
{
    x264_pthread_mutex_lock( &ba->mutex );
    
    /* Convert YUV to RGB */
    yuv_to_rgb( ref_yuv[0], ref_yuv[1], ref_yuv[2],
                ref_stride[0], ref_stride[1],
                ba->rgb_ref, ba->rgb_stride,
                ba->width, ba->height );
    
    yuv_to_rgb( enc_yuv[0], enc_yuv[1], enc_yuv[2],
                enc_stride[0], enc_stride[1],
                ba->rgb_enc, ba->rgb_stride,
                ba->width, ba->height );
    
    /* Compute butteraugli distance */
    float distance = butteraugli_compute_distance( ba->context,
                                                  ba->rgb_ref, ba->rgb_ref + 1, ba->rgb_ref + 2,
                                                  ba->rgb_stride,
                                                  ba->rgb_enc, ba->rgb_enc + 1, ba->rgb_enc + 2,
                                                  ba->rgb_stride );
    
    /* Invalidate mask cache since reference changed */
    ba->mask_valid = 0;
    
    x264_pthread_mutex_unlock( &ba->mutex );
    
    return distance;
}

void x264_butteraugli_heatmap( x264_butteraugli_t *ba,
                              const uint8_t *ref_yuv[3], int ref_stride[3],
                              const uint8_t *enc_yuv[3], int enc_stride[3],
                              uint8_t *heatmap, int heatmap_stride )
{
    x264_pthread_mutex_lock( &ba->mutex );
    
    /* Convert YUV to RGB */
    yuv_to_rgb( ref_yuv[0], ref_yuv[1], ref_yuv[2],
                ref_stride[0], ref_stride[1],
                ba->rgb_ref, ba->rgb_stride,
                ba->width, ba->height );
    
    yuv_to_rgb( enc_yuv[0], enc_yuv[1], enc_yuv[2],
                enc_stride[0], enc_stride[1],
                ba->rgb_enc, ba->rgb_stride,
                ba->width, ba->height );
    
    /* Temporary float heatmap */
    float *temp_heatmap = x264_malloc( ba->width * ba->height * sizeof(float) );
    if( temp_heatmap )
    {
        butteraugli_compute_heatmap( ba->context,
                                    ba->rgb_ref, ba->rgb_ref + 1, ba->rgb_ref + 2,
                                    ba->rgb_stride,
                                    ba->rgb_enc, ba->rgb_enc + 1, ba->rgb_enc + 2,
                                    ba->rgb_stride,
                                    temp_heatmap, ba->width );
        
        /* Convert float heatmap to uint8_t */
        for( int y = 0; y < ba->height; y++ )
        {
            for( int x = 0; x < ba->width; x++ )
            {
                float value = temp_heatmap[y * ba->width + x];
                /* Scale and clip to 0-255 range */
                value = value * 255.0f / ba->config.target_distance;
                heatmap[y * heatmap_stride + x] = x264_clip3( (int)value, 0, 255 );
            }
        }
        
        x264_free( temp_heatmap );
    }
    
    x264_pthread_mutex_unlock( &ba->mutex );
}

void x264_butteraugli_visual_mask( x264_butteraugli_t *ba,
                                  const uint8_t *ref_yuv[3], int ref_stride[3],
                                  float *mask, int mask_stride )
{
    x264_pthread_mutex_lock( &ba->mutex );
    
    /* Check if we need to recompute the mask */
    if( !ba->mask_valid )
    {
        /* Convert YUV to RGB */
        yuv_to_rgb( ref_yuv[0], ref_yuv[1], ref_yuv[2],
                    ref_stride[0], ref_stride[1],
                    ba->rgb_ref, ba->rgb_stride,
                    ba->width, ba->height );
        
        /* Compute visual mask */
        butteraugli_compute_mask( ba->context,
                                 ba->rgb_ref, ba->rgb_ref + 1, ba->rgb_ref + 2,
                                 ba->rgb_stride,
                                 ba->visual_mask_cache, ba->width );
        
        ba->mask_valid = 1;
    }
    
    /* Copy cached mask to output */
    for( int y = 0; y < ba->height; y++ )
    {
        memcpy( mask + y * mask_stride,
                ba->visual_mask_cache + y * ba->width,
                ba->width * sizeof(float) );
    }
    
    x264_pthread_mutex_unlock( &ba->mutex );
}

float x264_butteraugli_mb_weight( x264_butteraugli_t *ba,
                                 const uint8_t *ref_yuv[3], int ref_stride[3],
                                 const uint8_t *enc_yuv[3], int enc_stride[3],
                                 int mb_x, int mb_y )
{
    /* Compute butteraugli for a 16x16 macroblock region */
    int x = mb_x * 16;
    int y = mb_y * 16;
    int width = X264_MIN( 16, ba->width - x );
    int height = X264_MIN( 16, ba->height - y );
    
    if( width <= 0 || height <= 0 )
        return 0.0f;
    
    /* For 16x16 MB, use thread-local buffers to avoid allocation overhead */
    if( width == 16 && height == 16 )
    {
        /* Allocate thread-local buffers on stack */
        ALIGNED_ARRAY_32( uint8_t, rgb_ref_local, [16*16*3] );
        ALIGNED_ARRAY_32( uint8_t, rgb_enc_local, [16*16*3] );
        
        /* Convert MB to RGB using thread-local buffers */
        for( int j = 0; j < 16; j++ )
        {
            for( int i = 0; i < 16; i++ )
            {
                int src_x = x + i;
                int src_y = y + j;
                
                int yi_ref = ref_yuv[0][src_y * ref_stride[0] + src_x];
                int ui_ref = ref_yuv[1][(src_y/2) * ref_stride[1] + (src_x/2)] - 128;
                int vi_ref = ref_yuv[2][(src_y/2) * ref_stride[2] + (src_x/2)] - 128;
                
                int yi_enc = enc_yuv[0][src_y * enc_stride[0] + src_x];
                int ui_enc = enc_yuv[1][(src_y/2) * enc_stride[1] + (src_x/2)] - 128;
                int vi_enc = enc_yuv[2][(src_y/2) * enc_stride[2] + (src_x/2)] - 128;
                
                /* Reference RGB */
                int r = yi_ref + ((360 * vi_ref) >> 8);
                int g = yi_ref - ((88 * ui_ref + 184 * vi_ref) >> 8);
                int b = yi_ref + ((455 * ui_ref) >> 8);
                
                rgb_ref_local[j * 16 * 3 + i * 3 + 0] = x264_clip3( r, 0, 255 );
                rgb_ref_local[j * 16 * 3 + i * 3 + 1] = x264_clip3( g, 0, 255 );
                rgb_ref_local[j * 16 * 3 + i * 3 + 2] = x264_clip3( b, 0, 255 );
                
                /* Encoded RGB */
                r = yi_enc + ((360 * vi_enc) >> 8);
                g = yi_enc - ((88 * ui_enc + 184 * vi_enc) >> 8);
                b = yi_enc + ((455 * ui_enc) >> 8);
                
                rgb_enc_local[j * 16 * 3 + i * 3 + 0] = x264_clip3( r, 0, 255 );
                rgb_enc_local[j * 16 * 3 + i * 3 + 1] = x264_clip3( g, 0, 255 );
                rgb_enc_local[j * 16 * 3 + i * 3 + 2] = x264_clip3( b, 0, 255 );
            }
        }
        
        /* Create temporary context for MB processing */
        void *mb_context = butteraugli_create_context( 16, 16, 1 );
        float mb_distance = 0.0f;
        
        if( mb_context )
        {
            mb_distance = butteraugli_compute_distance( mb_context,
                                                       rgb_ref_local, rgb_ref_local + 1, rgb_ref_local + 2,
                                                       16 * 3,
                                                       rgb_enc_local, rgb_enc_local + 1, rgb_enc_local + 2,
                                                       16 * 3 );
            butteraugli_destroy_context( mb_context );
        }
        
        /* Convert distance to weight (inverse relationship) */
        float weight = 1.0f / (1.0f + mb_distance / ba->config.target_distance);
        
        /* Apply strength factor */
        weight = 1.0f - ba->config.strength * (1.0f - weight);
        
        return weight;
    }
    else
    {
        /* For partial MBs, use tile computation */
        float mb_distance = x264_butteraugli_compute_tile( ba,
                                                           ref_yuv, ref_stride,
                                                           enc_yuv, enc_stride,
                                                           x, y, width, height );
        
        /* Convert distance to weight (inverse relationship) */
        float weight = 1.0f / (1.0f + mb_distance / ba->config.target_distance);
        
        /* Apply strength factor */
        weight = 1.0f - ba->config.strength * (1.0f - weight);
        
        return weight;
    }
}

void x264_butteraugli_configure( x264_butteraugli_t *ba, const x264_butteraugli_config_t *config )
{
    x264_pthread_mutex_lock( &ba->mutex );
    ba->config = *config;
    ba->mask_valid = 0;  /* Invalidate cache when config changes */
    x264_pthread_mutex_unlock( &ba->mutex );
}

float x264_butteraugli_compute_tile( x264_butteraugli_t *ba,
                                    const uint8_t *ref_yuv[3], int ref_stride[3],
                                    const uint8_t *enc_yuv[3], int enc_stride[3],
                                    int tile_x, int tile_y, int tile_width, int tile_height )
{
    /* Temporary buffers for tile */
    int rgb_tile_stride = ALIGN( tile_width * 3, 16 );
    uint8_t *rgb_ref_tile = x264_malloc( rgb_tile_stride * tile_height );
    uint8_t *rgb_enc_tile = x264_malloc( rgb_tile_stride * tile_height );
    
    if( !rgb_ref_tile || !rgb_enc_tile )
    {
        x264_free( rgb_ref_tile );
        x264_free( rgb_enc_tile );
        return 0.0f;
    }
    
    /* Extract and convert tile to RGB */
    for( int y = 0; y < tile_height; y++ )
    {
        for( int x = 0; x < tile_width; x++ )
        {
            int src_x = tile_x + x;
            int src_y = tile_y + y;
            
            int yi_ref = ref_yuv[0][src_y * ref_stride[0] + src_x];
            int ui_ref = ref_yuv[1][(src_y/2) * ref_stride[1] + (src_x/2)] - 128;
            int vi_ref = ref_yuv[2][(src_y/2) * ref_stride[2] + (src_x/2)] - 128;
            
            int yi_enc = enc_yuv[0][src_y * enc_stride[0] + src_x];
            int ui_enc = enc_yuv[1][(src_y/2) * enc_stride[1] + (src_x/2)] - 128;
            int vi_enc = enc_yuv[2][(src_y/2) * enc_stride[2] + (src_x/2)] - 128;
            
            /* Reference RGB */
            int r = yi_ref + ((360 * vi_ref) >> 8);
            int g = yi_ref - ((88 * ui_ref + 184 * vi_ref) >> 8);
            int b = yi_ref + ((455 * ui_ref) >> 8);
            
            rgb_ref_tile[y * rgb_tile_stride + x * 3 + 0] = x264_clip3( r, 0, 255 );
            rgb_ref_tile[y * rgb_tile_stride + x * 3 + 1] = x264_clip3( g, 0, 255 );
            rgb_ref_tile[y * rgb_tile_stride + x * 3 + 2] = x264_clip3( b, 0, 255 );
            
            /* Encoded RGB */
            r = yi_enc + ((360 * vi_enc) >> 8);
            g = yi_enc - ((88 * ui_enc + 184 * vi_enc) >> 8);
            b = yi_enc + ((455 * ui_enc) >> 8);
            
            rgb_enc_tile[y * rgb_tile_stride + x * 3 + 0] = x264_clip3( r, 0, 255 );
            rgb_enc_tile[y * rgb_tile_stride + x * 3 + 1] = x264_clip3( g, 0, 255 );
            rgb_enc_tile[y * rgb_tile_stride + x * 3 + 2] = x264_clip3( b, 0, 255 );
        }
    }
    
    /* Create temporary context for tile processing */
    void *tile_context = butteraugli_create_context( tile_width, tile_height, 1 );
    float distance = 0.0f;
    
    if( tile_context )
    {
        distance = butteraugli_compute_distance( tile_context,
                                               rgb_ref_tile, rgb_ref_tile + 1, rgb_ref_tile + 2,
                                               rgb_tile_stride,
                                               rgb_enc_tile, rgb_enc_tile + 1, rgb_enc_tile + 2,
                                               rgb_tile_stride );
        butteraugli_destroy_context( tile_context );
    }
    
    x264_free( rgb_ref_tile );
    x264_free( rgb_enc_tile );
    
    return distance;
}

float x264_butteraugli_fuzzy_class( float score )
{
    return (float)butteraugli_fuzzy_class( (double)score );
}

float x264_butteraugli_fuzzy_inverse( float seek )
{
    return (float)butteraugli_fuzzy_inverse( (double)seek );
}

int x264_butteraugli_adaptive_quantization( x264_butteraugli_t *ba,
                                           const uint8_t *yuv[3], int stride[3],
                                           float *quant_map, int quant_stride )
{
    if( !ba || !ba->context )
        return 0;
    
    x264_pthread_mutex_lock( &ba->mutex );
    
    /* Convert YUV to RGB */
    yuv_to_rgb( yuv[0], yuv[1], yuv[2],
                stride[0], stride[1],
                ba->rgb_ref, ba->rgb_stride,
                ba->width, ba->height );
    
    /* Call butteraugli adaptive quantization with planar RGB */
    int result = butteraugli_adaptive_quantization( ba->width, ba->height,
                                                   ba->rgb_ref, ba->rgb_ref + 1, ba->rgb_ref + 2,
                                                   ba->rgb_stride,
                                                   quant_map, quant_stride );
    
    x264_pthread_mutex_unlock( &ba->mutex );
    
    return result;
}

#else /* !HAVE_BUTTERAUGLI */

/* Stub implementations when butteraugli is not available */

x264_butteraugli_t *x264_butteraugli_create( int width, int height, int num_threads )
{
    return NULL;
}

void x264_butteraugli_destroy( x264_butteraugli_t *ba )
{
}

float x264_butteraugli_compute( x264_butteraugli_t *ba,
                               const uint8_t *ref_yuv[3], int ref_stride[3],
                               const uint8_t *enc_yuv[3], int enc_stride[3] )
{
    return 0.0f;
}

void x264_butteraugli_heatmap( x264_butteraugli_t *ba,
                              const uint8_t *ref_yuv[3], int ref_stride[3],
                              const uint8_t *enc_yuv[3], int enc_stride[3],
                              uint8_t *heatmap, int heatmap_stride )
{
}

void x264_butteraugli_visual_mask( x264_butteraugli_t *ba,
                                  const uint8_t *ref_yuv[3], int ref_stride[3],
                                  float *mask, int mask_stride )
{
}

float x264_butteraugli_mb_weight( x264_butteraugli_t *ba,
                                 const uint8_t *ref_yuv[3], int ref_stride[3],
                                 const uint8_t *enc_yuv[3], int enc_stride[3],
                                 int mb_x, int mb_y )
{
    return 1.0f;
}

void x264_butteraugli_configure( x264_butteraugli_t *ba, const x264_butteraugli_config_t *config )
{
}

float x264_butteraugli_compute_tile( x264_butteraugli_t *ba,
                                    const uint8_t *ref_yuv[3], int ref_stride[3],
                                    const uint8_t *enc_yuv[3], int enc_stride[3],
                                    int tile_x, int tile_y, int tile_width, int tile_height )
{
    return 0.0f;
}

float x264_butteraugli_fuzzy_class( float score )
{
    return 0.0f;
}

float x264_butteraugli_fuzzy_inverse( float seek )
{
    return 1.0f;
}

int x264_butteraugli_adaptive_quantization( x264_butteraugli_t *ba,
                                           const uint8_t *yuv[3], int stride[3],
                                           float *quant_map, int quant_stride )
{
    return 0;
}

#endif /* HAVE_BUTTERAUGLI */

/* Butteraugli function pointer initialization - always available */
void x264_butteraugli_init( uint32_t cpu, x264_butteraugli_function_t *baf )
{
#ifdef HAVE_BUTTERAUGLI
    /* Initialize with scalar implementations */
    baf->yuv_to_rgb = butteraugli_yuv_to_rgb_c;
    baf->srgb_to_linear = butteraugli_srgb_to_linear_c;
    baf->create_image_planar = butteraugli_create_image_planar_c;
    baf->compute_visual_mask = butteraugli_compute_visual_mask_c;
    baf->heatmap_to_uint8 = butteraugli_heatmap_to_uint8_c;

#if HAVE_MMX
    /* x86 optimizations */
    if( cpu & X264_CPU_SSE2 )
    {
        baf->yuv_to_rgb = x264_butteraugli_yuv_to_rgb_sse2;
        baf->srgb_to_linear = x264_butteraugli_srgb_to_linear_sse2;
        baf->create_image_planar = x264_butteraugli_create_image_planar_sse2;
        baf->compute_visual_mask = x264_butteraugli_compute_visual_mask_sse2;
        baf->heatmap_to_uint8 = x264_butteraugli_heatmap_to_uint8_sse2;
    }
    
    if( cpu & X264_CPU_AVX2 )
    {
        baf->yuv_to_rgb = x264_butteraugli_yuv_to_rgb_avx2;
        baf->srgb_to_linear = x264_butteraugli_srgb_to_linear_avx2;
        baf->create_image_planar = x264_butteraugli_create_image_planar_avx2;
        baf->compute_visual_mask = x264_butteraugli_compute_visual_mask_avx2;
        baf->heatmap_to_uint8 = x264_butteraugli_heatmap_to_uint8_avx2;
    }
    
    if( cpu & X264_CPU_AVX512 )
    {
        baf->yuv_to_rgb = x264_butteraugli_yuv_to_rgb_avx512;
        baf->srgb_to_linear = x264_butteraugli_srgb_to_linear_avx512;
        baf->create_image_planar = x264_butteraugli_create_image_planar_avx512;
        baf->compute_visual_mask = x264_butteraugli_compute_visual_mask_avx512;
        baf->heatmap_to_uint8 = x264_butteraugli_heatmap_to_uint8_avx512;
    }
#endif

#if HAVE_ARMV6 || HAVE_AARCH64
    /* ARM optimizations */
    if( cpu & X264_CPU_NEON )
    {
        baf->yuv_to_rgb = x264_butteraugli_yuv_to_rgb_neon;
        baf->srgb_to_linear = x264_butteraugli_srgb_to_linear_neon;
        baf->create_image_planar = x264_butteraugli_create_image_planar_neon;
        baf->compute_visual_mask = x264_butteraugli_compute_visual_mask_neon;
        baf->heatmap_to_uint8 = x264_butteraugli_heatmap_to_uint8_neon;
    }
    
    if( cpu & X264_CPU_SVE )
    {
        baf->yuv_to_rgb = x264_butteraugli_yuv_to_rgb_sve;
        baf->srgb_to_linear = x264_butteraugli_srgb_to_linear_sve;
        baf->create_image_planar = x264_butteraugli_create_image_planar_sve;
        baf->compute_visual_mask = x264_butteraugli_compute_visual_mask_sve;
        baf->heatmap_to_uint8 = x264_butteraugli_heatmap_to_uint8_sve;
    }
#endif
#else
    /* No-op when butteraugli is not available */
    memset( baf, 0, sizeof(*baf) );
#endif
}