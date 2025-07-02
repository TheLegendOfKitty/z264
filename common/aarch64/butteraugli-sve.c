/*****************************************************************************
 * butteraugli-sve.c: aarch64 butteraugli SVE optimizations
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

#include "common/common.h"
#include "common/butteraugli.h"
#include "butteraugli.h"

#if HAVE_AARCH64 && defined(__ARM_FEATURE_SVE)
#include <arm_sve.h>

/* YUV to RGB conversion coefficients for different color spaces */
typedef struct
{
    int16_t coeff_r_v;      /* R = Y + coeff_r_v * V */
    int16_t coeff_g_u;      /* G = Y - coeff_g_u * U - coeff_g_v * V */
    int16_t coeff_g_v;
    int16_t coeff_b_u;      /* B = Y + coeff_b_u * U */
} yuv_to_rgb_coeffs_sve_t;

/* Pre-computed coefficients scaled by 256 for fixed-point arithmetic */
static const yuv_to_rgb_coeffs_sve_t yuv_coeffs_sve[3] = {
    /* BT.601 */
    { 360, -88, -184, 455 },
    /* BT.709 */
    { 404, -48, -120, 475 },
    /* BT.2020 */
    { 377, -42, -133, 482 }
};

/* SVE YUV to RGB conversion for all colorspaces */
void x264_butteraugli_yuv_to_rgb_sve( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                      int y_stride, int uv_stride,
                                      uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                      int width, int height, x264_butteraugli_colorspace_t colorspace )
{
    const yuv_to_rgb_coeffs_sve_t *coeffs = &yuv_coeffs_sve[colorspace];
    
    /* Create coefficient vectors */
    svint16_t coeff_r_v = svdup_n_s16(coeffs->coeff_r_v);
    svint16_t coeff_g_u = svdup_n_s16(coeffs->coeff_g_u);
    svint16_t coeff_g_v = svdup_n_s16(coeffs->coeff_g_v);
    svint16_t coeff_b_u = svdup_n_s16(coeffs->coeff_b_u);
    svint16_t uv_offset = svdup_n_s16(128);
    
    for( int j = 0; j < height; j++ )
    {
        int i = 0;
        
        /* Get vector length for current implementation */
        uint64_t vl = svcntb();
        
        /* Process VL pixels at a time */
        for( ; i + vl <= width; i += vl )
        {
            svbool_t pg = svptrue_b8();
            
            /* Load Y values */
            svuint8_t y_val = svld1_u8(pg, &y[j * y_stride + i]);
            svint16_t y_val_lo = svreinterpret_s16_u16(svunpklo_u16(y_val));
            svint16_t y_val_hi = svreinterpret_s16_u16(svunpkhi_u16(y_val));
            
            /* Load U and V values (handling 4:2:0 subsampling) */
            svuint8_t u_val_half = svld1_u8(pg, &u[(j/2) * uv_stride + (i/2)]);
            svuint8_t v_val_half = svld1_u8(pg, &v[(j/2) * uv_stride + (i/2)]);
            
            /* Duplicate chroma for 4:2:0 to 4:4:4 */
            svuint8_t u_val = svzip1_u8(u_val_half, u_val_half);
            svuint8_t v_val = svzip1_u8(v_val_half, v_val_half);
            
            /* Convert to signed 16-bit and subtract 128 */
            svint16_t u_val_lo = svsub_s16_z(pg, svreinterpret_s16_u16(svunpklo_u16(u_val)), uv_offset);
            svint16_t u_val_hi = svsub_s16_z(pg, svreinterpret_s16_u16(svunpkhi_u16(u_val)), uv_offset);
            svint16_t v_val_lo = svsub_s16_z(pg, svreinterpret_s16_u16(svunpklo_u16(v_val)), uv_offset);
            svint16_t v_val_hi = svsub_s16_z(pg, svreinterpret_s16_u16(svunpkhi_u16(v_val)), uv_offset);
            
            /* Calculate R = Y + ((coeff_r_v * V) >> 8) */
            svint16_t r_lo = svadd_s16_z(pg, y_val_lo, svasr_n_s16_z(pg, svmul_s16_z(pg, v_val_lo, coeff_r_v), 8));
            svint16_t r_hi = svadd_s16_z(pg, y_val_hi, svasr_n_s16_z(pg, svmul_s16_z(pg, v_val_hi, coeff_r_v), 8));
            
            /* Calculate G = Y - ((coeff_g_u * U + coeff_g_v * V) >> 8) */
            svint16_t g_lo = svsub_s16_z(pg, y_val_lo, 
                svasr_n_s16_z(pg, svadd_s16_z(pg, 
                    svmul_s16_z(pg, u_val_lo, coeff_g_u),
                    svmul_s16_z(pg, v_val_lo, coeff_g_v)), 8));
            svint16_t g_hi = svsub_s16_z(pg, y_val_hi,
                svasr_n_s16_z(pg, svadd_s16_z(pg,
                    svmul_s16_z(pg, u_val_hi, coeff_g_u),
                    svmul_s16_z(pg, v_val_hi, coeff_g_v)), 8));
            
            /* Calculate B = Y + ((coeff_b_u * U) >> 8) */
            svint16_t b_lo = svadd_s16_z(pg, y_val_lo, svasr_n_s16_z(pg, svmul_s16_z(pg, u_val_lo, coeff_b_u), 8));
            svint16_t b_hi = svadd_s16_z(pg, y_val_hi, svasr_n_s16_z(pg, svmul_s16_z(pg, u_val_hi, coeff_b_u), 8));
            
            /* Pack and clamp to 0-255 */
            svuint8_t r_packed = svqxtunb_s16(svqxtunt_s16(svreinterpret_u16_s16(r_lo), svreinterpret_u16_s16(r_hi)));
            svuint8_t g_packed = svqxtunb_s16(svqxtunt_s16(svreinterpret_u16_s16(g_lo), svreinterpret_u16_s16(g_hi)));
            svuint8_t b_packed = svqxtunb_s16(svqxtunt_s16(svreinterpret_u16_s16(b_lo), svreinterpret_u16_s16(b_hi)));
            
            /* Store results */
            svst1_u8(pg, &rgb_r[j * rgb_stride + i], r_packed);
            svst1_u8(pg, &rgb_g[j * rgb_stride + i], g_packed);
            svst1_u8(pg, &rgb_b[j * rgb_stride + i], b_packed);
        }
        
        /* Handle remaining pixels */
        for( ; i < width; i++ )
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

/* Stub implementations for other SVE functions - can be implemented later */
void x264_butteraugli_srgb_to_linear_sve( const uint8_t *srgb, float *linear,
                                          int width, int height, int stride )
{
    /* Fall back to NEON for now */
    x264_butteraugli_srgb_to_linear_neon( srgb, linear, width, height, stride );
}

void x264_butteraugli_create_image_planar_sve( const uint8_t *rgb_r, const uint8_t *rgb_g, const uint8_t *rgb_b,
                                               int rgb_stride, float *planar_r, float *planar_g, float *planar_b,
                                               int width, int height )
{
    /* Fall back to NEON for now */
    x264_butteraugli_create_image_planar_neon( rgb_r, rgb_g, rgb_b, rgb_stride,
                                               planar_r, planar_g, planar_b, width, height );
}

void x264_butteraugli_compute_visual_mask_sve( const float *ref_r, const float *ref_g, const float *ref_b,
                                               int ref_stride, float *mask, int mask_stride,
                                               int width, int height )
{
    /* Fall back to NEON for now */
    x264_butteraugli_compute_visual_mask_neon( ref_r, ref_g, ref_b, ref_stride,
                                              mask, mask_stride, width, height );
}

void x264_butteraugli_heatmap_to_uint8_sve( const float *heatmap, int heatmap_stride,
                                            uint8_t *output, int output_stride,
                                            int width, int height, float scale )
{
    /* Fall back to NEON for now */
    x264_butteraugli_heatmap_to_uint8_neon( heatmap, heatmap_stride, output, output_stride,
                                            width, height, scale );
}

#else /* !__ARM_FEATURE_SVE */

/* Stub implementations when SVE is not available */
void x264_butteraugli_yuv_to_rgb_sve( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                      int y_stride, int uv_stride,
                                      uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                      int width, int height, x264_butteraugli_colorspace_t colorspace )
{
    x264_butteraugli_yuv_to_rgb_neon( y, u, v, y_stride, uv_stride,
                                      rgb_r, rgb_g, rgb_b, rgb_stride,
                                      width, height, colorspace );
}

void x264_butteraugli_srgb_to_linear_sve( const uint8_t *srgb, float *linear,
                                          int width, int height, int stride )
{
    x264_butteraugli_srgb_to_linear_neon( srgb, linear, width, height, stride );
}

void x264_butteraugli_create_image_planar_sve( const uint8_t *rgb_r, const uint8_t *rgb_g, const uint8_t *rgb_b,
                                               int rgb_stride, float *planar_r, float *planar_g, float *planar_b,
                                               int width, int height )
{
    x264_butteraugli_create_image_planar_neon( rgb_r, rgb_g, rgb_b, rgb_stride,
                                               planar_r, planar_g, planar_b, width, height );
}

void x264_butteraugli_compute_visual_mask_sve( const float *ref_r, const float *ref_g, const float *ref_b,
                                               int ref_stride, float *mask, int mask_stride,
                                               int width, int height )
{
    x264_butteraugli_compute_visual_mask_neon( ref_r, ref_g, ref_b, ref_stride,
                                              mask, mask_stride, width, height );
}

void x264_butteraugli_heatmap_to_uint8_sve( const float *heatmap, int heatmap_stride,
                                            uint8_t *output, int output_stride,
                                            int width, int height, float scale )
{
    x264_butteraugli_heatmap_to_uint8_neon( heatmap, heatmap_stride, output, output_stride,
                                            width, height, scale );
}

#endif /* __ARM_FEATURE_SVE */