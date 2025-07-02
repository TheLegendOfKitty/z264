/*****************************************************************************
 * butteraugli-optimized.c: ARM NEON optimized implementations
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

#if HAVE_NEON
#include <arm_neon.h>

/* Optimized YUV to RGB conversion with efficient chroma handling */
void x264_butteraugli_yuv_to_rgb_neon_optimized( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                                 int y_stride, int uv_stride,
                                                 uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                                 int width, int height, x264_butteraugli_colorspace_t colorspace )
{
    /* YUV to RGB conversion coefficients for different color spaces */
    typedef struct
    {
        int16_t coeff_r_v;      /* R = Y + coeff_r_v * V */
        int16_t coeff_g_u;      /* G = Y - coeff_g_u * U - coeff_g_v * V */
        int16_t coeff_g_v;
        int16_t coeff_b_u;      /* B = Y + coeff_b_u * U */
    } yuv_to_rgb_coeffs_neon_t;

    /* Pre-computed coefficients scaled by 256 for fixed-point arithmetic */
    static const yuv_to_rgb_coeffs_neon_t yuv_coeffs_neon[3] = {
        /* BT.601 */
        { 360, -88, -184, 455 },
        /* BT.709 */
        { 404, -48, -120, 475 },
        /* BT.2020 */
        { 377, -42, -133, 482 }
    };

    const yuv_to_rgb_coeffs_neon_t *coeffs = &yuv_coeffs_neon[colorspace];
    const int16x8_t coeff_r = vdupq_n_s16(coeffs->coeff_r_v);
    const int16x8_t coeff_g_u = vdupq_n_s16(coeffs->coeff_g_u);
    const int16x8_t coeff_g_v = vdupq_n_s16(coeffs->coeff_g_v);
    const int16x8_t coeff_b = vdupq_n_s16(coeffs->coeff_b_u);
    const uint8x16_t uv_offset = vdupq_n_u8(128);
    
    for( int j = 0; j < height; j++ )
    {
        int x = 0;
        
        /* Process 16 pixels at a time for better efficiency */
        for( ; x <= width - 16; x += 16 )
        {
            /* Load 16 Y values */
            uint8x16_t y_vals = vld1q_u8(y + j * y_stride + x);
            
            /* Load 8 U and V values (for 16 pixels in 4:2:0) */
            uint8x8_t u_vals_8 = vld1_u8(u + (j/2) * uv_stride + (x/2));
            uint8x8_t v_vals_8 = vld1_u8(v + (j/2) * uv_stride + (x/2));
            
            /* Efficient chroma duplication using table lookup */
            /* Create duplication pattern: 0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7 */
            static const uint8x16_t dup_tbl = {0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7};
            uint8x16_t u_vals_16 = vcombine_u8(u_vals_8, u_vals_8);
            uint8x16_t v_vals_16 = vcombine_u8(v_vals_8, v_vals_8);
            uint8x16_t u_vals = vqtbl1q_u8(u_vals_16, dup_tbl);
            uint8x16_t v_vals = vqtbl1q_u8(v_vals_16, dup_tbl);
            
            /* Subtract 128 from U,V */
            int8x16_t u_centered = vreinterpretq_s8_u8(vsubq_u8(u_vals, uv_offset));
            int8x16_t v_centered = vreinterpretq_s8_u8(vsubq_u8(v_vals, uv_offset));
            
            /* Process first 8 pixels */
            uint8x8_t y_low = vget_low_u8(y_vals);
            int8x8_t u_low = vget_low_s8(u_centered);
            int8x8_t v_low = vget_low_s8(v_centered);
            
            int16x8_t y_16_low = vreinterpretq_s16_u16(vmovl_u8(y_low));
            int16x8_t u_16_low = vmovl_s8(u_low);
            int16x8_t v_16_low = vmovl_s8(v_low);
            
            /* Calculate R = Y + ((coeff_r * V) >> 8) */
            int16x8_t r_calc_low = vmlaq_s16(y_16_low, v_16_low, coeff_r);
            uint8x8_t r_result_low = vqrshrun_n_s16(r_calc_low, 8);
            
            /* Calculate G = Y + ((coeff_g_u * U + coeff_g_v * V) >> 8) */
            int16x8_t g_temp_low = vmulq_s16(u_16_low, coeff_g_u);
            g_temp_low = vmlaq_s16(g_temp_low, v_16_low, coeff_g_v);
            int16x8_t g_calc_low = vaddq_s16(y_16_low, vshrq_n_s16(g_temp_low, 8));
            uint8x8_t g_result_low = vqmovun_s16(g_calc_low);
            
            /* Calculate B = Y + ((coeff_b * U) >> 8) */
            int16x8_t b_calc_low = vmlaq_s16(y_16_low, u_16_low, coeff_b);
            uint8x8_t b_result_low = vqrshrun_n_s16(b_calc_low, 8);
            
            /* Process second 8 pixels */
            uint8x8_t y_high = vget_high_u8(y_vals);
            int8x8_t u_high = vget_high_s8(u_centered);
            int8x8_t v_high = vget_high_s8(v_centered);
            
            int16x8_t y_16_high = vreinterpretq_s16_u16(vmovl_u8(y_high));
            int16x8_t u_16_high = vmovl_s8(u_high);
            int16x8_t v_16_high = vmovl_s8(v_high);
            
            int16x8_t r_calc_high = vmlaq_s16(y_16_high, v_16_high, coeff_r);
            uint8x8_t r_result_high = vqrshrun_n_s16(r_calc_high, 8);
            
            int16x8_t g_temp_high = vmulq_s16(u_16_high, coeff_g_u);
            g_temp_high = vmlaq_s16(g_temp_high, v_16_high, coeff_g_v);
            int16x8_t g_calc_high = vaddq_s16(y_16_high, vshrq_n_s16(g_temp_high, 8));
            uint8x8_t g_result_high = vqmovun_s16(g_calc_high);
            
            int16x8_t b_calc_high = vmlaq_s16(y_16_high, u_16_high, coeff_b);
            uint8x8_t b_result_high = vqrshrun_n_s16(b_calc_high, 8);
            
            /* Combine and store 16 pixels */
            uint8x16_t r_result = vcombine_u8(r_result_low, r_result_high);
            uint8x16_t g_result = vcombine_u8(g_result_low, g_result_high);
            uint8x16_t b_result = vcombine_u8(b_result_low, b_result_high);
            
            vst1q_u8(rgb_r + j * rgb_stride + x, r_result);
            vst1q_u8(rgb_g + j * rgb_stride + x, g_result);
            vst1q_u8(rgb_b + j * rgb_stride + x, b_result);
        }
        
        /* Process 8 pixels at a time */
        for( ; x <= width - 8; x += 8 )
        {
            /* Load 8 Y values */
            uint8x8_t y_vals = vld1_u8(y + j * y_stride + x);
            
            /* Load 4 U and V values (for 8 pixels in 4:2:0) */
            uint8x8_t u_vals_4 = vld1_u8(u + (j/2) * uv_stride + (x/2));
            uint8x8_t v_vals_4 = vld1_u8(v + (j/2) * uv_stride + (x/2));
            
            /* Duplicate chroma values efficiently using interleave */
            uint8x8x2_t u_dup = vzip_u8(u_vals_4, u_vals_4);
            uint8x8x2_t v_dup = vzip_u8(v_vals_4, v_vals_4);
            uint8x8_t u_vals = u_dup.val[0];
            uint8x8_t v_vals = v_dup.val[0];
            
            /* Subtract 128 from U,V */
            int8x8_t u_centered = vreinterpret_s8_u8(vsub_u8(u_vals, vget_low_u8(uv_offset)));
            int8x8_t v_centered = vreinterpret_s8_u8(vsub_u8(v_vals, vget_low_u8(uv_offset)));
            
            /* Convert to 16-bit */
            int16x8_t y_16 = vreinterpretq_s16_u16(vmovl_u8(y_vals));
            int16x8_t u_16 = vmovl_s8(u_centered);
            int16x8_t v_16 = vmovl_s8(v_centered);
            
            /* Calculate RGB values */
            int16x8_t r_calc = vmlaq_s16(y_16, v_16, coeff_r);
            uint8x8_t r_result = vqrshrun_n_s16(r_calc, 8);
            
            int16x8_t g_temp = vmulq_s16(u_16, coeff_g_u);
            g_temp = vmlaq_s16(g_temp, v_16, coeff_g_v);
            int16x8_t g_calc = vaddq_s16(y_16, vshrq_n_s16(g_temp, 8));
            uint8x8_t g_result = vqmovun_s16(g_calc);
            
            int16x8_t b_calc = vmlaq_s16(y_16, u_16, coeff_b);
            uint8x8_t b_result = vqrshrun_n_s16(b_calc, 8);
            
            /* Store RGB to separate planes */
            vst1_u8(rgb_r + j * rgb_stride + x, r_result);
            vst1_u8(rgb_g + j * rgb_stride + x, g_result);
            vst1_u8(rgb_b + j * rgb_stride + x, b_result);
        }
        
        /* Handle remaining pixels with scalar code */
        for( ; x < width; x++ )
        {
            int yi = y[j * y_stride + x];
            int ui = u[(j/2) * uv_stride + (x/2)] - 128;
            int vi = v[(j/2) * uv_stride + (x/2)] - 128;
            
            int r = yi + ((coeffs->coeff_r_v * vi) >> 8);
            int g = yi + ((coeffs->coeff_g_u * ui + coeffs->coeff_g_v * vi) >> 8);
            int b = yi + ((coeffs->coeff_b_u * ui) >> 8);
            
            rgb_r[j * rgb_stride + x] = x264_clip3( r, 0, 255 );
            rgb_g[j * rgb_stride + x] = x264_clip3( g, 0, 255 );
            rgb_b[j * rgb_stride + x] = x264_clip3( b, 0, 255 );
        }
    }
}

#endif /* HAVE_NEON */