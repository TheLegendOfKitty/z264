/*****************************************************************************
 * butteraugli-visual-mask.c: ARM NEON optimized visual mask computation
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
#include <math.h>

#if HAVE_NEON
#include <arm_neon.h>

/* NEON optimized visual mask computation */
void x264_butteraugli_compute_visual_mask_neon_optimized( const float *ref_r, const float *ref_g, const float *ref_b,
                                                          int ref_stride, float *mask, int mask_stride,
                                                          int width, int height )
{
    const float32x4_t luma_r = vdupq_n_f32(0.299f);
    const float32x4_t luma_g = vdupq_n_f32(0.587f);
    const float32x4_t luma_b = vdupq_n_f32(0.114f);
    const float32x4_t const_9 = vdupq_n_f32(1.0f / 9.0f);
    const float32x4_t const_100 = vdupq_n_f32(100.0f);
    const float32x4_t const_05 = vdupq_n_f32(0.5f);
    const float32x4_t const_2 = vdupq_n_f32(2.0f);
    
    /* Process interior pixels (skip borders) */
    for( int y = 1; y < height - 1; y++ )
    {
        int x = 1;
        
        /* Process 4 pixels at once where possible */
        for( ; x < width - 5; x += 4 )
        {
            float32x4_t mask_vals = vdupq_n_f32(0.0f);
            
            /* Process each of the 4 center pixels */
            for( int i = 0; i < 4; i++ )
            {
                float32x4_t sum = vdupq_n_f32(0.0f);
                float32x4_t sum_sq = vdupq_n_f32(0.0f);
                
                /* Process 3x3 neighborhood */
                for( int dy = -1; dy <= 1; dy++ )
                {
                    int row_offset = (y + dy) * ref_stride + x + i - 1;
                    
                    /* Load 3 consecutive pixels from each plane */
                    float32x4_t r_vals = vld1q_f32(&ref_r[row_offset]);
                    float32x4_t g_vals = vld1q_f32(&ref_g[row_offset]);
                    float32x4_t b_vals = vld1q_f32(&ref_b[row_offset]);
                    
                    /* Compute luma values */
                    float32x4_t luma = vmulq_f32(r_vals, luma_r);
                    luma = vmlaq_f32(luma, g_vals, luma_g);
                    luma = vmlaq_f32(luma, b_vals, luma_b);
                    
                    /* Sum the first 3 values */
                    float32x2_t luma_low = vget_low_f32(luma);
                    float32x2_t luma_high = vget_high_f32(luma);
                    float32x2_t sum_pair = vpadd_f32(luma_low, luma_high);
                    
                    float sum_3 = vget_lane_f32(sum_pair, 0) + vgetq_lane_f32(luma, 2);
                    sum = vdupq_n_f32(sum_3);
                    
                    /* Sum of squares */
                    float32x4_t luma_sq = vmulq_f32(luma, luma);
                    float32x2_t luma_sq_low = vget_low_f32(luma_sq);
                    float32x2_t luma_sq_high = vget_high_f32(luma_sq);
                    float32x2_t sum_sq_pair = vpadd_f32(luma_sq_low, luma_sq_high);
                    
                    float sum_sq_3 = vget_lane_f32(sum_sq_pair, 0) + vgetq_lane_f32(luma_sq, 2);
                    sum_sq = vdupq_n_f32(sum_sq_3);
                }
                
                /* Compute variance */
                float32x4_t mean = vmulq_f32(sum, const_9);
                float32x4_t mean_sq = vmulq_f32(mean, mean);
                float32x4_t variance = vsubq_f32(vmulq_f32(sum_sq, const_9), mean_sq);
                
                /* Extract variance value for exponential */
                float var_val = vgetq_lane_f32(variance, 0);
                
                /* Compute masking value */
                float masking;
                if( var_val < 0.001f )
                {
                    masking = 2.0f;
                }
                else if( var_val > 0.1f )
                {
                    masking = 0.8f + 0.2f * expf(-var_val * 5.0f);
                }
                else
                {
                    masking = 1.0f + expf(-var_val * 100.0f);
                }
                
                /* Store in vector */
                switch(i) {
                    case 0: mask_vals = vsetq_lane_f32(masking, mask_vals, 0); break;
                    case 1: mask_vals = vsetq_lane_f32(masking, mask_vals, 1); break;
                    case 2: mask_vals = vsetq_lane_f32(masking, mask_vals, 2); break;
                    case 3: mask_vals = vsetq_lane_f32(masking, mask_vals, 3); break;
                }
            }
            
            /* Clamp values */
            mask_vals = vmaxq_f32(mask_vals, const_05);
            mask_vals = vminq_f32(mask_vals, const_2);
            
            /* Store 4 results */
            vst1q_f32(&mask[y * mask_stride + x], mask_vals);
        }
        
        /* Handle remaining pixels with scalar code */
        for( ; x < width - 1; x++ )
        {
            float sum = 0.0f;
            float sum_sq = 0.0f;
            
            /* 3x3 neighborhood */
            for( int dy = -1; dy <= 1; dy++ )
            {
                for( int dx = -1; dx <= 1; dx++ )
                {
                    int offset = (y + dy) * ref_stride + (x + dx);
                    float luma = 0.299f * ref_r[offset] + 
                                0.587f * ref_g[offset] + 
                                0.114f * ref_b[offset];
                    sum += luma;
                    sum_sq += luma * luma;
                }
            }
            
            float mean = sum / 9.0f;
            float variance = (sum_sq / 9.0f) - (mean * mean);
            
            float masking;
            if( variance < 0.001f )
            {
                masking = 2.0f;
            }
            else if( variance > 0.1f )
            {
                masking = 0.8f + 0.2f * expf(-variance * 5.0f);
            }
            else
            {
                masking = 1.0f + expf(-variance * 100.0f);
            }
            
            mask[y * mask_stride + x] = fminf(2.0f, fmaxf(0.5f, masking));
        }
    }
    
    /* Fill borders with neutral value */
    for( int x = 0; x < width; x++ )
    {
        mask[x] = 1.0f;
        mask[(height - 1) * mask_stride + x] = 1.0f;
    }
    for( int y = 1; y < height - 1; y++ )
    {
        mask[y * mask_stride] = 1.0f;
        mask[y * mask_stride + width - 1] = 1.0f;
    }
}

#endif /* HAVE_NEON */