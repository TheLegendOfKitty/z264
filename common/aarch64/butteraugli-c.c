/*****************************************************************************
 * butteraugli-c.c: aarch64 butteraugli NEON optimizations
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

/* sRGB to linear lookup table (pre-computed) */
/* Values computed as: val <= 0.04045 ? val/12.92 : pow((val+0.055)/1.055, 2.4) */
static const float srgb_to_linear_lut[256] = {
    /* Values 0-15 */
    0.0f, 0.000303527f, 0.000607054f, 0.000910581f, 0.001214108f, 0.001517635f, 0.001821162f, 0.002124689f,
    0.002428216f, 0.002731743f, 0.003035270f, 0.003346535f, 0.003676507f, 0.004024717f, 0.004391442f, 0.004776953f,
    /* Values 16-31 */
    0.005181516f, 0.005605391f, 0.006048833f, 0.006512091f, 0.006995410f, 0.007499032f, 0.008023193f, 0.008568126f,
    0.009134058f, 0.009721218f, 0.010329823f, 0.010960094f, 0.011612245f, 0.012286488f, 0.012983032f, 0.013702083f,
    /* Values 32-47 */
    0.014443844f, 0.015208514f, 0.015996293f, 0.016807382f, 0.017641954f, 0.018500220f, 0.019382360f, 0.020288564f,
    0.021219010f, 0.022173884f, 0.023153366f, 0.024157632f, 0.025186859f, 0.026241221f, 0.027320891f, 0.028426039f,
    /* Values 48-63 */
    0.029556834f, 0.030713443f, 0.031896033f, 0.033104766f, 0.034339808f, 0.035601314f, 0.036889450f, 0.038204371f,
    0.039546237f, 0.040915205f, 0.042311430f, 0.043735067f, 0.045186270f, 0.046665191f, 0.048171984f, 0.049706798f,
    /* Values 64-79 */
    0.051269785f, 0.052861094f, 0.054480874f, 0.056129272f, 0.057806437f, 0.059512517f, 0.061247658f, 0.063012006f,
    0.064805708f, 0.066628908f, 0.068481749f, 0.070364374f, 0.072276925f, 0.074219543f, 0.076192369f, 0.078195542f,
    /* Values 80-95 */
    0.080229201f, 0.082293484f, 0.084388529f, 0.086514471f, 0.088671446f, 0.090859588f, 0.093079030f, 0.095329904f,
    0.097612343f, 0.099926477f, 0.102272435f, 0.104650346f, 0.107060337f, 0.109502536f, 0.111977069f, 0.114484063f,
    /* Values 96-111 */
    0.117023644f, 0.119595937f, 0.122201067f, 0.124839158f, 0.127510334f, 0.130214718f, 0.132952433f, 0.135723602f,
    0.138528347f, 0.141366789f, 0.144239050f, 0.147145251f, 0.150085513f, 0.153059955f, 0.156068698f, 0.159111861f,
    /* Values 112-127 */
    0.162189563f, 0.165301924f, 0.168449062f, 0.171631094f, 0.174848138f, 0.178100312f, 0.181387733f, 0.184710517f,
    0.188068782f, 0.191462643f, 0.194892218f, 0.198357622f, 0.201858972f, 0.205396383f, 0.208969971f, 0.212579851f,
    /* Values 128-143 */
    0.216226139f, 0.219908950f, 0.223628399f, 0.227384601f, 0.231177670f, 0.235007721f, 0.238874869f, 0.242779228f,
    0.246720912f, 0.250700036f, 0.254716713f, 0.258771058f, 0.262863184f, 0.266993204f, 0.271161233f, 0.275367383f,
    /* Values 144-159 */
    0.279611768f, 0.283894501f, 0.288215695f, 0.292575463f, 0.296973918f, 0.301411172f, 0.305887338f, 0.310402527f,
    0.314956852f, 0.319550425f, 0.324183359f, 0.328855765f, 0.333567755f, 0.338319441f, 0.343110934f, 0.347942346f,
    /* Values 160-175 */
    0.352813789f, 0.357725375f, 0.362677215f, 0.367669421f, 0.372702105f, 0.377775378f, 0.382889352f, 0.388044138f,
    0.393239848f, 0.398476593f, 0.403754485f, 0.409073635f, 0.414434155f, 0.419836156f, 0.425279750f, 0.430765047f,
    /* Values 176-191 */
    0.436292160f, 0.441861200f, 0.447472278f, 0.453125506f, 0.458820995f, 0.464558857f, 0.470339202f, 0.476162142f,
    0.482027789f, 0.487936254f, 0.493887648f, 0.499882083f, 0.505919670f, 0.512000520f, 0.518124744f, 0.524292454f,
    /* Values 192-207 */
    0.530503761f, 0.536758777f, 0.543057613f, 0.549400380f, 0.555787189f, 0.562218152f, 0.568693380f, 0.575212984f,
    0.581777076f, 0.588385767f, 0.595039168f, 0.601737390f, 0.608480545f, 0.615268743f, 0.622102097f, 0.628980717f,
    /* Values 208-223 */
    0.635904715f, 0.642874202f, 0.649889289f, 0.656950088f, 0.664056710f, 0.671209267f, 0.678407870f, 0.685652630f,
    0.692943660f, 0.700281071f, 0.707664975f, 0.715095484f, 0.722572709f, 0.730096763f, 0.737667757f, 0.745285803f,
    /* Values 224-239 */
    0.752951013f, 0.760663498f, 0.768423370f, 0.776230741f, 0.784085723f, 0.791988428f, 0.799938967f, 0.807937452f,
    0.815983995f, 0.824078708f, 0.832221703f, 0.840413092f, 0.848652986f, 0.856941498f, 0.865278738f, 0.873664819f,
    /* Values 240-255 */
    0.882099852f, 0.890583948f, 0.899117217f, 0.907699770f, 0.916331718f, 0.925013172f, 0.933744241f, 0.942525036f,
    0.951355667f, 0.960236244f, 0.969166877f, 0.978147675f, 0.987178749f, 0.996260208f, 1.005392163f, 1.014574723f
};

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

void x264_butteraugli_yuv_to_rgb_neon( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                       int y_stride, int uv_stride,
                                       uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                       int width, int height, x264_butteraugli_colorspace_t colorspace )
{
    // YUV to RGB conversion constants based on color space
    const yuv_to_rgb_coeffs_neon_t *coeffs = &yuv_coeffs_neon[colorspace];
    const int16x4_t coeff_r = vdup_n_s16(coeffs->coeff_r_v);
    const int16x4_t coeff_g_u = vdup_n_s16(coeffs->coeff_g_u);
    const int16x4_t coeff_g_v = vdup_n_s16(coeffs->coeff_g_v);
    const int16x4_t coeff_b = vdup_n_s16(coeffs->coeff_b_u);
    const uint8x8_t uv_offset = vdup_n_u8(128);
    
    for( int j = 0; j < height; j++ )
    {
        int x = 0;
        
        // Process 8 pixels at a time
        for( ; x <= width - 8; x += 8 )
        {
            // Load 8 Y values
            uint8x8_t y_vals = vld1_u8(y + j * y_stride + x);
            
            // Load 4 U and V values (for 8 pixels in 4:2:0)
            uint8x8_t u_vals_4 = vld1_u8(u + (j/2) * uv_stride + (x/2));
            uint8x8_t v_vals_4 = vld1_u8(v + (j/2) * uv_stride + (x/2));
            
            // Duplicate U,V horizontally for 4:2:0 -> 4:4:4 conversion
            uint8x8x2_t u_zip = vzip_u8(u_vals_4, u_vals_4);
            uint8x8x2_t v_zip = vzip_u8(v_vals_4, v_vals_4);
            uint8x8_t u_vals = u_zip.val[0];
            uint8x8_t v_vals = v_zip.val[0];
            
            // Subtract 128 from U,V
            int8x8_t u_centered = vreinterpret_s8_u8(vsub_u8(u_vals, uv_offset));
            int8x8_t v_centered = vreinterpret_s8_u8(vsub_u8(v_vals, uv_offset));
            
            // Convert to 16-bit
            int16x8_t y_16 = vreinterpretq_s16_u16(vmovl_u8(y_vals));
            int16x8_t u_16 = vmovl_s8(u_centered);
            int16x8_t v_16 = vmovl_s8(v_centered);
            
            // Calculate R = Y + ((360 * V) >> 8)
            int16x8_t r_calc = vmlaq_s16(y_16, v_16, vdupq_lane_s16(coeff_r, 0));
            uint8x8_t r_result = vqrshrun_n_s16(r_calc, 8);
            
            // Calculate G = Y - ((88 * U + 184 * V) >> 8)
            int16x8_t g_temp = vmulq_s16(u_16, vdupq_lane_s16(coeff_g_u, 0));
            g_temp = vmlaq_s16(g_temp, v_16, vdupq_lane_s16(coeff_g_v, 0));
            int16x8_t g_calc = vaddq_s16(y_16, g_temp);
            uint8x8_t g_result = vqrshrun_n_s16(g_calc, 8);
            
            // Calculate B = Y + ((455 * U) >> 8)
            int16x8_t b_calc = vmlaq_s16(y_16, u_16, vdupq_lane_s16(coeff_b, 0));
            uint8x8_t b_result = vqrshrun_n_s16(b_calc, 8);
            
            // Store RGB to separate planes
            vst1_u8(rgb_r + j * rgb_stride + x, r_result);
            vst1_u8(rgb_g + j * rgb_stride + x, g_result);
            vst1_u8(rgb_b + j * rgb_stride + x, b_result);
        }
        
        // Handle remaining pixels with scalar code
        for( ; x < width; x++ )
        {
            int yi = y[j * y_stride + x];
            int ui = u[(j/2) * uv_stride + (x/2)] - 128;
            int vi = v[(j/2) * uv_stride + (x/2)] - 128;
            
            int r = yi + ((coeffs->coeff_r_v * vi) >> 8);
            int g = yi - ((coeffs->coeff_g_u * ui + coeffs->coeff_g_v * vi) >> 8);
            int b = yi + ((coeffs->coeff_b_u * ui) >> 8);
            
            rgb_r[j * rgb_stride + x] = x264_clip3( r, 0, 255 );
            rgb_g[j * rgb_stride + x] = x264_clip3( g, 0, 255 );
            rgb_b[j * rgb_stride + x] = x264_clip3( b, 0, 255 );
        }
    }
}

void x264_butteraugli_srgb_to_linear_neon( const uint8_t *srgb, float *linear,
                                           int width, int height, int stride )
{
    for( int j = 0; j < height; j++ )
    {
        int x = 0;
        
        // Process 4 pixels at a time using lookup table
        for( ; x <= width - 4; x += 4 )
        {
            // Load 4 sRGB values
            uint8_t val0 = srgb[j * stride + x + 0];
            uint8_t val1 = srgb[j * stride + x + 1];
            uint8_t val2 = srgb[j * stride + x + 2];
            uint8_t val3 = srgb[j * stride + x + 3];
            
            // Lookup linear values
            float32x4_t linear_vals = {
                srgb_to_linear_lut[val0],
                srgb_to_linear_lut[val1],
                srgb_to_linear_lut[val2],
                srgb_to_linear_lut[val3]
            };
            
            // Store result
            vst1q_f32(linear + j * width + x, linear_vals);
        }
        
        // Handle remaining pixels with scalar code
        for( ; x < width; x++ )
        {
            linear[j * width + x] = srgb_to_linear_lut[srgb[j * stride + x]];
        }
    }
}

void x264_butteraugli_create_image_planar_neon( const uint8_t *rgb_r, const uint8_t *rgb_g, const uint8_t *rgb_b,
                                                int rgb_stride, float *planar_r, float *planar_g, float *planar_b,
                                                int width, int height )
{
    for( int j = 0; j < height; j++ )
    {
        int x = 0;
        
        // Process 8 pixels at a time
        for( ; x <= width - 8; x += 8 )
        {
            // Load 8 R values and convert to float
            uint8x8_t r_u8 = vld1_u8(rgb_r + j * rgb_stride + x);
            uint16x8_t r_u16 = vmovl_u8(r_u8);
            uint32x4_t r_u32_low = vmovl_u16(vget_low_u16(r_u16));
            uint32x4_t r_u32_high = vmovl_u16(vget_high_u16(r_u16));
            float32x4_t r_f32_low = vcvtq_f32_u32(r_u32_low);
            float32x4_t r_f32_high = vcvtq_f32_u32(r_u32_high);
            
            // Load 8 G values and convert to float
            uint8x8_t g_u8 = vld1_u8(rgb_g + j * rgb_stride + x);
            uint16x8_t g_u16 = vmovl_u8(g_u8);
            uint32x4_t g_u32_low = vmovl_u16(vget_low_u16(g_u16));
            uint32x4_t g_u32_high = vmovl_u16(vget_high_u16(g_u16));
            float32x4_t g_f32_low = vcvtq_f32_u32(g_u32_low);
            float32x4_t g_f32_high = vcvtq_f32_u32(g_u32_high);
            
            // Load 8 B values and convert to float
            uint8x8_t b_u8 = vld1_u8(rgb_b + j * rgb_stride + x);
            uint16x8_t b_u16 = vmovl_u8(b_u8);
            uint32x4_t b_u32_low = vmovl_u16(vget_low_u16(b_u16));
            uint32x4_t b_u32_high = vmovl_u16(vget_high_u16(b_u16));
            float32x4_t b_f32_low = vcvtq_f32_u32(b_u32_low);
            float32x4_t b_f32_high = vcvtq_f32_u32(b_u32_high);
            
            // Store to planar format
            vst1q_f32(planar_r + j * width + x, r_f32_low);
            vst1q_f32(planar_r + j * width + x + 4, r_f32_high);
            vst1q_f32(planar_g + j * width + x, g_f32_low);
            vst1q_f32(planar_g + j * width + x + 4, g_f32_high);
            vst1q_f32(planar_b + j * width + x, b_f32_low);
            vst1q_f32(planar_b + j * width + x + 4, b_f32_high);
        }
        
        // Handle remaining pixels with scalar code
        for( ; x < width; x++ )
        {
            planar_r[j * width + x] = (float)rgb_r[j * rgb_stride + x];
            planar_g[j * width + x] = (float)rgb_g[j * rgb_stride + x];
            planar_b[j * width + x] = (float)rgb_b[j * rgb_stride + x];
        }
    }
}

void x264_butteraugli_compute_visual_mask_neon( const float *ref_r, const float *ref_g, const float *ref_b,
                                                int ref_stride, float *mask, int mask_stride,
                                                int width, int height )
{
    
    for( int y = 1; y < height - 1; y++ )
    {
        int x = 1;
        
        // Vectorized 3x3 variance computation using NEON
        for( ; x < width - 1; x++ )
        {
            float sum = 0.0f;
            float sum_sq = 0.0f;
            
            // Process the 3x3 neighborhood with partial vectorization
            // Load and process each row of 3 pixels
            for( int dy = -1; dy <= 1; dy++ )
            {
                int row_offset = (y + dy) * ref_stride + (x - 1);
                
                // Load 3 pixels from each color plane into vectors
                // We use float32x2_t for pairs and handle the third separately
                float32x2_t r_01 = vld1_f32(ref_r + row_offset);
                float32x2_t g_01 = vld1_f32(ref_g + row_offset);
                float32x2_t b_01 = vld1_f32(ref_b + row_offset);
                float r_2 = ref_r[row_offset + 2];
                float g_2 = ref_g[row_offset + 2];
                float b_2 = ref_b[row_offset + 2];
                
                // Compute luma for first 2 pixels
                float32x2_t luma_01 = vmul_n_f32(r_01, 0.299f);
                luma_01 = vmla_n_f32(luma_01, g_01, 0.587f);
                luma_01 = vmla_n_f32(luma_01, b_01, 0.114f);
                
                // Compute luma for third pixel
                float luma_2 = 0.299f * r_2 + 0.587f * g_2 + 0.114f * b_2;
                
                // Accumulate sum and sum_sq
                float32x2_t sum_pair = vpadd_f32(luma_01, luma_01);  // sum of first 2
                sum += vget_lane_f32(sum_pair, 0) + luma_2;
                
                float32x2_t sq_01 = vmul_f32(luma_01, luma_01);
                float32x2_t sum_sq_pair = vpadd_f32(sq_01, sq_01);
                sum_sq += vget_lane_f32(sum_sq_pair, 0) + luma_2 * luma_2;
            }
            
            // Compute variance for the 9 values
            float mean = sum / 9.0f;
            float variance = (sum_sq / 9.0f) - (mean * mean);
            
            // Apply exponential mapping and clamping
            float masking = 1.0f + expf(-variance * 100.0f);
            mask[y * mask_stride + x] = fminf(2.0f, fmaxf(0.5f, masking));
        }
    }
}

void x264_butteraugli_heatmap_to_uint8_neon( const float *heatmap, int heatmap_stride,
                                             uint8_t *output, int output_stride,
                                             int width, int height, float scale )
{
    const float32x4_t scale_vec = vdupq_n_f32(scale);
    const float32x4_t max_val = vdupq_n_f32(255.0f);
    const float32x4_t zero = vdupq_n_f32(0.0f);
    
    for( int y = 0; y < height; y++ )
    {
        int x = 0;
        
        // Process 4 pixels at a time
        for( ; x <= width - 4; x += 4 )
        {
            // Load 4 float values
            float32x4_t heat_vals = vld1q_f32(heatmap + y * heatmap_stride + x);
            
            // Scale values
            heat_vals = vmulq_f32(heat_vals, scale_vec);
            
            // Clamp to 0-255 range
            heat_vals = vmaxq_f32(zero, vminq_f32(max_val, heat_vals));
            
            // Convert to uint32, then to uint8
            uint32x4_t uint_vals = vcvtq_u32_f32(heat_vals);
            uint16x4_t uint16_vals = vmovn_u32(uint_vals);
            uint8x8_t uint8_vals = vmovn_u16(vcombine_u16(uint16_vals, uint16_vals));
            
            // Store 4 bytes
            uint8_t temp[8];
            vst1_u8(temp, uint8_vals);
            for( int i = 0; i < 4; i++ )
                output[y * output_stride + x + i] = temp[i];
        }
        
        // Handle remaining pixels with scalar code
        for( ; x < width; x++ )
        {
            float val = heatmap[y * heatmap_stride + x] * scale;
            output[y * output_stride + x] = (uint8_t)fminf(255.0f, fmaxf(0.0f, val));
        }
    }
}

#endif /* HAVE_NEON */