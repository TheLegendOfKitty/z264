/*****************************************************************************
 * butteraugli-c.c: x86 butteraugli C wrappers
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

#if HAVE_MMX
#include <emmintrin.h> /* SSE2 */

/* Assembly functions that only support BT.601 */
void x264_butteraugli_yuv_to_rgb_sse2_bt601( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                             int y_stride, int uv_stride,
                                             uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                             int width, int height );
void x264_butteraugli_yuv_to_rgb_avx2_bt601( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                             int y_stride, int uv_stride,
                                             uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                             int width, int height );

/* YUV to RGB conversion coefficients for different color spaces */
typedef struct
{
    int16_t coeff_r_v;      /* R = Y + coeff_r_v * V */
    int16_t coeff_g_u;      /* G = Y - coeff_g_u * U - coeff_g_v * V */
    int16_t coeff_g_v;
    int16_t coeff_b_u;      /* B = Y + coeff_b_u * U */
} yuv_to_rgb_coeffs_x86_t;

/* Pre-computed coefficients scaled by 256 for fixed-point arithmetic */
static const yuv_to_rgb_coeffs_x86_t yuv_coeffs_x86[3] = {
    /* BT.601 */
    { 360, -88, -184, 455 },
    /* BT.709 */
    { 404, -48, -120, 475 },
    /* BT.2020 */
    { 377, -42, -133, 482 }
};

/* SSE2 YUV to RGB conversion for BT.709 and BT.2020 using intrinsics */
static void x264_butteraugli_yuv_to_rgb_sse2_generic( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                                      int y_stride, int uv_stride,
                                                      uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                                      int width, int height, x264_butteraugli_colorspace_t colorspace )
{
    const yuv_to_rgb_coeffs_x86_t *coeffs = &yuv_coeffs_x86[colorspace];
    const __m128i coeff_r_v = _mm_set1_epi16(coeffs->coeff_r_v);
    const __m128i coeff_g_u = _mm_set1_epi16(coeffs->coeff_g_u);
    const __m128i coeff_g_v = _mm_set1_epi16(coeffs->coeff_g_v);
    const __m128i coeff_b_u = _mm_set1_epi16(coeffs->coeff_b_u);
    const __m128i uv_offset = _mm_set1_epi8(-128);
    const __m128i zero = _mm_setzero_si128();
    
    for( int j = 0; j < height; j++ )
    {
        int i = 0;
        
        /* Process 8 pixels at a time */
        for( ; i + 7 < width; i += 8 )
        {
            /* Load 8 Y values */
            __m128i y_val = _mm_loadl_epi64((__m128i*)&y[j * y_stride + i]);
            y_val = _mm_unpacklo_epi8(y_val, zero); /* Convert to 16-bit */
            
            /* Load 4 U and V values (4:2:0 subsampling) */
            __m128i u_val = _mm_cvtsi32_si128(*(uint32_t*)&u[(j/2) * uv_stride + (i/2)]);
            __m128i v_val = _mm_cvtsi32_si128(*(uint32_t*)&v[(j/2) * uv_stride + (i/2)]);
            
            /* Expand chroma for 4:2:0 to 4:4:4 (duplicate each value) */
            u_val = _mm_unpacklo_epi8(u_val, u_val); /* U0U0 U1U1 U2U2 U3U3 */
            v_val = _mm_unpacklo_epi8(v_val, v_val); /* V0V0 V1V1 V2V2 V3V3 */
            
            /* Subtract 128 from UV */
            u_val = _mm_add_epi8(u_val, uv_offset);
            v_val = _mm_add_epi8(v_val, uv_offset);
            
            /* Convert UV to signed 16-bit */
            __m128i u_val16 = _mm_unpacklo_epi8(zero, u_val);
            u_val16 = _mm_srai_epi16(u_val16, 8);
            __m128i v_val16 = _mm_unpacklo_epi8(zero, v_val);
            v_val16 = _mm_srai_epi16(v_val16, 8);
            
            /* Calculate R = Y + ((coeff_r_v * V) >> 8) */
            __m128i r_val = _mm_mullo_epi16(v_val16, coeff_r_v);
            r_val = _mm_srai_epi16(r_val, 8);
            r_val = _mm_add_epi16(r_val, y_val);
            
            /* Calculate G = Y - ((coeff_g_u * U + coeff_g_v * V) >> 8) */
            __m128i g_val = _mm_mullo_epi16(u_val16, coeff_g_u);
            __m128i g_val2 = _mm_mullo_epi16(v_val16, coeff_g_v);
            g_val = _mm_add_epi16(g_val, g_val2);
            g_val = _mm_srai_epi16(g_val, 8);
            g_val = _mm_sub_epi16(y_val, g_val);
            
            /* Calculate B = Y + ((coeff_b_u * U) >> 8) */
            __m128i b_val = _mm_mullo_epi16(u_val16, coeff_b_u);
            b_val = _mm_srai_epi16(b_val, 8);
            b_val = _mm_add_epi16(b_val, y_val);
            
            /* Pack and clamp to 0-255 */
            r_val = _mm_packus_epi16(r_val, r_val);
            g_val = _mm_packus_epi16(g_val, g_val);
            b_val = _mm_packus_epi16(b_val, b_val);
            
            /* Store results */
            _mm_storel_epi64((__m128i*)&rgb_r[j * rgb_stride + i], r_val);
            _mm_storel_epi64((__m128i*)&rgb_g[j * rgb_stride + i], g_val);
            _mm_storel_epi64((__m128i*)&rgb_b[j * rgb_stride + i], b_val);
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

/* SSE2 wrapper that handles multiple colorspaces */
void x264_butteraugli_yuv_to_rgb_sse2( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                       int y_stride, int uv_stride,
                                       uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                       int width, int height, x264_butteraugli_colorspace_t colorspace )
{
    if( colorspace == X264_BUTTERAUGLI_CS_BT601 )
    {
        /* Use optimized assembly for BT.601 */
        x264_butteraugli_yuv_to_rgb_sse2_bt601( y, u, v, y_stride, uv_stride,
                                                rgb_r, rgb_g, rgb_b, rgb_stride,
                                                width, height );
    }
    else
    {
        /* Use SSE2 intrinsics for BT.709 and BT.2020 */
        x264_butteraugli_yuv_to_rgb_sse2_generic( y, u, v, y_stride, uv_stride,
                                                 rgb_r, rgb_g, rgb_b, rgb_stride,
                                                 width, height, colorspace );
    }
}

#ifdef __AVX2__
#include <immintrin.h> /* AVX2 */

/* AVX2 YUV to RGB conversion for BT.709 and BT.2020 using intrinsics */
static void x264_butteraugli_yuv_to_rgb_avx2_generic( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                                      int y_stride, int uv_stride,
                                                      uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                                      int width, int height, x264_butteraugli_colorspace_t colorspace )
{
    const yuv_to_rgb_coeffs_x86_t *coeffs = &yuv_coeffs_x86[colorspace];
    const __m256i coeff_r_v = _mm256_set1_epi16(coeffs->coeff_r_v);
    const __m256i coeff_g_u = _mm256_set1_epi16(coeffs->coeff_g_u);
    const __m256i coeff_g_v = _mm256_set1_epi16(coeffs->coeff_g_v);
    const __m256i coeff_b_u = _mm256_set1_epi16(coeffs->coeff_b_u);
    const __m256i uv_offset = _mm256_set1_epi8(-128);
    const __m256i zero = _mm256_setzero_si256();
    
    for( int j = 0; j < height; j++ )
    {
        int i = 0;
        
        /* Process 16 pixels at a time */
        for( ; i + 15 < width; i += 16 )
        {
            /* Load 16 Y values */
            __m128i y_val_lo = _mm_loadu_si128((__m128i*)&y[j * y_stride + i]);
            __m256i y_val = _mm256_cvtepu8_epi16(y_val_lo);
            
            /* Load 8 U and V values (4:2:0 subsampling) */
            __m128i u_val_8 = _mm_loadl_epi64((__m128i*)&u[(j/2) * uv_stride + (i/2)]);
            __m128i v_val_8 = _mm_loadl_epi64((__m128i*)&v[(j/2) * uv_stride + (i/2)]);
            
            /* Expand chroma for 4:2:0 to 4:4:4 (duplicate each value) */
            u_val_8 = _mm_unpacklo_epi8(u_val_8, u_val_8); /* U0U0 U1U1... */
            v_val_8 = _mm_unpacklo_epi8(v_val_8, v_val_8); /* V0V0 V1V1... */
            
            /* Subtract 128 from UV */
            u_val_8 = _mm_add_epi8(u_val_8, _mm256_castsi256_si128(uv_offset));
            v_val_8 = _mm_add_epi8(v_val_8, _mm256_castsi256_si128(uv_offset));
            
            /* Convert UV to signed 16-bit */
            __m256i u_val = _mm256_cvtepi8_epi16(u_val_8);
            __m256i v_val = _mm256_cvtepi8_epi16(v_val_8);
            
            /* Calculate R = Y + ((coeff_r_v * V) >> 8) */
            __m256i r_val = _mm256_mullo_epi16(v_val, coeff_r_v);
            r_val = _mm256_srai_epi16(r_val, 8);
            r_val = _mm256_add_epi16(r_val, y_val);
            
            /* Calculate G = Y - ((coeff_g_u * U + coeff_g_v * V) >> 8) */
            __m256i g_val = _mm256_mullo_epi16(u_val, coeff_g_u);
            __m256i g_val2 = _mm256_mullo_epi16(v_val, coeff_g_v);
            g_val = _mm256_add_epi16(g_val, g_val2);
            g_val = _mm256_srai_epi16(g_val, 8);
            g_val = _mm256_sub_epi16(y_val, g_val);
            
            /* Calculate B = Y + ((coeff_b_u * U) >> 8) */
            __m256i b_val = _mm256_mullo_epi16(u_val, coeff_b_u);
            b_val = _mm256_srai_epi16(b_val, 8);
            b_val = _mm256_add_epi16(b_val, y_val);
            
            /* Pack and clamp to 0-255 */
            r_val = _mm256_packus_epi16(r_val, r_val);
            g_val = _mm256_packus_epi16(g_val, g_val);
            b_val = _mm256_packus_epi16(b_val, b_val);
            
            /* Extract 128-bit results */
            __m128i r_packed = _mm256_extracti128_si256(r_val, 0);
            __m128i g_packed = _mm256_extracti128_si256(g_val, 0);
            __m128i b_packed = _mm256_extracti128_si256(b_val, 0);
            
            /* Store results */
            _mm_storeu_si128((__m128i*)&rgb_r[j * rgb_stride + i], r_packed);
            _mm_storeu_si128((__m128i*)&rgb_g[j * rgb_stride + i], g_packed);
            _mm_storeu_si128((__m128i*)&rgb_b[j * rgb_stride + i], b_packed);
        }
        
        /* Handle remaining pixels with SSE2 */
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
#endif /* __AVX2__ */

/* AVX2 wrapper that handles multiple colorspaces */
void x264_butteraugli_yuv_to_rgb_avx2( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                       int y_stride, int uv_stride,
                                       uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                       int width, int height, x264_butteraugli_colorspace_t colorspace )
{
    if( colorspace == X264_BUTTERAUGLI_CS_BT601 )
    {
        /* Use optimized assembly for BT.601 */
        x264_butteraugli_yuv_to_rgb_avx2_bt601( y, u, v, y_stride, uv_stride,
                                                rgb_r, rgb_g, rgb_b, rgb_stride,
                                                width, height );
    }
    else
    {
#ifdef __AVX2__
        /* Use AVX2 intrinsics for BT.709 and BT.2020 */
        x264_butteraugli_yuv_to_rgb_avx2_generic( y, u, v, y_stride, uv_stride,
                                                 rgb_r, rgb_g, rgb_b, rgb_stride,
                                                 width, height, colorspace );
#else
        /* For BT.709 and BT.2020, use SSE2 fallback */
        x264_butteraugli_yuv_to_rgb_sse2( y, u, v, y_stride, uv_stride,
                                         rgb_r, rgb_g, rgb_b, rgb_stride,
                                         width, height, colorspace );
#endif
    }
}

/* AVX-512 implementations */
#ifdef __AVX512F__
#include <immintrin.h> /* AVX-512 */

/* AVX-512 YUV to RGB conversion for all colorspaces */
void x264_butteraugli_yuv_to_rgb_avx512( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                         int y_stride, int uv_stride,
                                         uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                         int width, int height, x264_butteraugli_colorspace_t colorspace )
{
    const yuv_to_rgb_coeffs_x86_t *coeffs = &yuv_coeffs_x86[colorspace];
    const __m512i coeff_r_v = _mm512_set1_epi16(coeffs->coeff_r_v);
    const __m512i coeff_g_u = _mm512_set1_epi16(coeffs->coeff_g_u);
    const __m512i coeff_g_v = _mm512_set1_epi16(coeffs->coeff_g_v);
    const __m512i coeff_b_u = _mm512_set1_epi16(coeffs->coeff_b_u);
    const __m512i uv_offset = _mm512_set1_epi8(-128);
    const __m512i zero = _mm512_setzero_si512();
    
    for( int j = 0; j < height; j++ )
    {
        int i = 0;
        
        /* Process 32 pixels at a time */
        for( ; i + 31 < width; i += 32 )
        {
            /* Load 32 Y values */
            __m256i y_val_256 = _mm256_loadu_si256((__m256i*)&y[j * y_stride + i]);
            __m512i y_val = _mm512_cvtepu8_epi16(y_val_256);
            
            /* Load 16 U and V values (4:2:0 subsampling) */
            __m128i u_val_128 = _mm_loadu_si128((__m128i*)&u[(j/2) * uv_stride + (i/2)]);
            __m128i v_val_128 = _mm_loadu_si128((__m128i*)&v[(j/2) * uv_stride + (i/2)]);
            
            /* Expand chroma for 4:2:0 to 4:4:4 */
            __m256i u_val_256 = _mm256_cvtepu8_epi16(u_val_128);
            __m256i v_val_256 = _mm256_cvtepu8_epi16(v_val_128);
            
            /* Duplicate each 16-bit value for 2x2 blocks */
            __m512i u_val = _mm512_permutexvar_epi16(_mm512_set_epi16(
                15,15,14,14,13,13,12,12,11,11,10,10,9,9,8,8,
                7,7,6,6,5,5,4,4,3,3,2,2,1,1,0,0), 
                _mm512_castsi256_si512(u_val_256));
            __m512i v_val = _mm512_permutexvar_epi16(_mm512_set_epi16(
                15,15,14,14,13,13,12,12,11,11,10,10,9,9,8,8,
                7,7,6,6,5,5,4,4,3,3,2,2,1,1,0,0), 
                _mm512_castsi256_si512(v_val_256));
            
            /* Subtract 128 from UV */
            u_val = _mm512_sub_epi16(u_val, _mm512_set1_epi16(128));
            v_val = _mm512_sub_epi16(v_val, _mm512_set1_epi16(128));
            
            /* Calculate R = Y + ((coeff_r_v * V) >> 8) */
            __m512i r_val = _mm512_mullo_epi16(v_val, coeff_r_v);
            r_val = _mm512_srai_epi16(r_val, 8);
            r_val = _mm512_add_epi16(r_val, y_val);
            
            /* Calculate G = Y - ((coeff_g_u * U + coeff_g_v * V) >> 8) */
            __m512i g_val = _mm512_mullo_epi16(u_val, coeff_g_u);
            __m512i g_val2 = _mm512_mullo_epi16(v_val, coeff_g_v);
            g_val = _mm512_add_epi16(g_val, g_val2);
            g_val = _mm512_srai_epi16(g_val, 8);
            g_val = _mm512_sub_epi16(y_val, g_val);
            
            /* Calculate B = Y + ((coeff_b_u * U) >> 8) */
            __m512i b_val = _mm512_mullo_epi16(u_val, coeff_b_u);
            b_val = _mm512_srai_epi16(b_val, 8);
            b_val = _mm512_add_epi16(b_val, y_val);
            
            /* Pack and clamp to 0-255 */
            __m256i r_packed = _mm512_cvtepi16_epi8(_mm512_packus_epi16(r_val, r_val));
            __m256i g_packed = _mm512_cvtepi16_epi8(_mm512_packus_epi16(g_val, g_val));
            __m256i b_packed = _mm512_cvtepi16_epi8(_mm512_packus_epi16(b_val, b_val));
            
            /* Store results */
            _mm256_storeu_si256((__m256i*)&rgb_r[j * rgb_stride + i], r_packed);
            _mm256_storeu_si256((__m256i*)&rgb_g[j * rgb_stride + i], g_packed);
            _mm256_storeu_si256((__m256i*)&rgb_b[j * rgb_stride + i], b_packed);
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

/* AVX-512 sRGB to linear conversion */
void x264_butteraugli_srgb_to_linear_avx512( const uint8_t *srgb, float *linear,
                                             int width, int height, int stride )
{
    const __m512 scale = _mm512_set1_ps(1.0f / 255.0f);
    const __m512 threshold = _mm512_set1_ps(0.04045f);
    const __m512 div_12_92 = _mm512_set1_ps(1.0f / 12.92f);
    const __m512 offset = _mm512_set1_ps(0.055f);
    const __m512 div_1_055 = _mm512_set1_ps(1.0f / 1.055f);
    const __m512 exp_2_4 = _mm512_set1_ps(2.4f);
    
    for( int j = 0; j < height; j++ )
    {
        int i = 0;
        
        /* Process 16 pixels at a time */
        for( ; i + 15 < width; i += 16 )
        {
            /* Load 16 uint8_t values */
            __m128i srgb_val = _mm_loadu_si128((__m128i*)&srgb[j * stride + i]);
            
            /* Convert to float and normalize */
            __m512i srgb_32 = _mm512_cvtepu8_epi32(srgb_val);
            __m512 val = _mm512_mul_ps(_mm512_cvtepi32_ps(srgb_32), scale);
            
            /* Apply sRGB to linear conversion */
            __mmask16 mask = _mm512_cmp_ps_mask(val, threshold, _CMP_LE_OQ);
            
            /* val <= 0.04045 ? val / 12.92 : pow((val + 0.055) / 1.055, 2.4) */
            __m512 linear_low = _mm512_mul_ps(val, div_12_92);
            __m512 linear_high = _mm512_pow_ps(
                _mm512_mul_ps(_mm512_add_ps(val, offset), div_1_055), exp_2_4);
            
            __m512 result = _mm512_mask_blend_ps(mask, linear_high, linear_low);
            
            /* Store results */
            _mm512_storeu_ps(&linear[j * width + i], result);
        }
        
        /* Handle remaining pixels */
        for( ; i < width; i++ )
        {
            float val = srgb[j * stride + i] / 255.0f;
            if( val <= 0.04045f )
                linear[j * width + i] = val / 12.92f;
            else
                linear[j * width + i] = powf( (val + 0.055f) / 1.055f, 2.4f );
        }
    }
}

/* AVX-512 create planar image implementation */
void x264_butteraugli_create_image_planar_avx512( const uint8_t *rgb_r, const uint8_t *rgb_g, const uint8_t *rgb_b,
                                                  int rgb_stride, float *planar_r, float *planar_g, float *planar_b,
                                                  int width, int height )
{
    for( int j = 0; j < height; j++ )
    {
        int i = 0;
        
        /* Process 16 pixels at a time */
        for( ; i + 15 < width; i += 16 )
        {
            /* Load 16 uint8_t values from each channel */
            __m128i r_val = _mm_loadu_si128((__m128i*)&rgb_r[j * rgb_stride + i]);
            __m128i g_val = _mm_loadu_si128((__m128i*)&rgb_g[j * rgb_stride + i]);
            __m128i b_val = _mm_loadu_si128((__m128i*)&rgb_b[j * rgb_stride + i]);
            
            /* Convert to float using AVX-512 */
            __m512i r_32 = _mm512_cvtepu8_epi32(r_val);
            __m512i g_32 = _mm512_cvtepu8_epi32(g_val);
            __m512i b_32 = _mm512_cvtepu8_epi32(b_val);
            
            __m512 r_float = _mm512_cvtepi32_ps(r_32);
            __m512 g_float = _mm512_cvtepi32_ps(g_32);
            __m512 b_float = _mm512_cvtepi32_ps(b_32);
            
            /* Store results */
            _mm512_storeu_ps(&planar_r[j * width + i], r_float);
            _mm512_storeu_ps(&planar_g[j * width + i], g_float);
            _mm512_storeu_ps(&planar_b[j * width + i], b_float);
        }
        
        /* Handle remaining pixels */
        for( ; i < width; i++ )
        {
            planar_r[j * width + i] = (float)rgb_r[j * rgb_stride + i];
            planar_g[j * width + i] = (float)rgb_g[j * rgb_stride + i];
            planar_b[j * width + i] = (float)rgb_b[j * rgb_stride + i];
        }
    }
}

/* AVX-512 compute visual mask implementation */
void x264_butteraugli_compute_visual_mask_avx512( const float *ref_r, const float *ref_g, const float *ref_b,
                                                  int ref_stride, float *mask, int mask_stride,
                                                  int width, int height )
{
    const __m512 luma_r = _mm512_set1_ps(0.2126f);
    const __m512 luma_g = _mm512_set1_ps(0.7152f);
    const __m512 luma_b = _mm512_set1_ps(0.0722f);
    const __m512 one = _mm512_set1_ps(1.0f);
    const __m512 two = _mm512_set1_ps(2.0f);
    const __m512 half = _mm512_set1_ps(0.5f);
    const __m512 scale = _mm512_set1_ps(-100.0f);
    
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
        int x = 1;
        
        /* Process 16 pixels at a time (with appropriate masking for edges) */
        for( ; x + 16 < width - 1; x += 16 )
        {
            __m512 sum = _mm512_setzero_ps();
            __m512 sum_sq = _mm512_setzero_ps();
            
            /* Compute statistics over 3x3 neighborhood */
            for( int dy = -1; dy <= 1; dy++ )
            {
                for( int dx = -1; dx <= 1; dx++ )
                {
                    /* Load 16 pixels with offset */
                    __m512 r = _mm512_loadu_ps(&ref_r[(y + dy) * ref_stride + (x + dx)]);
                    __m512 g = _mm512_loadu_ps(&ref_g[(y + dy) * ref_stride + (x + dx)]);
                    __m512 b = _mm512_loadu_ps(&ref_b[(y + dy) * ref_stride + (x + dx)]);
                    
                    /* Compute luma */
                    __m512 luma = _mm512_fmadd_ps(r, luma_r,
                                  _mm512_fmadd_ps(g, luma_g,
                                  _mm512_mul_ps(b, luma_b)));
                    
                    sum = _mm512_add_ps(sum, luma);
                    sum_sq = _mm512_fmadd_ps(luma, luma, sum_sq);
                }
            }
            
            /* Compute mean and variance */
            __m512 mean = _mm512_mul_ps(sum, _mm512_set1_ps(1.0f / 9.0f));
            __m512 variance = _mm512_fnmadd_ps(mean, mean, 
                              _mm512_mul_ps(sum_sq, _mm512_set1_ps(1.0f / 9.0f)));
            
            /* Map variance to masking strength using exp approximation */
            __m512 neg_var_scaled = _mm512_mul_ps(variance, scale);
            
            /* Use fast exp approximation for AVX-512 */
            __m512 exp_val = _mm512_exp_ps(neg_var_scaled);
            __m512 masking = _mm512_add_ps(one, exp_val);
            
            /* Clamp between 0.5 and 2.0 */
            masking = _mm512_min_ps(two, _mm512_max_ps(half, masking));
            
            /* Store results */
            _mm512_storeu_ps(&mask[y * mask_stride + x], masking);
        }
        
        /* Handle remaining pixels */
        for( ; x < width - 1; x++ )
        {
            float sum = 0.0f;
            float sum_sq = 0.0f;
            
            for( int dy = -1; dy <= 1; dy++ )
            {
                for( int dx = -1; dx <= 1; dx++ )
                {
                    int offset = (y + dy) * ref_stride + (x + dx);
                    float luma = 0.2126f * ref_r[offset] + 0.7152f * ref_g[offset] + 0.0722f * ref_b[offset];
                    sum += luma;
                    sum_sq += luma * luma;
                }
            }
            
            float mean = sum / 9.0f;
            float variance = (sum_sq / 9.0f) - (mean * mean);
            float masking = 1.0f + expf(-variance * 100.0f);
            mask[y * mask_stride + x] = fminf(2.0f, fmaxf(0.5f, masking));
        }
    }
}

/* AVX-512 heatmap to uint8 implementation */
void x264_butteraugli_heatmap_to_uint8_avx512( const float *heatmap, int heatmap_stride,
                                               uint8_t *output, int output_stride,
                                               int width, int height, float scale )
{
    const __m512 scale_vec = _mm512_set1_ps(scale);
    const __m512 zero = _mm512_setzero_ps();
    const __m512 max_val = _mm512_set1_ps(255.0f);
    
    for( int y = 0; y < height; y++ )
    {
        int x = 0;
        
        /* Process 16 pixels at a time */
        for( ; x + 15 < width; x += 16 )
        {
            /* Load 16 float values */
            __m512 val = _mm512_loadu_ps(&heatmap[y * heatmap_stride + x]);
            
            /* Scale values */
            val = _mm512_mul_ps(val, scale_vec);
            
            /* Clamp to 0-255 */
            val = _mm512_min_ps(max_val, _mm512_max_ps(zero, val));
            
            /* Convert to int32 then to uint8 */
            __m512i val_i32 = _mm512_cvtps_epi32(val);
            __m256i val_i16 = _mm512_cvtepi32_epi16(val_i32);
            __m128i val_u8 = _mm256_cvtepi16_epi8(val_i16);
            
            /* Store results */
            _mm_storeu_si128((__m128i*)&output[y * output_stride + x], val_u8);
        }
        
        /* Handle remaining pixels */
        for( ; x < width; x++ )
        {
            float val = heatmap[y * heatmap_stride + x] * scale;
            output[y * output_stride + x] = (uint8_t)x264_clip3f( val, 0.0f, 255.0f );
        }
    }
}

#else /* !__AVX512F__ */

/* Stub implementations when AVX-512 is not available */
void x264_butteraugli_yuv_to_rgb_avx512( const uint8_t *y, const uint8_t *u, const uint8_t *v,
                                         int y_stride, int uv_stride,
                                         uint8_t *rgb_r, uint8_t *rgb_g, uint8_t *rgb_b, int rgb_stride,
                                         int width, int height, x264_butteraugli_colorspace_t colorspace )
{
    x264_butteraugli_yuv_to_rgb_avx2( y, u, v, y_stride, uv_stride,
                                      rgb_r, rgb_g, rgb_b, rgb_stride,
                                      width, height, colorspace );
}

void x264_butteraugli_srgb_to_linear_avx512( const uint8_t *srgb, float *linear,
                                             int width, int height, int stride )
{
    x264_butteraugli_srgb_to_linear_avx2( srgb, linear, width, height, stride );
}

void x264_butteraugli_create_image_planar_avx512( const uint8_t *rgb_r, const uint8_t *rgb_g, const uint8_t *rgb_b,
                                                  int rgb_stride, float *planar_r, float *planar_g, float *planar_b,
                                                  int width, int height )
{
    x264_butteraugli_create_image_planar_avx2( rgb_r, rgb_g, rgb_b, rgb_stride,
                                              planar_r, planar_g, planar_b, width, height );
}

void x264_butteraugli_compute_visual_mask_avx512( const float *ref_r, const float *ref_g, const float *ref_b,
                                                  int ref_stride, float *mask, int mask_stride,
                                                  int width, int height )
{
    x264_butteraugli_compute_visual_mask_avx2( ref_r, ref_g, ref_b, ref_stride,
                                              mask, mask_stride, width, height );
}

void x264_butteraugli_heatmap_to_uint8_avx512( const float *heatmap, int heatmap_stride,
                                               uint8_t *output, int output_stride,
                                               int width, int height, float scale )
{
    x264_butteraugli_heatmap_to_uint8_avx2( heatmap, heatmap_stride, output, output_stride,
                                            width, height, scale );
}

#endif /* __AVX512F__ */

#endif /* HAVE_MMX */