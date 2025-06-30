/*****************************************************************************
 * butteraugli_wrapper.cc: C++ wrapper for butteraugli library
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

#ifdef HAVE_BUTTERAUGLI

#include <cstring>
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>

#include "butteraugli/butteraugli.h"

extern "C" {

struct ButteraugliContext {
    int width;
    int height;
    int num_threads;
    float hf_asymmetry;
    
    ButteraugliContext(int w, int h, int threads) 
        : width(w), height(h), num_threads(threads), hf_asymmetry(1.0f) {
    }
};

void* butteraugli_create_context(int width, int height, int num_threads) {
    try {
        return new ButteraugliContext(width, height, num_threads);
    } catch (...) {
        return nullptr;
    }
}

void butteraugli_destroy_context(void* context) {
    delete static_cast<ButteraugliContext*>(context);
}

// Helper function to convert sRGB to linear
static inline float Srgb8ToLinear(uint8_t val) {
    float srgb = val / 255.0f;
    if (srgb <= 0.04045f) {
        return srgb / 12.92f;
    } else {
        return std::pow((srgb + 0.055f) / 1.055f, 2.4f);
    }
}

// Helper function to create ImageF vector from planar RGB data
static std::vector<butteraugli::ImageF> CreateImageFromPlanar(
    const uint8_t* r, const uint8_t* g, const uint8_t* b,
    int stride, int width, int height) {
    
    std::vector<butteraugli::ImageF> image;
    image.reserve(3);
    
    // Create R, G, B planes
    for (int c = 0; c < 3; c++) {
        image.emplace_back(width, height);
        const uint8_t* src = (c == 0) ? r : (c == 1) ? g : b;
        
        for (int y = 0; y < height; y++) {
            float* row = image[c].Row(y);
            for (int x = 0; x < width; x++) {
                row[x] = Srgb8ToLinear(src[y * stride + x * 3]);
            }
        }
    }
    
    return image;
}

float butteraugli_compute_distance(void* context,
                                   const uint8_t* ref_r, const uint8_t* ref_g, const uint8_t* ref_b,
                                   int ref_stride,
                                   const uint8_t* enc_r, const uint8_t* enc_g, const uint8_t* enc_b,
                                   int enc_stride) {
    ButteraugliContext* ctx = static_cast<ButteraugliContext*>(context);
    if (!ctx) return 0.0f;
    
    try {
        // Create butteraugli images
        std::vector<butteraugli::ImageF> ref_image = CreateImageFromPlanar(
            ref_r, ref_g, ref_b, ref_stride, ctx->width, ctx->height);
        std::vector<butteraugli::ImageF> enc_image = CreateImageFromPlanar(
            enc_r, enc_g, enc_b, enc_stride, ctx->width, ctx->height);
        
        // Compute butteraugli distance
        butteraugli::ImageF diff_map;
        double diff_value;
        
        if (!butteraugli::ButteraugliInterface(ref_image, enc_image, ctx->hf_asymmetry,
                                              diff_map, diff_value)) {
            return 0.0f;
        }
        
        return static_cast<float>(diff_value);
        
    } catch (...) {
        return 0.0f;
    }
}

void butteraugli_compute_heatmap(void* context,
                                 const uint8_t* ref_r, const uint8_t* ref_g, const uint8_t* ref_b,
                                 int ref_stride,
                                 const uint8_t* enc_r, const uint8_t* enc_g, const uint8_t* enc_b,
                                 int enc_stride,
                                 float* heatmap, int heatmap_stride) {
    ButteraugliContext* ctx = static_cast<ButteraugliContext*>(context);
    if (!ctx) return;
    
    try {
        // Create butteraugli images
        std::vector<butteraugli::ImageF> ref_image = CreateImageFromPlanar(
            ref_r, ref_g, ref_b, ref_stride, ctx->width, ctx->height);
        std::vector<butteraugli::ImageF> enc_image = CreateImageFromPlanar(
            enc_r, enc_g, enc_b, enc_stride, ctx->width, ctx->height);
        
        // Compute butteraugli heatmap
        butteraugli::ImageF diff_map;
        double diff_value;
        
        if (!butteraugli::ButteraugliInterface(ref_image, enc_image, ctx->hf_asymmetry,
                                              diff_map, diff_value)) {
            return;
        }
        
        // Copy heatmap data
        for (int y = 0; y < ctx->height; y++) {
            const float* src_row = diff_map.Row(y);
            float* dst_row = heatmap + y * heatmap_stride;
            std::memcpy(dst_row, src_row, ctx->width * sizeof(float));
        }
        
    } catch (...) {
        // Fill with zeros on error
        for (int y = 0; y < ctx->height; y++) {
            std::memset(heatmap + y * heatmap_stride, 0, ctx->width * sizeof(float));
        }
    }
}

void butteraugli_compute_mask(void* context,
                              const uint8_t* ref_r, const uint8_t* ref_g, const uint8_t* ref_b,
                              int ref_stride,
                              float* mask, int mask_stride) {
    ButteraugliContext* ctx = static_cast<ButteraugliContext*>(context);
    if (!ctx) return;
    
    try {
        // Create butteraugli image
        std::vector<butteraugli::ImageF> ref_image = CreateImageFromPlanar(
            ref_r, ref_g, ref_b, ref_stride, ctx->width, ctx->height);
        
        // Compute visual masking based on image complexity
        // This is a simplified version - butteraugli doesn't expose the exact masking
        // so we approximate based on local variance and edge detection
        
        for (int y = 0; y < ctx->height; y++) {
            float* mask_row = mask + y * mask_stride;
            
            for (int x = 0; x < ctx->width; x++) {
                // Compute local variance in a 3x3 window
                float sum = 0.0f;
                float sum_sq = 0.0f;
                int count = 0;
                
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < ctx->width && ny >= 0 && ny < ctx->height) {
                            // Use luma from linear RGB
                            float luma = 0.299f * ref_image[0].Row(ny)[nx] +
                                        0.587f * ref_image[1].Row(ny)[nx] +
                                        0.114f * ref_image[2].Row(ny)[nx];
                            sum += luma;
                            sum_sq += luma * luma;
                            count++;
                        }
                    }
                }
                
                float mean = sum / count;
                float variance = (sum_sq / count) - (mean * mean);
                
                // Map variance to masking factor
                // High variance (edges, texture) = less masking (value closer to 1.0)
                // Low variance (smooth areas) = more masking (value closer to 2.0)
                float masking = 1.0f + std::exp(-variance * 100.0f);
                mask_row[x] = std::min(2.0f, std::max(0.5f, masking));
            }
        }
        
        // Apply spatial blur to smooth the mask
        // Simple box filter for now
        std::vector<float> temp_mask(ctx->width * ctx->height);
        for (int y = 1; y < ctx->height - 1; y++) {
            for (int x = 1; x < ctx->width - 1; x++) {
                float sum = 0.0f;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        sum += mask[(y + dy) * mask_stride + (x + dx)];
                    }
                }
                temp_mask[y * ctx->width + x] = sum / 9.0f;
            }
        }
        
        // Copy back
        for (int y = 1; y < ctx->height - 1; y++) {
            std::memcpy(mask + y * mask_stride + 1,
                       &temp_mask[y * ctx->width + 1],
                       (ctx->width - 2) * sizeof(float));
        }
        
    } catch (...) {
        // Fill with neutral value on error
        for (int y = 0; y < ctx->height; y++) {
            std::fill_n(mask + y * mask_stride, ctx->width, 1.0f);
        }
    }
}

} // extern "C"

#endif // HAVE_BUTTERAUGLI