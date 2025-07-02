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
                row[x] = Srgb8ToLinear(src[y * stride + x]);
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
        // Enhanced version with multiple spatial frequency bands
        
        // First pass: Compute luminance and basic statistics
        butteraugli::ImageF luma(ctx->width, ctx->height);
        for (int y = 0; y < ctx->height; y++) {
            for (int x = 0; x < ctx->width; x++) {
                // BT.709 luma coefficients for linear RGB
                float l = 0.2126f * ref_image[0].Row(y)[x] +
                         0.7152f * ref_image[1].Row(y)[x] +
                         0.0722f * ref_image[2].Row(y)[x];
                luma.Row(y)[x] = l;
            }
        }
        
        // Second pass: Multi-scale edge detection and variance computation
        for (int y = 0; y < ctx->height; y++) {
            float* mask_row = mask + y * mask_stride;
            
            for (int x = 0; x < ctx->width; x++) {
                float masking = 0.0f;
                float total_weight = 0.0f;
                
                // Analyze at multiple scales (3x3, 5x5, 7x7)
                const int scales[] = {1, 2, 3};
                const float scale_weights[] = {0.5f, 0.3f, 0.2f};
                
                for (int s = 0; s < 3; s++) {
                    int radius = scales[s];
                    float weight = scale_weights[s];
                    
                    float sum = 0.0f;
                    float sum_sq = 0.0f;
                    float edge_strength = 0.0f;
                    int count = 0;
                    
                    // Compute variance and edge detection in window
                    for (int dy = -radius; dy <= radius; dy++) {
                        for (int dx = -radius; dx <= radius; dx++) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && nx < ctx->width && ny >= 0 && ny < ctx->height) {
                                float l = luma.Row(ny)[nx];
                                sum += l;
                                sum_sq += l * l;
                                count++;
                                
                                // Sobel edge detection components
                                if (dx != 0 && dy != 0 && nx > 0 && nx < ctx->width-1 && 
                                    ny > 0 && ny < ctx->height-1) {
                                    float gx = luma.Row(ny)[nx+1] - luma.Row(ny)[nx-1];
                                    float gy = luma.Row(ny+1)[nx] - luma.Row(ny-1)[nx];
                                    edge_strength += std::sqrt(gx*gx + gy*gy);
                                }
                            }
                        }
                    }
                    
                    float mean = sum / count;
                    float variance = (sum_sq / count) - (mean * mean);
                    edge_strength /= count;
                    
                    // Combine variance and edge information
                    // High frequency content = less masking
                    float activity = variance + 0.1f * edge_strength;
                    
                    // Map activity to masking strength
                    // Using a more sophisticated transfer function
                    float scale_masking;
                    if (activity < 0.001f) {
                        scale_masking = 2.0f;  // Very smooth areas: maximum masking
                    } else if (activity < 0.01f) {
                        scale_masking = 1.5f + 0.5f * std::exp(-activity * 500.0f);
                    } else if (activity < 0.1f) {
                        scale_masking = 1.0f + 0.5f * std::exp(-activity * 50.0f);
                    } else {
                        scale_masking = 0.8f + 0.2f * std::exp(-activity * 5.0f);
                    }
                    
                    masking += weight * scale_masking;
                    total_weight += weight;
                }
                
                masking /= total_weight;
                mask_row[x] = std::min(2.0f, std::max(0.5f, masking));
            }
        }
        
        // Apply edge-preserving filter to smooth the mask
        std::vector<float> temp_mask(ctx->width * ctx->height);
        for (int y = 1; y < ctx->height - 1; y++) {
            for (int x = 1; x < ctx->width - 1; x++) {
                float center = mask[y * mask_stride + x];
                float sum = center * 4.0f;  // Give more weight to center
                float weight_sum = 4.0f;
                
                // Bilateral-like filtering
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        
                        float neighbor = mask[(y + dy) * mask_stride + (x + dx)];
                        float diff = std::abs(neighbor - center);
                        float weight = std::exp(-diff * 10.0f);  // Edge preservation
                        
                        sum += neighbor * weight;
                        weight_sum += weight;
                    }
                }
                temp_mask[y * ctx->width + x] = sum / weight_sum;
            }
        }
        
        // Copy back with border handling
        for (int y = 0; y < ctx->height; y++) {
            for (int x = 0; x < ctx->width; x++) {
                if (y == 0 || y == ctx->height - 1 || x == 0 || x == ctx->width - 1) {
                    mask[y * mask_stride + x] = 1.0f;  // Neutral masking at borders
                } else {
                    mask[y * mask_stride + x] = temp_mask[y * ctx->width + x];
                }
            }
        }
        
    } catch (...) {
        // Fill with neutral value on error
        for (int y = 0; y < ctx->height; y++) {
            std::fill_n(mask + y * mask_stride, ctx->width, 1.0f);
        }
    }
}

double butteraugli_fuzzy_class(double score) {
    try {
        return butteraugli::ButteraugliFuzzyClass(score);
    } catch (...) {
        return 0.0;
    }
}

double butteraugli_fuzzy_inverse(double seek) {
    try {
        return butteraugli::ButteraugliFuzzyInverse(seek);
    } catch (...) {
        return 1.0;
    }
}

bool butteraugli_adaptive_quantization(int width, int height,
                                       const uint8_t* rgb_r, const uint8_t* rgb_g, const uint8_t* rgb_b,
                                       int rgb_stride, float* quant_map, int quant_stride) {
    try {
        // Create vector of vectors for RGB data as expected by butteraugli API
        std::vector<std::vector<float>> rgb(3);
        for (int c = 0; c < 3; c++) {
            rgb[c].resize(width * height);
        }
        
        // Convert uint8_t RGB to float and copy to vectors
        const uint8_t* channels[3] = { rgb_r, rgb_g, rgb_b };
        for (int c = 0; c < 3; c++) {
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    // Convert sRGB to linear as butteraugli expects
                    float val = Srgb8ToLinear(channels[c][y * rgb_stride + x]);
                    rgb[c][y * width + x] = val;
                }
            }
        }
        
        // Call butteraugli adaptive quantization
        std::vector<float> quant;
        if (!butteraugli::ButteraugliAdaptiveQuantization(width, height, rgb, quant)) {
            return false;
        }
        
        // Copy quantization map to output with proper stride
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                quant_map[y * quant_stride + x] = quant[y * width + x];
            }
        }
        
        return true;
    } catch (...) {
        return false;
    }
}

} // extern "C"

#endif // HAVE_BUTTERAUGLI