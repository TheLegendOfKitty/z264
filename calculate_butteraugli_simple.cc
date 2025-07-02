#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cmath>

// Include butteraugli
#include "third_party/butteraugli/butteraugli/butteraugli.h"

using namespace butteraugli;

// Load raw YUV420 frame
bool load_yuv420_frame(const char* filename, int width, int height, int frame_num,
                       uint8_t* y_plane, uint8_t* u_plane, uint8_t* v_plane) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }
    
    size_t frame_size = width * height * 3 / 2;
    size_t offset = frame_size * frame_num;
    
    file.seekg(offset);
    
    // Read Y plane
    file.read(reinterpret_cast<char*>(y_plane), width * height);
    // Read U plane (quarter size)
    file.read(reinterpret_cast<char*>(u_plane), width * height / 4);
    // Read V plane (quarter size)
    file.read(reinterpret_cast<char*>(v_plane), width * height / 4);
    
    return file.good();
}

// Convert YUV to RGB and create butteraugli images
void create_butteraugli_images(int width, int height,
                              const uint8_t* y_plane, const uint8_t* u_plane, const uint8_t* v_plane,
                              std::vector<ImageF>& rgb_images) {
    rgb_images.resize(3);
    for (int c = 0; c < 3; c++) {
        rgb_images[c] = ImageF(width, height);
    }
    
    // Simple YUV to RGB conversion (BT.709)
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int y_idx = y * width + x;
            int uv_idx = (y/2) * (width/2) + (x/2);
            
            float Y = y_plane[y_idx];
            float U = u_plane[uv_idx] - 128.0f;
            float V = v_plane[uv_idx] - 128.0f;
            
            float R = Y + 1.5748f * V;
            float G = Y - 0.1873f * U - 0.4681f * V;
            float B = Y + 1.8556f * U;
            
            // Clamp and convert to linear (approximate sRGB to linear)
            R = std::max(0.0f, std::min(255.0f, R)) / 255.0f;
            G = std::max(0.0f, std::min(255.0f, G)) / 255.0f;
            B = std::max(0.0f, std::min(255.0f, B)) / 255.0f;
            
            // Apply gamma correction (sRGB to linear)
            auto srgb_to_linear = [](float val) {
                if (val <= 0.04045f) {
                    return val / 12.92f;
                } else {
                    return std::pow((val + 0.055f) / 1.055f, 2.4f);
                }
            };
            
            rgb_images[0].Row(y)[x] = srgb_to_linear(R);
            rgb_images[1].Row(y)[x] = srgb_to_linear(G);
            rgb_images[2].Row(y)[x] = srgb_to_linear(B);
        }
    }
}

int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cout << "Usage: " << argv[0] << " <reference.yuv> <encoded.yuv> <width> <height>" << std::endl;
        std::cout << "This tool calculates butteraugli distance between YUV420 files" << std::endl;
        return 1;
    }
    
    const char* ref_file = argv[1];
    const char* enc_file = argv[2];
    int width = std::atoi(argv[3]);
    int height = std::atoi(argv[4]);
    
    std::cout << "Calculating butteraugli distance for " << width << "x" << height << " video" << std::endl;
    
    // Allocate buffers for YUV planes
    std::vector<uint8_t> ref_y(width * height);
    std::vector<uint8_t> ref_u(width * height / 4);
    std::vector<uint8_t> ref_v(width * height / 4);
    std::vector<uint8_t> enc_y(width * height);
    std::vector<uint8_t> enc_u(width * height / 4);
    std::vector<uint8_t> enc_v(width * height / 4);
    
    // Process first 10 frames
    double total_distance = 0.0;
    int num_frames = 0;
    
    for (int frame = 0; frame < 10; frame++) {
        // Load reference frame
        if (!load_yuv420_frame(ref_file, width, height, frame, 
                              ref_y.data(), ref_u.data(), ref_v.data())) {
            break;
        }
        
        // Load encoded frame
        if (!load_yuv420_frame(enc_file, width, height, frame,
                              enc_y.data(), enc_u.data(), enc_v.data())) {
            break;
        }
        
        // Convert to butteraugli RGB images
        std::vector<ImageF> ref_rgb, enc_rgb;
        create_butteraugli_images(width, height, ref_y.data(), ref_u.data(), ref_v.data(), ref_rgb);
        create_butteraugli_images(width, height, enc_y.data(), enc_u.data(), enc_v.data(), enc_rgb);
        
        // Calculate butteraugli distance
        ImageF diffmap(width, height);
        double diffvalue;
        
        if (ButteraugliInterface(ref_rgb, enc_rgb, 1.0f, diffmap, diffvalue)) {
            std::cout << "Frame " << frame << " butteraugli distance: " << diffvalue << std::endl;
            total_distance += diffvalue;
            num_frames++;
        } else {
            std::cerr << "Failed to calculate butteraugli distance for frame " << frame << std::endl;
        }
    }
    
    if (num_frames > 0) {
        double avg_distance = total_distance / num_frames;
        std::cout << "\nAverage butteraugli distance: " << avg_distance << std::endl;
        
        if (avg_distance < kButteraugliQuantLow) {
            std::cout << "Quality: EXCELLENT (imperceptible difference)" << std::endl;
        } else if (avg_distance < kButteraugliQuantHigh) {
            std::cout << "Quality: GOOD (subtle differences)" << std::endl;
        } else {
            std::cout << "Quality: POOR (noticeable differences)" << std::endl;
        }
    }
    
    return 0;
}