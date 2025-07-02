#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <iomanip>

// Include butteraugli directly
#include "third_party/butteraugli/butteraugli/butteraugli.h"

using namespace butteraugli;

// Simple function to load a PGM file
bool load_pgm(const std::string& filename, std::vector<float>& image, int& width, int& height) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) return false;
    
    std::string magic;
    file >> magic;
    if (magic != "P5") return false;
    
    file >> width >> height;
    int maxval;
    file >> maxval;
    file.ignore(); // Skip newline
    
    std::vector<uint8_t> raw_data(width * height);
    file.read(reinterpret_cast<char*>(raw_data.data()), width * height);
    
    image.resize(width * height);
    for (size_t i = 0; i < raw_data.size(); i++) {
        image[i] = raw_data[i] / 255.0f;
    }
    
    return true;
}

// Calculate L3 norm from diffmap
double calculate_l3_norm(const ImageF& diffmap) {
    double sum_cubes = 0.0;
    size_t pixel_count = 0;
    
    for (size_t y = 0; y < diffmap.ysize(); y++) {
        const float* row = diffmap.Row(y);
        for (size_t x = 0; x < diffmap.xsize(); x++) {
            double val = static_cast<double>(row[x]);
            sum_cubes += val * val * val;
            pixel_count++;
        }
    }
    
    return cbrt(sum_cubes / pixel_count);
}

// Calculate infinity norm (maximum value) from diffmap
double calculate_infnorm(const ImageF& diffmap) {
    double max_val = 0.0;
    
    for (size_t y = 0; y < diffmap.ysize(); y++) {
        const float* row = diffmap.Row(y);
        for (size_t x = 0; x < diffmap.xsize(); x++) {
            max_val = std::max(max_val, static_cast<double>(row[x]));
        }
    }
    
    return max_val;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cout << "Usage: " << argv[0] << " <reference.pgm> <encoded.pgm>" << std::endl;
        std::cout << "This calculates butteraugli global score, infnorm, and L3-norm." << std::endl;
        return 1;
    }
    
    std::vector<float> ref_image, enc_image;
    int width, height;
    
    if (!load_pgm(argv[1], ref_image, width, height)) {
        std::cerr << "Failed to load reference image: " << argv[1] << std::endl;
        return 1;
    }
    
    int enc_width, enc_height;
    if (!load_pgm(argv[2], enc_image, enc_width, enc_height)) {
        std::cerr << "Failed to load encoded image: " << argv[2] << std::endl;
        return 1;
    }
    
    if (width != enc_width || height != enc_height) {
        std::cerr << "Image dimensions don't match!" << std::endl;
        return 1;
    }
    
    // Create RGB images from grayscale (duplicate gray to all channels)
    std::vector<ImageF> ref_rgb(3), enc_rgb(3);
    for (int c = 0; c < 3; c++) {
        ref_rgb[c] = ImageF(width, height);
        enc_rgb[c] = ImageF(width, height);
        
        for (int y = 0; y < height; y++) {
            float* ref_row = ref_rgb[c].Row(y);
            float* enc_row = enc_rgb[c].Row(y);
            for (int x = 0; x < width; x++) {
                ref_row[x] = ref_image[y * width + x];
                enc_row[x] = enc_image[y * width + x];
            }
        }
    }
    
    // Calculate butteraugli diffmap and scores
    ImageF diffmap(width, height);
    double global_score;
    
    if (ButteraugliInterface(ref_rgb, enc_rgb, 1.0f, diffmap, global_score)) {
        // Calculate various norms from the diffmap
        double infnorm = calculate_infnorm(diffmap);
        double l3_norm = calculate_l3_norm(diffmap);
        
        std::cout << "Butteraugli_Global_Score: " << global_score << std::endl;
        std::cout << "Butteraugli_InfNorm: " << infnorm << std::endl;
        std::cout << "Butteraugli_L3Norm: " << l3_norm << std::endl;
        
    } else {
        std::cerr << "Failed to calculate butteraugli distance" << std::endl;
        return 1;
    }
    
    return 0;
}