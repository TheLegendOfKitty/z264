#include <iostream>
#include <vector>
#include <string>
#include <fstream>

// Include butteraugli directly
#include "third_party/butteraugli/butteraugli/butteraugli.h"

using namespace butteraugli;

// Simple function to load a PGM file (for testing)
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

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cout << "Usage: " << argv[0] << " <reference.pgm> <encoded.pgm>" << std::endl;
        std::cout << "This is a simple test that requires PGM format images." << std::endl;
        std::cout << "Convert images using: convert image.png image.pgm" << std::endl;
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
    
    // Calculate butteraugli distance
    ImageF diffmap(width, height);
    double diffvalue;
    
    if (ButteraugliInterface(ref_rgb, enc_rgb, 1.0f, diffmap, diffvalue)) {
        std::cout << "Butteraugli distance: " << diffvalue << std::endl;
        
        if (diffvalue < 0.26) {
            std::cout << "Quality: GOOD (images appear identical)" << std::endl;
        } else if (diffvalue < 1.454) {
            std::cout << "Quality: OK (subtle differences)" << std::endl;
        } else {
            std::cout << "Quality: BAD (noticeable differences)" << std::endl;
        }
    } else {
        std::cerr << "Failed to calculate butteraugli distance" << std::endl;
        return 1;
    }
    
    return 0;
}