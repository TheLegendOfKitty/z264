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

// Analyze diffmap statistics
void analyze_diffmap(const ImageF& diffmap, const std::string& label) {
    std::vector<float> values;
    
    // Collect all values
    for (size_t y = 0; y < diffmap.ysize(); y++) {
        const float* row = diffmap.Row(y);
        for (size_t x = 0; x < diffmap.xsize(); x++) {
            values.push_back(row[x]);
        }
    }
    
    std::sort(values.begin(), values.end());
    
    double sum = 0, sum_squares = 0;
    for (float val : values) {
        sum += val;
        sum_squares += val * val;
    }
    
    size_t n = values.size();
    double mean = sum / n;
    double stddev = sqrt(sum_squares / n - mean * mean);
    
    std::cout << "\n" << label << " Diffmap Analysis:" << std::endl;
    std::cout << "  Size: " << diffmap.xsize() << "x" << diffmap.ysize() << " (" << n << " pixels)" << std::endl;
    std::cout << "  Min: " << values[0] << std::endl;
    std::cout << "  Max: " << values[n-1] << std::endl;
    std::cout << "  Mean: " << std::fixed << std::setprecision(6) << mean << std::endl;
    std::cout << "  StdDev: " << stddev << std::endl;
    std::cout << "  Median: " << values[n/2] << std::endl;
    std::cout << "  95th percentile: " << values[size_t(n * 0.95)] << std::endl;
    std::cout << "  99th percentile: " << values[size_t(n * 0.99)] << std::endl;
    std::cout << "  99.9th percentile: " << values[size_t(n * 0.999)] << std::endl;
    
    // Count pixels above certain thresholds
    int count_01 = 0, count_05 = 0, count_10 = 0;
    for (float val : values) {
        if (val > 0.1) count_01++;
        if (val > 0.5) count_05++;
        if (val > 1.0) count_10++;
    }
    
    std::cout << "  Pixels > 0.1: " << count_01 << " (" << (100.0 * count_01 / n) << "%)" << std::endl;
    std::cout << "  Pixels > 0.5: " << count_05 << " (" << (100.0 * count_05 / n) << "%)" << std::endl;
    std::cout << "  Pixels > 1.0: " << count_10 << " (" << (100.0 * count_10 / n) << "%)" << std::endl;
}

// Save diffmap as PGM for visualization
void save_diffmap_as_pgm(const ImageF& diffmap, const std::string& filename) {
    // Find max value for normalization
    float max_val = 0;
    for (size_t y = 0; y < diffmap.ysize(); y++) {
        const float* row = diffmap.Row(y);
        for (size_t x = 0; x < diffmap.xsize(); x++) {
            max_val = std::max(max_val, row[x]);
        }
    }
    
    std::ofstream file(filename, std::ios::binary);
    file << "P5\n" << diffmap.xsize() << " " << diffmap.ysize() << "\n255\n";
    
    for (size_t y = 0; y < diffmap.ysize(); y++) {
        const float* row = diffmap.Row(y);
        for (size_t x = 0; x < diffmap.xsize(); x++) {
            uint8_t pixel = uint8_t(255 * row[x] / max_val);
            file.write(reinterpret_cast<const char*>(&pixel), 1);
        }
    }
}

// Compare two diffmaps pixel by pixel
void compare_diffmaps(const ImageF& diffmap1, const ImageF& diffmap2, const std::string& label1, const std::string& label2) {
    if (diffmap1.xsize() != diffmap2.xsize() || diffmap1.ysize() != diffmap2.ysize()) {
        std::cout << "Diffmaps have different dimensions!" << std::endl;
        return;
    }
    
    double max_diff = 0;
    double sum_diff = 0;
    double sum_abs_diff = 0;
    size_t diff_pixels = 0;
    size_t total_pixels = 0;
    
    for (size_t y = 0; y < diffmap1.ysize(); y++) {
        const float* row1 = diffmap1.Row(y);
        const float* row2 = diffmap2.Row(y);
        for (size_t x = 0; x < diffmap1.xsize(); x++) {
            double diff = row2[x] - row1[x];
            sum_diff += diff;
            sum_abs_diff += std::abs(diff);
            max_diff = std::max(max_diff, std::abs(diff));
            
            if (std::abs(diff) > 1e-8) diff_pixels++;
            total_pixels++;
        }
    }
    
    std::cout << "\nDiffmap Comparison (" << label1 << " vs " << label2 << "):" << std::endl;
    std::cout << "  Max absolute difference: " << std::scientific << max_diff << std::endl;
    std::cout << "  Mean difference: " << sum_diff / total_pixels << std::endl;
    std::cout << "  Mean absolute difference: " << sum_abs_diff / total_pixels << std::endl;
    std::cout << "  Different pixels: " << diff_pixels << "/" << total_pixels;
    std::cout << " (" << (100.0 * diff_pixels / total_pixels) << "%)" << std::endl;
    
    if (diff_pixels == 0) {
        std::cout << "  Result: IDENTICAL diffmaps" << std::endl;
    } else if (max_diff < 1e-6) {
        std::cout << "  Result: NEARLY IDENTICAL (floating point precision)" << std::endl;
    } else {
        std::cout << "  Result: DIFFERENT diffmaps" << std::endl;
    }
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cout << "Usage: " << argv[0] << " <reference.pgm> <encoded1.pgm> <encoded2.pgm>" << std::endl;
        std::cout << "This performs detailed diffmap analysis comparing two encoded images to reference." << std::endl;
        return 1;
    }
    
    std::vector<float> ref_image, enc1_image, enc2_image;
    int width, height;
    
    if (!load_pgm(argv[1], ref_image, width, height)) {
        std::cerr << "Failed to load reference image: " << argv[1] << std::endl;
        return 1;
    }
    
    int enc1_width, enc1_height, enc2_width, enc2_height;
    if (!load_pgm(argv[2], enc1_image, enc1_width, enc1_height)) {
        std::cerr << "Failed to load first encoded image: " << argv[2] << std::endl;
        return 1;
    }
    
    if (!load_pgm(argv[3], enc2_image, enc2_width, enc2_height)) {
        std::cerr << "Failed to load second encoded image: " << argv[3] << std::endl;
        return 1;
    }
    
    if (width != enc1_width || height != enc1_height || width != enc2_width || height != enc2_height) {
        std::cerr << "Image dimensions don't match!" << std::endl;
        return 1;
    }
    
    // Create RGB images from grayscale
    std::vector<ImageF> ref_rgb(3), enc1_rgb(3), enc2_rgb(3);
    for (int c = 0; c < 3; c++) {
        ref_rgb[c] = ImageF(width, height);
        enc1_rgb[c] = ImageF(width, height);
        enc2_rgb[c] = ImageF(width, height);
        
        for (int y = 0; y < height; y++) {
            float* ref_row = ref_rgb[c].Row(y);
            float* enc1_row = enc1_rgb[c].Row(y);
            float* enc2_row = enc2_rgb[c].Row(y);
            for (int x = 0; x < width; x++) {
                ref_row[x] = ref_image[y * width + x];
                enc1_row[x] = enc1_image[y * width + x];
                enc2_row[x] = enc2_image[y * width + x];
            }
        }
    }
    
    // Calculate butteraugli diffmaps
    ImageF diffmap1(width, height), diffmap2(width, height);
    double diffvalue1, diffvalue2;
    
    if (!ButteraugliInterface(ref_rgb, enc1_rgb, 1.0f, diffmap1, diffvalue1)) {
        std::cerr << "Failed to calculate butteraugli for first image" << std::endl;
        return 1;
    }
    
    if (!ButteraugliInterface(ref_rgb, enc2_rgb, 1.0f, diffmap2, diffvalue2)) {
        std::cerr << "Failed to calculate butteraugli for second image" << std::endl;
        return 1;
    }
    
    std::cout << "Global Scores:" << std::endl;
    std::cout << "  Image 1 (" << argv[2] << "): " << diffvalue1 << std::endl;
    std::cout << "  Image 2 (" << argv[3] << "): " << diffvalue2 << std::endl;
    std::cout << "  Difference: " << std::abs(diffvalue2 - diffvalue1) << std::endl;
    
    analyze_diffmap(diffmap1, "Image 1");
    analyze_diffmap(diffmap2, "Image 2");
    
    compare_diffmaps(diffmap1, diffmap2, "Image 1", "Image 2");
    
    // Save diffmaps for visual inspection
    save_diffmap_as_pgm(diffmap1, "diffmap1.pgm");
    save_diffmap_as_pgm(diffmap2, "diffmap2.pgm");
    std::cout << "\nDiffmaps saved as diffmap1.pgm and diffmap2.pgm" << std::endl;
    
    return 0;
}