#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cstring>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

// Include our butteraugli wrapper
extern "C" {
    void* butteraugli_create_context(int width, int height);
    void butteraugli_destroy_context(void* context);
    float butteraugli_compute_distance(void* context,
                                       const uint8_t* ref_r, const uint8_t* ref_g, const uint8_t* ref_b,
                                       int ref_stride,
                                       const uint8_t* enc_r, const uint8_t* enc_g, const uint8_t* enc_b,
                                       int enc_stride);
}

struct VideoFrame {
    uint8_t* data[4];
    int linesize[4];
    int width, height;
    
    VideoFrame(int w, int h) : width(w), height(h) {
        memset(data, 0, sizeof(data));
        memset(linesize, 0, sizeof(linesize));
    }
    
    ~VideoFrame() {
        if (data[0]) av_freep(&data[0]);
    }
};

class VideoDecoder {
private:
    AVFormatContext* fmt_ctx = nullptr;
    AVCodecContext* codec_ctx = nullptr;
    AVPacket* packet = nullptr;
    AVFrame* frame = nullptr;
    SwsContext* sws_ctx = nullptr;
    int video_stream_idx = -1;
    
public:
    VideoDecoder() {
        packet = av_packet_alloc();
        frame = av_frame_alloc();
    }
    
    ~VideoDecoder() {
        if (sws_ctx) sws_freeContext(sws_ctx);
        if (codec_ctx) avcodec_free_context(&codec_ctx);
        if (fmt_ctx) avformat_close_input(&fmt_ctx);
        if (frame) av_frame_free(&frame);
        if (packet) av_packet_free(&packet);
    }
    
    bool open(const std::string& filename) {
        if (avformat_open_input(&fmt_ctx, filename.c_str(), nullptr, nullptr) < 0) {
            std::cerr << "Error opening file: " << filename << std::endl;
            return false;
        }
        
        if (avformat_find_stream_info(fmt_ctx, nullptr) < 0) {
            std::cerr << "Error finding stream info" << std::endl;
            return false;
        }
        
        // Find video stream
        for (unsigned i = 0; i < fmt_ctx->nb_streams; i++) {
            if (fmt_ctx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) {
                video_stream_idx = i;
                break;
            }
        }
        
        if (video_stream_idx == -1) {
            std::cerr << "No video stream found" << std::endl;
            return false;
        }
        
        // Find decoder
        const AVCodec* codec = avcodec_find_decoder(fmt_ctx->streams[video_stream_idx]->codecpar->codec_id);
        if (!codec) {
            std::cerr << "Decoder not found" << std::endl;
            return false;
        }
        
        codec_ctx = avcodec_alloc_context3(codec);
        if (!codec_ctx) {
            std::cerr << "Failed to allocate codec context" << std::endl;
            return false;
        }
        
        if (avcodec_parameters_to_context(codec_ctx, fmt_ctx->streams[video_stream_idx]->codecpar) < 0) {
            std::cerr << "Failed to copy codec parameters" << std::endl;
            return false;
        }
        
        if (avcodec_open2(codec_ctx, codec, nullptr) < 0) {
            std::cerr << "Failed to open codec" << std::endl;
            return false;
        }
        
        return true;
    }
    
    bool decode_frame(VideoFrame& out_frame) {
        while (true) {
            int ret = av_read_frame(fmt_ctx, packet);
            if (ret < 0) return false; // EOF or error
            
            if (packet->stream_index != video_stream_idx) {
                av_packet_unref(packet);
                continue;
            }
            
            ret = avcodec_send_packet(codec_ctx, packet);
            av_packet_unref(packet);
            
            if (ret < 0) continue;
            
            ret = avcodec_receive_frame(codec_ctx, frame);
            if (ret < 0) continue;
            
            // Convert to RGB
            if (!sws_ctx) {
                sws_ctx = sws_getContext(
                    frame->width, frame->height, (AVPixelFormat)frame->format,
                    frame->width, frame->height, AV_PIX_FMT_RGB24,
                    SWS_BILINEAR, nullptr, nullptr, nullptr);
            }
            
            if (!out_frame.data[0]) {
                out_frame.width = frame->width;
                out_frame.height = frame->height;
                av_image_alloc(out_frame.data, out_frame.linesize, 
                              frame->width, frame->height, AV_PIX_FMT_RGB24, 32);
            }
            
            sws_scale(sws_ctx, frame->data, frame->linesize, 0, frame->height,
                     out_frame.data, out_frame.linesize);
            
            return true;
        }
    }
    
    int get_width() const { return codec_ctx ? codec_ctx->width : 0; }
    int get_height() const { return codec_ctx ? codec_ctx->height : 0; }
};

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <reference_video> <encoded_video>" << std::endl;
        return 1;
    }
    
    std::string ref_file = argv[1];
    std::string enc_file = argv[2];
    
    VideoDecoder ref_decoder, enc_decoder;
    
    if (!ref_decoder.open(ref_file) || !enc_decoder.open(enc_file)) {
        std::cerr << "Failed to open video files" << std::endl;
        return 1;
    }
    
    int width = ref_decoder.get_width();
    int height = ref_decoder.get_height();
    
    if (width != enc_decoder.get_width() || height != enc_decoder.get_height()) {
        std::cerr << "Video dimensions don't match" << std::endl;
        return 1;
    }
    
    void* butteraugli_ctx = butteraugli_create_context(width, height);
    if (!butteraugli_ctx) {
        std::cerr << "Failed to create butteraugli context" << std::endl;
        return 1;
    }
    
    VideoFrame ref_frame(width, height);
    VideoFrame enc_frame(width, height);
    
    std::vector<float> scores;
    int frame_count = 0;
    
    std::cout << "Frame,Butteraugli_Score" << std::endl;
    
    while (ref_decoder.decode_frame(ref_frame) && enc_decoder.decode_frame(enc_frame)) {
        // Extract RGB planes
        uint8_t* ref_rgb = ref_frame.data[0];
        uint8_t* enc_rgb = enc_frame.data[0];
        int rgb_stride = ref_frame.linesize[0];
        
        // Separate RGB channels
        std::vector<uint8_t> ref_r(width * height), ref_g(width * height), ref_b(width * height);
        std::vector<uint8_t> enc_r(width * height), enc_g(width * height), enc_b(width * height);
        
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int rgb_idx = y * rgb_stride + x * 3;
                int plane_idx = y * width + x;
                
                ref_r[plane_idx] = ref_rgb[rgb_idx];
                ref_g[plane_idx] = ref_rgb[rgb_idx + 1];
                ref_b[plane_idx] = ref_rgb[rgb_idx + 2];
                
                enc_r[plane_idx] = enc_rgb[rgb_idx];
                enc_g[plane_idx] = enc_rgb[rgb_idx + 1];
                enc_b[plane_idx] = enc_rgb[rgb_idx + 2];
            }
        }
        
        float score = butteraugli_compute_distance(butteraugli_ctx,
                                                   ref_r.data(), ref_g.data(), ref_b.data(), width,
                                                   enc_r.data(), enc_g.data(), enc_b.data(), width);
        
        scores.push_back(score);
        std::cout << frame_count << "," << score << std::endl;
        frame_count++;
    }
    
    if (!scores.empty()) {
        float sum = 0;
        for (float score : scores) sum += score;
        float average = sum / scores.size();
        
        std::cerr << "Processed " << frame_count << " frames" << std::endl;
        std::cerr << "Average Butteraugli score: " << average << std::endl;
    }
    
    butteraugli_destroy_context(butteraugli_ctx);
    return 0;
}