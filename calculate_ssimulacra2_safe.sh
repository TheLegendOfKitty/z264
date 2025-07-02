#!/bin/bash

echo "Calculating SSIMULACRA2 scores (safe mode - sampling frames)..."

# Sample frames: 10, 50, 90, 130 (4 frames across the video)
sample_frames=(010 050 090 130)

# Convert reference frames
echo "Converting reference frames..."
mkdir -p ref_frames
for frame in "${sample_frames[@]}"; do
    ffmpeg -i food8.y4m -vf "select=eq(n\,$(echo $frame | sed 's/^0*//'))" -vframes 1 ref_frames/frame_${frame}.png -y 2>/dev/null
done

echo "QP,Type,SSIMULACRA2_Score" > ssimulacra2_results.csv

for qp in 18 22 26 30; do
    for type in baseline butteraugli; do
        echo "Processing ${type}_qp${qp}..."
        
        # Convert sample frames from encoded video
        mkdir -p ${type}_qp${qp}_frames
        for frame in "${sample_frames[@]}"; do
            ffmpeg -i ${type}_qp${qp}.264 -vf "select=eq(n\,$(echo $frame | sed 's/^0*//'))" -vframes 1 ${type}_qp${qp}_frames/frame_${frame}.png -y 2>/dev/null
        done
        
        # Calculate SSIMULACRA2 for sample frames
        total_score=0
        count=0
        
        for frame in "${sample_frames[@]}"; do
            if [ -f "ref_frames/frame_${frame}.png" ] && [ -f "${type}_qp${qp}_frames/frame_${frame}.png" ]; then
                score=$(ssimulacra2 "ref_frames/frame_${frame}.png" "${type}_qp${qp}_frames/frame_${frame}.png" 2>/dev/null)
                if [ ! -z "$score" ] && [ "$score" != "ERROR" ]; then
                    total_score=$(echo "$total_score + $score" | bc -l)
                    count=$((count + 1))
                    echo "  Frame $frame: $score"
                fi
            fi
        done
        
        # Calculate average
        if [ $count -gt 0 ]; then
            avg=$(echo "scale=6; $total_score / $count" | bc -l)
            echo "$qp,$type,$avg" >> ssimulacra2_results.csv
            echo "  Average: $avg"
        else
            echo "$qp,$type,ERROR" >> ssimulacra2_results.csv
        fi
        
        # Cleanup
        rm -rf ${type}_qp${qp}_frames
    done
done

# Cleanup
rm -rf ref_frames

echo "Done! Results (sampled frames):"
cat ssimulacra2_results.csv