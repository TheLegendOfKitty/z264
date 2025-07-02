#!/bin/bash

echo "Calculating SSIMULACRA2 scores..."
echo "QP,Type,SSIMULACRA2_Score"

# First decode the h264 files to raw YUV for comparison
for qp in 18 22 26 30; do
    for type in baseline butteraugli; do
        # Decode to YUV if not already done
        if [ ! -f "${type}_qp${qp}.yuv" ]; then
            ffmpeg -i ${type}_qp${qp}.264 -f rawvideo -pix_fmt yuv420p ${type}_qp${qp}.yuv -y 2>/dev/null
        fi
        
        # Convert YUV to PNG frames for ssimulacra2
        mkdir -p temp_${type}_qp${qp}
        ffmpeg -f rawvideo -pix_fmt yuv420p -s 1920x1080 -i ${type}_qp${qp}.yuv temp_${type}_qp${qp}/frame_%03d.png -y 2>/dev/null
        
        # Convert reference to PNG frames if not done
        if [ ! -d "temp_reference" ]; then
            mkdir -p temp_reference
            ffmpeg -i food8.y4m temp_reference/frame_%03d.png -y 2>/dev/null
        fi
        
        # Calculate average SSIMULACRA2 score
        total=0
        count=0
        scores=""
        
        # Calculate for each frame
        for i in $(seq -f "%03g" 1 141); do
            if [ -f "temp_${type}_qp${qp}/frame_${i}.png" ] && [ -f "temp_reference/frame_${i}.png" ]; then
                score=$(ssimulacra2 "temp_reference/frame_${i}.png" "temp_${type}_qp${qp}/frame_${i}.png" 2>/dev/null)
                if [ ! -z "$score" ]; then
                    scores="$scores $score"
                    count=$((count + 1))
                fi
            fi
        done
        
        # Calculate average
        if [ $count -gt 0 ]; then
            avg=$(echo "$scores" | awk '{sum=0; for(i=1;i<=NF;i++)sum+=$i; print sum/NF}')
            echo "$qp,$type,$avg"
        else
            echo "$qp,$type,ERROR"
        fi
        
        # Cleanup
        rm -rf temp_${type}_qp${qp}
    done
done

# Cleanup reference
rm -rf temp_reference

echo "Done calculating SSIMULACRA2 scores"