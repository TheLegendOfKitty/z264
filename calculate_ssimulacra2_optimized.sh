#!/bin/bash

echo "Calculating SSIMULACRA2 scores..."
echo "QP,Type,SSIMULACRA2_Score" > ssimulacra2_results.csv

# Convert reference to PNG frames first
if [ ! -d "temp_reference" ]; then
    echo "Converting reference video to PNG frames..."
    mkdir -p temp_reference
    ffmpeg -i food8.y4m temp_reference/frame_%03d.png -y 2>/dev/null
fi

# Process each encoding
for qp in 18 22 26 30; do
    for type in baseline butteraugli; do
        echo "Processing ${type}_qp${qp}..."
        
        # Convert to PNG frames
        mkdir -p temp_${type}_qp${qp}
        ffmpeg -i ${type}_qp${qp}.264 temp_${type}_qp${qp}/frame_%03d.png -y 2>/dev/null
        
        # Create a temporary file for scores
        scores_file="scores_${type}_qp${qp}.txt"
        > $scores_file
        
        # Calculate SSIMULACRA2 for all frames in parallel
        for i in $(seq -f "%03g" 1 141); do
            if [ -f "temp_${type}_qp${qp}/frame_${i}.png" ] && [ -f "temp_reference/frame_${i}.png" ]; then
                (ssimulacra2 "temp_reference/frame_${i}.png" "temp_${type}_qp${qp}/frame_${i}.png" 2>/dev/null >> $scores_file) &
                
                # Limit parallel jobs to avoid overwhelming the system
                if [ $(jobs -r | wc -l) -ge 8 ]; then
                    wait -n
                fi
            fi
        done
        
        # Wait for all remaining jobs
        wait
        
        # Calculate average
        if [ -s $scores_file ]; then
            avg=$(awk '{sum+=$1; count++} END {if(count>0) print sum/count; else print "ERROR"}' $scores_file)
            echo "$qp,$type,$avg" >> ssimulacra2_results.csv
            echo "QP $qp $type: Average SSIMULACRA2 = $avg"
        else
            echo "$qp,$type,ERROR" >> ssimulacra2_results.csv
            echo "QP $qp $type: ERROR"
        fi
        
        # Cleanup
        rm -f $scores_file
        rm -rf temp_${type}_qp${qp}
    done
done

# Cleanup reference
rm -rf temp_reference

echo "Done! Results saved to ssimulacra2_results.csv"
cat ssimulacra2_results.csv