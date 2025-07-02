#!/bin/bash

echo "Extracting XPSNR values..."
echo "QP,Type,XPSNR_Y,XPSNR_U,XPSNR_V"

for qp in 18 22 26 30; do
    for type in baseline butteraugli; do
        echo -n "$qp,$type,"
        # Run ffmpeg and capture the last XPSNR line
        result=$(ffmpeg -i ${type}_qp${qp}.264 -i food8.y4m -lavfi xpsnr -f null - 2>&1 | grep "XPSNR" | tail -1)
        # Extract Y, U, V values
        y_val=$(echo "$result" | sed -n 's/.*y: \([0-9.]*\).*/\1/p')
        u_val=$(echo "$result" | sed -n 's/.*u: \([0-9.]*\).*/\1/p')
        v_val=$(echo "$result" | sed -n 's/.*v: \([0-9.]*\).*/\1/p')
        echo "$y_val,$u_val,$v_val"
    done
done