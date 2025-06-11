#!/bin/bash

# Test script for parallel lookahead feature

echo "Testing x264 parallel lookahead performance..."

# Test video file (adjust path as needed)
INPUT="${1:-test.y4m}"
OUTPUT_BASE="test_parallel_lookahead"

if [ ! -f "$INPUT" ]; then
    echo "Error: Input file '$INPUT' not found"
    echo "Usage: $0 [input_file]"
    exit 1
fi

# Build x264 if needed
if [ ! -f ./x264 ]; then
    echo "Building x264..."
    make -j$(nproc) || exit 1
fi

# Common encoding parameters
COMMON_PARAMS="--preset medium --crf 23 --threads 4 --lookahead 40"

echo "============================================"
echo "1. Baseline (no parallel lookahead)"
echo "============================================"
time ./x264 $COMMON_PARAMS -o ${OUTPUT_BASE}_baseline.264 "$INPUT" 2>&1 | tee baseline.log
BASELINE_FPS=$(grep "encoded" baseline.log | grep -oP '\d+\.\d+(?= fps)')

echo -e "\n============================================"
echo "2. Parallel lookahead (auto workers)"
echo "============================================"
time ./x264 $COMMON_PARAMS --parallel-lookahead -o ${OUTPUT_BASE}_parallel_auto.264 "$INPUT" 2>&1 | tee parallel_auto.log
PARALLEL_AUTO_FPS=$(grep "encoded" parallel_auto.log | grep -oP '\d+\.\d+(?= fps)')

echo -e "\n============================================"
echo "3. Parallel lookahead (2 workers)"
echo "============================================"
time ./x264 $COMMON_PARAMS --parallel-lookahead --lookahead-workers 2 -o ${OUTPUT_BASE}_parallel_2.264 "$INPUT" 2>&1 | tee parallel_2.log
PARALLEL_2_FPS=$(grep "encoded" parallel_2.log | grep -oP '\d+\.\d+(?= fps)')

echo -e "\n============================================"
echo "4. Parallel lookahead (4 workers)"
echo "============================================"
time ./x264 $COMMON_PARAMS --parallel-lookahead --lookahead-workers 4 -o ${OUTPUT_BASE}_parallel_4.264 "$INPUT" 2>&1 | tee parallel_4.log
PARALLEL_4_FPS=$(grep "encoded" parallel_4.log | grep -oP '\d+\.\d+(?= fps)')

# Calculate improvements
if [ -n "$BASELINE_FPS" ] && [ -n "$PARALLEL_AUTO_FPS" ]; then
    IMPROVEMENT_AUTO=$(echo "scale=2; ($PARALLEL_AUTO_FPS - $BASELINE_FPS) / $BASELINE_FPS * 100" | bc)
    echo -e "\n============================================"
    echo "Performance Summary:"
    echo "============================================"
    echo "Baseline:                 $BASELINE_FPS fps"
    echo "Parallel (auto):          $PARALLEL_AUTO_FPS fps (${IMPROVEMENT_AUTO}% improvement)"
    
    if [ -n "$PARALLEL_2_FPS" ]; then
        IMPROVEMENT_2=$(echo "scale=2; ($PARALLEL_2_FPS - $BASELINE_FPS) / $BASELINE_FPS * 100" | bc)
        echo "Parallel (2 workers):     $PARALLEL_2_FPS fps (${IMPROVEMENT_2}% improvement)"
    fi
    
    if [ -n "$PARALLEL_4_FPS" ]; then
        IMPROVEMENT_4=$(echo "scale=2; ($PARALLEL_4_FPS - $BASELINE_FPS) / $BASELINE_FPS * 100" | bc)
        echo "Parallel (4 workers):     $PARALLEL_4_FPS fps (${IMPROVEMENT_4}% improvement)"
    fi
fi

# Verify output files are identical
echo -e "\n============================================"
echo "Verifying output consistency..."
echo "============================================"
for output in ${OUTPUT_BASE}_parallel_*.264; do
    if [ -f "$output" ]; then
        diff -q ${OUTPUT_BASE}_baseline.264 "$output" || echo "WARNING: $output differs from baseline!"
    fi
done

# Cleanup
rm -f *.log ${OUTPUT_BASE}_*.264

echo -e "\nTest complete!"