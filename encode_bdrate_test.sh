#\!/bin/bash

# Create RD curves with multiple QP points
QP_VALUES=(18 22 26 30)

echo "Encoding baseline versions..."
for qp in "${QP_VALUES[@]}"; do
    echo "Encoding baseline QP=$qp"
    ./x264 --qp $qp -o baseline_qp${qp}.264 food8.y4m 2>&1 | tail -5 > baseline_qp${qp}.log
done

echo "Encoding butteraugli versions..."
for qp in "${QP_VALUES[@]}"; do
    echo "Encoding butteraugli QP=$qp"
    ./x264 --qp $qp --butteraugli --butteraugli-strength 0.5 -o butteraugli_qp${qp}.264 food8.y4m 2>&1 | tail -5 > butteraugli_qp${qp}.log
done
