#\!/bin/bash

echo "QP,Type,Bitrate,PSNR_Y,PSNR_U,PSNR_V,PSNR_Avg,SSIM_Y,SSIM_U,SSIM_V,SSIM_All,XPSNR_Y,XPSNR_U,XPSNR_V" > rd_metrics.csv

for qp in 18 22 26 30; do
    for type in baseline butteraugli; do
        echo "Processing ${type}_qp${qp}..."
        
        # Get bitrate from log
        bitrate=$(grep "kb/s:" ${type}_qp${qp}.log | awk -F: '{print $2}' | sed 's/ kb\/s//')
        
        # Get PSNR
        psnr_output=$(ffmpeg -i ${type}_qp${qp}.264 -i food8.y4m -lavfi psnr -f null - 2>&1 | grep "PSNR" | tail -1)
        psnr_y=$(echo $psnr_output | sed -n 's/.*y:\([0-9.]*\).*/\1/p')
        psnr_u=$(echo $psnr_output | sed -n 's/.*u:\([0-9.]*\).*/\1/p')
        psnr_v=$(echo $psnr_output | sed -n 's/.*v:\([0-9.]*\).*/\1/p')
        psnr_avg=$(echo $psnr_output | sed -n 's/.*average:\([0-9.]*\).*/\1/p')
        
        # Get SSIM
        ssim_output=$(ffmpeg -i ${type}_qp${qp}.264 -i food8.y4m -lavfi ssim -f null - 2>&1 | grep "SSIM" | tail -1)
        ssim_y=$(echo $ssim_output | sed -n 's/.*Y:\([0-9.]*\).*/\1/p')
        ssim_u=$(echo $ssim_output | sed -n 's/.*U:\([0-9.]*\).*/\1/p')
        ssim_v=$(echo $ssim_output | sed -n 's/.*V:\([0-9.]*\).*/\1/p')
        ssim_all=$(echo $ssim_output | sed -n 's/.*All:\([0-9.]*\).*/\1/p')
        
        # Get XPSNR
        xpsnr_output=$(ffmpeg -i ${type}_qp${qp}.264 -i food8.y4m -lavfi xpsnr -f null - 2>&1 | grep "XPSNR" | tail -1)
        xpsnr_y=$(echo $xpsnr_output | awk '{print $3}')
        xpsnr_u=$(echo $xpsnr_output | awk '{print $5}')
        xpsnr_v=$(echo $xpsnr_output | awk '{print $7}')
        
        echo "$qp,$type,$bitrate,$psnr_y,$psnr_u,$psnr_v,$psnr_avg,$ssim_y,$ssim_u,$ssim_v,$ssim_all,$xpsnr_y,$xpsnr_u,$xpsnr_v" >> rd_metrics.csv
    done
done

echo "Metrics saved to rd_metrics.csv"
cat rd_metrics.csv
