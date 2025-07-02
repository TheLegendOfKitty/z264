#\!/bin/bash

echo "QP,Type,Bitrate,PSNR_Y,PSNR_U,PSNR_V,PSNR_Avg,SSIM_Y,SSIM_U,SSIM_V,SSIM_All,XPSNR_Y,XPSNR_U,XPSNR_V" > rd_metrics.csv

for qp in 18 22 26 30; do
    for type in baseline butteraugli; do
        echo "Processing ${type}_qp${qp}..."
        
        # Get bitrate from log
        bitrate=$(grep "kb/s:" ${type}_qp${qp}.log | awk -F: '{print $2}')
        
        # Get PSNR
        psnr_output=$(ffmpeg -i ${type}_qp${qp}.264 -i food8.y4m -lavfi psnr -f null - 2>&1 | grep "PSNR" | tail -1)
        psnr_y=$(echo $psnr_output | grep -oP 'y:\K[0-9.]+')
        psnr_u=$(echo $psnr_output | grep -oP 'u:\K[0-9.]+')
        psnr_v=$(echo $psnr_output | grep -oP 'v:\K[0-9.]+')
        psnr_avg=$(echo $psnr_output | grep -oP 'average:\K[0-9.]+')
        
        # Get SSIM
        ssim_output=$(ffmpeg -i ${type}_qp${qp}.264 -i food8.y4m -lavfi ssim -f null - 2>&1 | grep "SSIM" | tail -1)
        ssim_y=$(echo $ssim_output | grep -oP 'Y:\K[0-9.]+')
        ssim_u=$(echo $ssim_output | grep -oP 'U:\K[0-9.]+')
        ssim_v=$(echo $ssim_output | grep -oP 'V:\K[0-9.]+')
        ssim_all=$(echo $ssim_output | grep -oP 'All:\K[0-9.]+')
        
        # Get XPSNR
        xpsnr_output=$(ffmpeg -i ${type}_qp${qp}.264 -i food8.y4m -lavfi xpsnr -f null - 2>&1 | grep "XPSNR" | tail -1)
        xpsnr_y=$(echo $xpsnr_output | grep -oP 'y:\s*\K[0-9.]+')
        xpsnr_u=$(echo $xpsnr_output | grep -oP 'u:\s*\K[0-9.]+')
        xpsnr_v=$(echo $xpsnr_output | grep -oP 'v:\s*\K[0-9.]+')
        
        echo "$qp,$type,$bitrate,$psnr_y,$psnr_u,$psnr_v,$psnr_avg,$ssim_y,$ssim_u,$ssim_v,$ssim_all,$xpsnr_y,$xpsnr_u,$xpsnr_v" >> rd_metrics.csv
    done
done

echo "Metrics saved to rd_metrics.csv"
cat rd_metrics.csv
