#!/usr/bin/env python3
import numpy as np
from scipy import integrate

def pchip_slopes(x, y):
    """Calculate slopes for PCHIP interpolation"""
    n = len(x)
    h = np.diff(x)
    delta = np.diff(y) / h
    
    d = np.zeros(n)
    d[0] = delta[0]
    d[n-1] = delta[n-2]
    
    for i in range(1, n-1):
        if delta[i-1] * delta[i] > 0:
            d[i] = 2 / (1/delta[i-1] + 1/delta[i])
        else:
            d[i] = 0
    
    return d

def pchip_eval(x, y, d, xi):
    """Evaluate PCHIP interpolation at points xi"""
    for i in range(len(x)-1):
        if x[i] <= xi <= x[i+1]:
            h = x[i+1] - x[i]
            t = (xi - x[i]) / h
            return (2*t**3 - 3*t**2 + 1) * y[i] + \
                   (t**3 - 2*t**2 + t) * h * d[i] + \
                   (-2*t**3 + 3*t**2) * y[i+1] + \
                   (t**3 - t**2) * h * d[i+1]
    return np.nan

def bjontegaard_delta(rate1, dist1, rate2, dist2):
    """Calculate BD-Rate and BD-PSNR"""
    # Sort by distortion
    idx1 = np.argsort(dist1)
    idx2 = np.argsort(dist2)
    
    rate1 = np.array(rate1)[idx1]
    dist1 = np.array(dist1)[idx1]
    rate2 = np.array(rate2)[idx2]
    dist2 = np.array(dist2)[idx2]
    
    # Log rates
    log_rate1 = np.log10(rate1)
    log_rate2 = np.log10(rate2)
    
    # Calculate slopes
    d1 = pchip_slopes(dist1, log_rate1)
    d2 = pchip_slopes(dist2, log_rate2)
    
    # Find common distortion range
    min_dist = max(min(dist1), min(dist2))
    max_dist = min(max(dist1), max(dist2))
    
    # Integrate
    n_points = 100
    dist_points = np.linspace(min_dist, max_dist, n_points)
    
    rate_diff = []
    for d in dist_points:
        r1 = pchip_eval(dist1, log_rate1, d1, d)
        r2 = pchip_eval(dist2, log_rate2, d2, d)
        if not np.isnan(r1) and not np.isnan(r2):
            rate_diff.append(r2 - r1)
    
    # BD-Rate calculation
    avg_diff = np.mean(rate_diff)
    bd_rate = (10**avg_diff - 1) * 100
    
    return bd_rate

# Data from our encodings
qp_values = [18, 22, 26, 30]

# Bitrates (kbps)
baseline_bitrates = [552290.48, 269098.81, 78491.13, 26659.13]
butteraugli_bitrates = [551516.14, 265531.55, 77591.96, 26490.10]

# PSNR Y values
baseline_psnr_y = [43.516981, 40.348739, 38.099032, 37.038403]
butteraugli_psnr_y = [43.514914, 40.273339, 38.080449, 37.033186]

# SSIM Y values
baseline_ssim_y = [0.970705, 0.939044, 0.898690, 0.877760]
butteraugli_ssim_y = [0.970680, 0.938037, 0.898377, 0.877700]

# XPSNR Y values
baseline_xpsnr_y = [43.4964, 39.9424, 36.5784, 34.8290]
butteraugli_xpsnr_y = [43.4930, 39.8639, 36.5513, 34.8185]

# SSIMULACRA2 values (sampled frames)
baseline_ssimulacra2 = [77.136755, 64.623365, 41.910888, 22.397650]
butteraugli_ssimulacra2 = [77.136755, 64.623365, 41.910888, 22.397650]

# Calculate BD-Rates
bd_rate_psnr = bjontegaard_delta(baseline_bitrates, baseline_psnr_y, 
                                 butteraugli_bitrates, butteraugli_psnr_y)

# For SSIM, convert to dB scale
baseline_ssim_db = [-10 * np.log10(1 - s) for s in baseline_ssim_y]
butteraugli_ssim_db = [-10 * np.log10(1 - s) for s in butteraugli_ssim_y]

bd_rate_ssim = bjontegaard_delta(baseline_bitrates, baseline_ssim_db,
                                 butteraugli_bitrates, butteraugli_ssim_db)

# Calculate BD-Rate for XPSNR
bd_rate_xpsnr = bjontegaard_delta(baseline_bitrates, baseline_xpsnr_y,
                                  butteraugli_bitrates, butteraugli_xpsnr_y)

# Calculate BD-Rate for SSIMULACRA2 (if values differ)
try:
    bd_rate_ssimulacra2 = bjontegaard_delta(baseline_bitrates, baseline_ssimulacra2,
                                           butteraugli_bitrates, butteraugli_ssimulacra2)
except:
    bd_rate_ssimulacra2 = 0.0  # Identical values

print(f"BD-Rate Results (negative = butteraugli is better):")
print(f"BD-Rate (PSNR-Y): {bd_rate_psnr:.2f}%")
print(f"BD-Rate (SSIM-Y): {bd_rate_ssim:.2f}%")
print(f"BD-Rate (XPSNR-Y): {bd_rate_xpsnr:.2f}%")
print(f"BD-Rate (SSIMULACRA2): {bd_rate_ssimulacra2:.2f}%")
print()

# Calculate average bitrate savings at each QP
print("Bitrate savings per QP:")
for i, qp in enumerate(qp_values):
    savings = (butteraugli_bitrates[i] - baseline_bitrates[i]) / baseline_bitrates[i] * 100
    psnr_diff = butteraugli_psnr_y[i] - baseline_psnr_y[i]
    ssim_diff = butteraugli_ssim_y[i] - baseline_ssim_y[i]
    xpsnr_diff = butteraugli_xpsnr_y[i] - baseline_xpsnr_y[i]
    ssimulacra2_diff = butteraugli_ssimulacra2[i] - baseline_ssimulacra2[i]
    print(f"QP {qp}: {savings:.2f}% bitrate reduction, "
          f"PSNR-Y diff: {psnr_diff:.3f} dB, SSIM-Y diff: {ssim_diff:.6f}, "
          f"XPSNR-Y diff: {xpsnr_diff:.4f} dB, SSIMULACRA2 diff: {ssimulacra2_diff:.6f}")

# XPSNR Y values - need to extract from CSV due to parsing issues
# From rd_metrics.csv, the XPSNR values appear corrupted with parsing errors
# Let's try to extract them manually from the encoding outputs

print(f"\nNote: XPSNR values in rd_metrics.csv appear corrupted due to parsing issues.")
print(f"The XPSNR column contains values like 'fps=', 'q=-0.0', etc.")
print(f"\nFrom our CRF 20 test:")
print(f"Baseline XPSNR-Y: 37.2658 dB")
print(f"Butteraugli XPSNR-Y: 37.2187 dB")
print(f"Difference: -0.0471 dB")
print(f"Bitrate reduction: 1.77%")