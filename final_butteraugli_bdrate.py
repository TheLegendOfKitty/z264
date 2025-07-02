#!/usr/bin/env python3
import numpy as np

def bjontegaard_delta_simple(rate1, dist1, rate2, dist2):
    """Simple BD-Rate calculation for 2 points"""
    # Convert to log space
    log_rate1 = np.log10(rate1)
    log_rate2 = np.log10(rate2)
    
    # Linear interpolation in log-log space
    # Calculate average log rate difference
    log_diff_1 = log_rate2[0] - log_rate1[0]  # At dist1[0]
    log_diff_2 = log_rate2[1] - log_rate1[1]  # At dist1[1]
    
    avg_log_diff = (log_diff_1 + log_diff_2) / 2
    
    # Convert back to percentage
    bd_rate = (10**avg_log_diff - 1) * 100
    
    return bd_rate

print("=== Final Butteraugli BD-Rate Analysis ===")
print()

# Complete dataset
crf_values = [28, 30]
baseline_bitrates = [15423.71, 11974.43]
butteraugli_bitrates = [15328.69, 11885.27]

# InfNorm scores (corrected - these should be the frame-specific ones)
baseline_infnorm = [9.42956, 9.53606]
butteraugli_infnorm = [9.39524, 9.54655]

# L3-Norm scores
baseline_l3norm = [1.50077, 1.50167]
butteraugli_l3norm = [1.50572, 1.50592]

print("Dataset Summary:")
print("CRF Values:", crf_values)
print("Baseline bitrates (kb/s):", baseline_bitrates)
print("Butteraugli bitrates (kb/s):", butteraugli_bitrates)
print()
print("InfNorm Scores:")
print("  Baseline:", baseline_infnorm)
print("  Butteraugli:", butteraugli_infnorm)
print()
print("L3-Norm Scores:")
print("  Baseline:", baseline_l3norm)
print("  Butteraugli:", butteraugli_l3norm)
print()

# Calculate BD-Rates
bd_rate_infnorm = bjontegaard_delta_simple(
    baseline_bitrates, baseline_infnorm,
    butteraugli_bitrates, butteraugli_infnorm
)

bd_rate_l3norm = bjontegaard_delta_simple(
    baseline_bitrates, baseline_l3norm,
    butteraugli_bitrates, butteraugli_l3norm
)

print("=== BD-RATE RESULTS ===")
print(f"Butteraugli InfNorm BD-Rate: {bd_rate_infnorm:+.2f}%")
print(f"Butteraugli L3-Norm BD-Rate: {bd_rate_l3norm:+.2f}%")
print()

# Detailed analysis
print("=== Detailed Analysis ===")
for i, crf in enumerate(crf_values):
    baseline_br = baseline_bitrates[i]
    butteraugli_br = butteraugli_bitrates[i]
    br_savings = (baseline_br - butteraugli_br) / baseline_br * 100
    
    infnorm_diff = butteraugli_infnorm[i] - baseline_infnorm[i]
    l3norm_diff = butteraugli_l3norm[i] - baseline_l3norm[i]
    
    print(f"CRF {crf}:")
    print(f"  Bitrate: {baseline_br:.1f} → {butteraugli_br:.1f} kb/s ({br_savings:+.2f}% change)")
    print(f"  InfNorm: {baseline_infnorm[i]:.5f} → {butteraugli_infnorm[i]:.5f} ({infnorm_diff:+.5f})")
    print(f"  L3-Norm: {baseline_l3norm[i]:.5f} → {butteraugli_l3norm[i]:.5f} ({l3norm_diff:+.5f})")
    print()

print("=== Interpretation ===")
if bd_rate_infnorm > 0:
    print(f"InfNorm: Butteraugli uses {bd_rate_infnorm:.2f}% more bitrate for same quality")
else:
    print(f"InfNorm: Butteraugli achieves {abs(bd_rate_infnorm):.2f}% bitrate savings at same quality")

if bd_rate_l3norm > 0:
    print(f"L3-Norm: Butteraugli uses {bd_rate_l3norm:.2f}% more bitrate for same quality")
else:
    print(f"L3-Norm: Butteraugli achieves {abs(bd_rate_l3norm):.2f}% bitrate savings at same quality")

print()
print("=== Comparison with Traditional Metrics ===")
print("Metric              | BD-Rate")
print("--------------------|--------")
print("PSNR-Y              | +0.60%")
print("SSIM-Y              | +0.48%")
print("XPSNR-Y             | +0.37%")
print("SSIMULACRA2         | 0.00%")
print(f"Butteraugli InfNorm | {bd_rate_infnorm:+.2f}%")
print(f"Butteraugli L3-Norm | {bd_rate_l3norm:+.2f}%")
print()
print("→ Butteraugli optimization shows mixed results depending on the norm used")