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
    """Calculate BD-Rate"""
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

print("=== Butteraugli InfNorm and L3-Norm BD-Rate Calculation ===")
print()

# Manual data from our tests
print("Collected Data Points:")
crf_values = [26, 28, 30, 32]

# Bitrates (kb/s)
baseline_bitrates = [20460.16, 15423.71, 11974.43, 9961.80]
butteraugli_bitrates = [20322.22, 15328.69, 11885.27, 9896.20]

print("CRF Values:", crf_values)
print("Baseline bitrates:", baseline_bitrates)
print("Butteraugli bitrates:", butteraugli_bitrates)

# We have butteraugli scores for CRF 28 and 30, need to estimate others
print()
print("Known Butteraugli InfNorm Scores:")
print("CRF 28: Baseline=9.42956, Butteraugli=9.39524")
print("CRF 30: Baseline=1.87217, Butteraugli=1.96445")

# For BD-rate calculation, we need all points. Let's calculate with the known data
print()
print("BD-Rate calculation with available data (CRF 28, 30):")

# Use only CRF 28 and 30 for BD-rate calculation
baseline_rates_subset = [15423.71, 11974.43]
butteraugli_rates_subset = [15328.69, 11885.27]
baseline_infnorm_subset = [9.42956, 1.87217]
butteraugli_infnorm_subset = [9.39524, 1.96445]

print("BD-Rate calculation:")
print("Baseline: Rates =", baseline_rates_subset, "InfNorm =", baseline_infnorm_subset)
print("Butteraugli: Rates =", butteraugli_rates_subset, "InfNorm =", butteraugli_infnorm_subset)

# Calculate BD-Rate for InfNorm
bd_rate_infnorm = bjontegaard_delta(baseline_rates_subset, baseline_infnorm_subset,
                                   butteraugli_rates_subset, butteraugli_infnorm_subset)

print()
print("=== RESULTS ===")
print(f"Butteraugli InfNorm BD-Rate: {bd_rate_infnorm:.2f}%")

# Calculate bitrate savings
print()
print("Bitrate savings per CRF:")
for i, crf in enumerate([28, 30]):
    baseline_br = baseline_rates_subset[i]
    butteraugli_br = butteraugli_rates_subset[i]
    savings = (baseline_br - butteraugli_br) / baseline_br * 100
    baseline_score = baseline_infnorm_subset[i]
    butteraugli_score = butteraugli_infnorm_subset[i]
    score_diff = butteraugli_score - baseline_score
    print(f"CRF {crf}: {savings:.2f}% bitrate savings, InfNorm change: {score_diff:+.5f}")

print()
print("Interpretation:")
if bd_rate_infnorm < 0:
    print(f"Butteraugli optimization achieves {abs(bd_rate_infnorm):.2f}% better quality at same bitrate")
else:
    print(f"Butteraugli optimization uses {bd_rate_infnorm:.2f}% more bitrate for same quality")

# Note about L3-norm
print()
print("Note: L3-norm would require calculating the cube root of sum of cubed differences")
print("from the butteraugli diffmap, which requires access to the full diffmap data.")
print("The InfNorm (maximum) is readily available from our current implementation.")