"""compare.py — C port vs Python golden vectors. Usage: python compare.py golden_dir out_c.raw hops_c.csv

Detection path (pitch, voicing, target, status) should agree ~100 %. The output waveform differs
sample-by-sample because the very first pitch mark is picked from near-silence, where float32 vs float64
rounding decides it; after that both versions follow the waveform from a different starting phase.
Listen to both or compare pitch tracks rather than expecting a high SNR."""
import sys
import numpy as np
gdir, craw, ccsv = sys.argv[1:4]
py = np.fromfile(f"{gdir}/output_q15.raw", dtype="<i2").astype(float) / 32768
c = np.fromfile(craw, dtype="<i2").astype(float) / 32768
hp = np.genfromtxt(f"{gdir}/hops.csv", delimiter=",", names=True)
hc = np.genfromtxt(ccsv, delimiter=",", names=True)
m = min(len(hp), len(hc)); hp, hc = hp[:m], hc[:m]
both = (hp["voiced"] == 1) & (hc["voiced"] == 1)
print(f"hops {m}: voiced agree {100*np.mean(hp['voiced']==hc['voiced']):.2f}%  "
      f"target agree {100*np.mean(hp['target']==hc['target']):.2f}%  status agree {100*np.mean(hp['status']==hc['status']):.2f}%")
if both.any():
    cents = 100 * np.abs(hp["m_s"][both] - hc["m_s"][both])
    print(f"  pitch |diff| on voiced hops: median {np.median(cents):.4f} c, 99th pct {np.percentile(cents,99):.3f} c, max {cents.max():.3f} c")
    print(f"  shift d_s |diff| max {np.max(np.abs(hp['d_s']-hc['d_s'])):.5f} st")
err = py - c
snr = 10 * np.log10(np.sum(py**2) / (np.sum(err**2) + 1e-20))
print(f"  output: SNR of C vs Python {snr:.1f} dB, max |diff| {np.max(np.abs(err)):.4f}, rms py {np.sqrt(np.mean(py**2)):.4f} c {np.sqrt(np.mean(c**2)):.4f}")
# where do they differ? 1 s blocks
blk = 16000
worst = []
for s in range(0, len(py) - blk, blk):
    p, q = py[s:s+blk], c[s:s+blk]
    if np.sum(p**2) > 1e-6:
        worst.append((10*np.log10(np.sum(p**2)/(np.sum((p-q)**2)+1e-20)), s/16000))
worst.sort()
print("  worst 1-s blocks (SNR dB, t s):", [(round(a,1), b) for a,b in worst[:4]])
