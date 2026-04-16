#!/usr/bin/env python3
"""
analyze.py - Post-hoc HRV validation analysis for HealthMonitor sessions.

Compares SCG-derived HRV metrics against ECG (gold standard) using
Bland-Altman analysis, Pearson correlation, and ICC(2,1).

Usage:
    python analyze.py data/session_20260410_012050

Dependencies:
    pip install numpy matplotlib scipy
"""

import sys
import os
import csv
import numpy as np
import matplotlib.pyplot as plt

try:
    from scipy import stats as sp_stats
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# -- Constants ----------------------------------------------------------------
SETTLE_S = 90.0          # Discard first 90 s (threshold settling)
MAX_PAIR_DT = 20.0       # Max seconds between paired ECG/SCG HRV reports

METRICS = [
    ("rmssd_ms", "RMSSD", "ms"),
    ("sdnn_ms",  "SDNN",  "ms"),
    ("pnn50_pct","pNN50", "%"),
    ("sd1_ms",   "SD1",   "ms"),
    ("sd2_ms",   "SD2",   "ms"),
]

# -- Plot style ---------------------------------------------------------------
plt.rcParams.update({
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 13,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 10,
    "figure.dpi": 150,
    "savefig.dpi": 300,
    "savefig.bbox": "tight",
    "font.family": "sans-serif",
})

C_BA = "#2c7bb6"     # Bland-Altman data points
C_CORR = "#d7191c"   # Correlation data points
C_BIAS = "#d73027"   # Bias line
C_LOA = "#999999"    # Limits of agreement


# -- Data loading -------------------------------------------------------------
def load_csv(filepath):
    """Load a CSV file, return list of dicts. Returns [] if missing/empty."""
    if not os.path.isfile(filepath):
        print(f"  Warning: {os.path.basename(filepath)} not found, skipping.")
        return []
    with open(filepath, "r") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print(f"  Warning: {os.path.basename(filepath)} is empty.")
    return rows


def resolve_session_path(arg):
    """Resolve session directory, trying both absolute and script-relative."""
    path = os.path.abspath(arg)
    if os.path.isdir(path):
        return path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(script_dir, arg)
    if os.path.isdir(path):
        return path
    sys.exit(f"Error: session directory not found: {arg}")


# -- Pairing ------------------------------------------------------------------
def pair_hrv_reports(ecg_rows, scg_rows):
    """Pair ECG and SCG HRV reports by nearest timestamp.

    Filters to t > SETTLE_S, pairs greedily, no double-pairing,
    rejects pairs with time gap > MAX_PAIR_DT.
    """
    ecg = [r for r in ecg_rows if float(r["elapsed_s"]) > SETTLE_S]
    scg = [r for r in scg_rows if float(r["elapsed_s"]) > SETTLE_S]

    pairs = []
    used = set()

    for e in ecg:
        et = float(e["elapsed_s"])
        best_j, best_dt = None, float("inf")
        for j, s in enumerate(scg):
            if j in used:
                continue
            dt = abs(et - float(s["elapsed_s"]))
            if dt < best_dt:
                best_dt = dt
                best_j = j
        if best_j is not None and best_dt <= MAX_PAIR_DT:
            pairs.append((e, scg[best_j]))
            used.add(best_j)

    return pairs


def filter_outliers(pairs):
    """Exclude pairs with extreme RMSSD differences (template-failure outliers).

    Uses 3xIQR on SCG-ECG RMSSD difference. Conservative enough to only
    remove template-failure artifacts, not genuine physiological variation.
    Returns (clean_pairs, outlier_pairs).
    """
    if len(pairs) < 8:
        return pairs, []

    diffs = [float(s["rmssd_ms"]) - float(e["rmssd_ms"]) for e, s in pairs]
    diffs_sorted = sorted(diffs)
    n = len(diffs_sorted)
    q1 = diffs_sorted[n // 4]
    q3 = diffs_sorted[3 * n // 4]
    iqr = q3 - q1
    lower = q1 - 3.0 * iqr
    upper = q3 + 3.0 * iqr

    clean, outliers = [], []
    for (e, s), d in zip(pairs, diffs):
        if lower <= d <= upper:
            clean.append((e, s))
        else:
            outliers.append((e, s))

    return clean, outliers


# -- Statistics ---------------------------------------------------------------
def pearsonr(x, y):
    """Pearson correlation coefficient. Returns (r, p) or (r, nan)."""
    n = len(x)
    if n < 3:
        return float("nan"), float("nan")
    xm = x - x.mean()
    ym = y - y.mean()
    denom = np.sqrt(np.sum(xm ** 2) * np.sum(ym ** 2))
    if denom < 1e-12:
        return 0.0, float("nan")
    r = float(np.sum(xm * ym) / denom)
    r = max(-1.0, min(1.0, r))

    if HAS_SCIPY and abs(r) < 1.0:
        t_stat = r * np.sqrt((n - 2) / (1 - r * r))
        p = float(2 * sp_stats.t.sf(abs(t_stat), n - 2))
    else:
        p = float("nan")
    return r, p


def compute_icc(x, y):
    """ICC(2,1) - two-way random, single measures, absolute agreement."""
    n = len(x)
    if n < 3:
        return float("nan")

    data = np.column_stack([x, y])
    k = 2
    grand_mean = data.mean()
    row_means = data.mean(axis=1)
    col_means = data.mean(axis=0)

    ss_total = float(np.sum((data - grand_mean) ** 2))
    ss_rows = float(k * np.sum((row_means - grand_mean) ** 2))
    ss_cols = float(n * np.sum((col_means - grand_mean) ** 2))
    ss_error = ss_total - ss_rows - ss_cols

    ms_rows = ss_rows / (n - 1)
    ms_cols = ss_cols / (k - 1)
    ms_error = ss_error / ((n - 1) * (k - 1))

    denom = ms_rows + (k - 1) * ms_error + k * (ms_cols - ms_error) / n
    if abs(denom) < 1e-12:
        return float("nan")
    return float((ms_rows - ms_error) / denom)


# -- Bland-Altman -------------------------------------------------------------
def plot_bland_altman(ecg_arrs, scg_arrs, session_dir,
                     outlier_ecg=None, outlier_scg=None):
    """Generate 5-panel Bland-Altman figure, save to session_dir."""
    fig, axes = plt.subplots(2, 3, figsize=(14, 9))
    axes_flat = axes.flatten()

    for idx, (key, name, unit) in enumerate(METRICS):
        ax = axes_flat[idx]
        ecg = ecg_arrs[key]
        scg = scg_arrs[key]
        diff = scg - ecg
        mean_both = (ecg + scg) / 2.0

        bias = np.mean(diff)
        sd = np.std(diff, ddof=1) if len(diff) > 1 else 0.0
        loa_hi = bias + 1.96 * sd
        loa_lo = bias - 1.96 * sd

        ax.scatter(mean_both, diff, c=C_BA, s=50, alpha=0.75, edgecolors="k",
                   linewidths=0.3, zorder=5)

        # Mark outliers in red
        if outlier_ecg and key in outlier_ecg and len(outlier_ecg[key]) > 0:
            o_ecg = outlier_ecg[key]
            o_scg = outlier_scg[key]
            o_diff = o_scg - o_ecg
            o_mean = (o_ecg + o_scg) / 2.0
            ax.scatter(o_mean, o_diff, c="#d73027", s=60, alpha=0.9,
                       linewidths=1.5, zorder=6, marker="x",
                       label=f"Outliers ({len(o_ecg)})")
        ax.axhline(bias, color=C_BIAS, linestyle="--", linewidth=1.2,
                   label=f"Bias: {bias:+.1f}")
        ax.axhline(loa_hi, color=C_LOA, linestyle="--", linewidth=0.9)
        ax.axhline(loa_lo, color=C_LOA, linestyle="--", linewidth=0.9)
        ax.fill_between(ax.get_xlim() if ax.get_xlim()[0] != ax.get_xlim()[1]
                        else [0, 1],
                        loa_lo, loa_hi, color=C_LOA, alpha=0.10, zorder=0)

        # Redraw fill after data-driven xlim
        xlo, xhi = ax.get_xlim()
        ax.fill_between([xlo, xhi], loa_lo, loa_hi, color=C_LOA, alpha=0.10,
                        zorder=0)

        ax.set_xlabel(f"Mean ({unit})")
        ax.set_ylabel(f"SCG - ECG ({unit})")
        ax.set_title(name)
        ax.legend(fontsize=9, loc="upper right")
        ax.grid(True, alpha=0.3)

        # Annotate LOA values
        ax.text(xhi, loa_hi, f" +{loa_hi:.1f}", va="bottom", fontsize=8,
                color=C_LOA)
        ax.text(xhi, loa_lo, f" {loa_lo:.1f}", va="top", fontsize=8,
                color=C_LOA)

    axes_flat[5].set_visible(False)
    fig.suptitle("Bland-Altman: SCG vs ECG HRV", fontsize=14, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    out = os.path.join(session_dir, "bland_altman.png")
    fig.savefig(out)
    print(f"  Saved {out}")
    return fig


# -- Correlation --------------------------------------------------------------
def plot_correlations(ecg_arrs, scg_arrs, session_dir):
    """Generate 5-panel correlation figure, save to session_dir."""
    fig, axes = plt.subplots(2, 3, figsize=(14, 9))
    axes_flat = axes.flatten()

    for idx, (key, name, unit) in enumerate(METRICS):
        ax = axes_flat[idx]
        ecg = ecg_arrs[key]
        scg = scg_arrs[key]

        ax.scatter(ecg, scg, c=C_CORR, s=50, alpha=0.75, edgecolors="k",
                   linewidths=0.3, zorder=5)

        # Identity line
        lo = min(ecg.min(), scg.min()) * 0.9
        hi = max(ecg.max(), scg.max()) * 1.1
        ax.plot([lo, hi], [lo, hi], "k--", linewidth=0.8, alpha=0.5,
                label="y = x")

        # Regression line
        if len(ecg) >= 3:
            coeffs = np.polyfit(ecg, scg, 1)
            xs = np.linspace(lo, hi, 50)
            ax.plot(xs, np.polyval(coeffs, xs), color=C_BA, linewidth=1.5)

        # Stats annotation
        r, p = pearsonr(ecg, scg)
        if np.isnan(p):
            p_str = "N/A"
        elif p < 0.001:
            p_str = "< 0.001"
        else:
            p_str = f"= {p:.3f}"
        ax.text(0.05, 0.95, f"r = {r:.3f}\np {p_str}", transform=ax.transAxes,
                fontsize=9, verticalalignment="top",
                bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

        ax.set_xlabel(f"ECG {name} ({unit})")
        ax.set_ylabel(f"SCG {name} ({unit})")
        ax.set_title(name)
        ax.legend(fontsize=8, loc="lower right")
        ax.grid(True, alpha=0.3)

    axes_flat[5].set_visible(False)
    fig.suptitle("Correlation: SCG vs ECG HRV", fontsize=14, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    out = os.path.join(session_dir, "correlation.png")
    fig.savefig(out)
    print(f"  Saved {out}")
    return fig


# -- BPM agreement ------------------------------------------------------------
def plot_bpm_agreement(beats, session_dir):
    """Plot ECG and SCG BPM over time, post-settling."""
    ecg = [(float(b["elapsed_s"]), float(b["bpm"]))
           for b in beats if b["source"] == "ecg" and
           float(b["elapsed_s"]) > SETTLE_S]
    scg = [(float(b["elapsed_s"]), float(b["bpm"]))
           for b in beats if b["source"] == "scg" and
           float(b["elapsed_s"]) > SETTLE_S]

    if not ecg and not scg:
        print("  No post-settling beat data for BPM plot.")
        return None

    fig, ax = plt.subplots(figsize=(12, 4.5))

    if ecg:
        et, eb = zip(*ecg)
        ax.plot(et, eb, color="#2166ac", linewidth=0.8, alpha=0.9,
                label="ECG BPM")
    if scg:
        st, sb = zip(*scg)
        ax.plot(st, sb, color="#b2182b", linewidth=0.8, alpha=0.8,
                label="SCG BPM")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Heart Rate (BPM)")
    ax.set_title("BPM Agreement: ECG vs SCG")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    out = os.path.join(session_dir, "bpm_agreement.png")
    fig.savefig(out)
    print(f"  Saved {out}")
    return fig


# -- Respiratory rate ---------------------------------------------------------
def plot_respiratory(resp, session_dir):
    """Plot respiratory rate time series with mean +/- SD band."""
    if not resp:
        print("  No respiratory data.")
        return None

    ts = np.array([float(r["elapsed_s"]) for r in resp])
    rates = np.array([float(r["rate_brpm"]) for r in resp])

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(ts, rates, color="#1a9850", linewidth=1.2, marker=".", markersize=4)

    mean_r = rates.mean()
    sd_r = rates.std()
    ax.axhline(mean_r, color="k", linestyle="--", linewidth=1.0, alpha=0.7)
    ax.fill_between(ax.get_xlim() if ax.get_xlim()[0] != ax.get_xlim()[1]
                    else [ts[0], ts[-1]],
                    mean_r - sd_r, mean_r + sd_r, color="#a6d96a", alpha=0.25)
    # Redraw fill after data-driven xlim
    xlo, xhi = ts[0], ts[-1]
    ax.fill_between([xlo, xhi], mean_r - sd_r, mean_r + sd_r,
                    color="#a6d96a", alpha=0.25)

    ax.text(0.02, 0.95, f"Mean: {mean_r:.1f} +/- {sd_r:.1f} BrPM (n={len(rates)})",
            transform=ax.transAxes, fontsize=10, verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8))

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Respiratory Rate (BrPM)")
    ax.set_title("Respiratory Rate")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    out = os.path.join(session_dir, "respiratory_rate.png")
    fig.savefig(out)
    print(f"  Saved {out}")
    return fig


# -- Summary table ------------------------------------------------------------
def print_metric_table(ecg_arrs, scg_arrs, label, n_pairs):
    """Print one method-comparison table for a given set of pairs."""
    print(f"\n  {label} (n={n_pairs}):")
    print(f"{'':2}{'Metric':12}{'ECG (mean+/-SD)':>18}{'SCG (mean+/-SD)':>18}"
          f"{'Bias':>8}{'95% LOA':>16}{'r':>8}{'ICC':>8}")
    print(f"{'':2}{'-'*12}{'-'*18}{'-'*18}{'-'*8}{'-'*16}{'-'*8}{'-'*8}")

    for key, name, unit in METRICS:
        ecg_v = ecg_arrs[key]
        scg_v = scg_arrs[key]
        diff = scg_v - ecg_v

        ecg_mean, ecg_sd = ecg_v.mean(), ecg_v.std()
        scg_mean, scg_sd = scg_v.mean(), scg_v.std()
        bias = diff.mean()
        sd_d = diff.std(ddof=1) if len(diff) > 1 else 0.0
        loa_lo = bias - 1.96 * sd_d
        loa_hi = bias + 1.96 * sd_d

        r, _ = pearsonr(ecg_v, scg_v)
        icc = compute_icc(ecg_v, scg_v)

        loa_str = f"[{loa_lo:+.1f}, {loa_hi:+.1f}]"
        print(f"  {name + ' (' + unit + ')':12}"
              f"{ecg_mean:7.1f}+/-{ecg_sd:<6.1f}"
              f"{scg_mean:7.1f}+/-{scg_sd:<6.1f}"
              f"{bias:+7.1f} {loa_str:>16}"
              f"{r:8.3f}{icc:8.3f}")


def print_summary(session_dir, ecg_hrv, scg_hrv, pairs, ecg_arrs, scg_arrs,
                  resp, beats, outlier_pairs=None,
                  all_ecg_arrs=None, all_scg_arrs=None):
    """Print formatted validation summary to stdout."""
    if outlier_pairs is None:
        outlier_pairs = []
    session_name = os.path.basename(session_dir)

    # Session metadata
    all_times = ([float(b["elapsed_s"]) for b in beats] +
                 [float(r["elapsed_s"]) for r in resp])
    duration = max(all_times) if all_times else 0
    mins, secs = divmod(int(duration), 60)

    ecg_beats = sum(1 for b in beats if b["source"] == "ecg")
    scg_beats = sum(1 for b in beats if b["source"] == "scg")

    ecg_post = [r for r in ecg_hrv if float(r["elapsed_s"]) > SETTLE_S]
    scg_post = [r for r in scg_hrv if float(r["elapsed_s"]) > SETTLE_S]

    total_paired = len(pairs) + len(outlier_pairs)

    sep = "=" * 72
    print(f"\n{sep}")
    print(f"  HRV Validation Analysis - {session_name}")
    print(sep)
    print(f"Duration:       {mins}m {secs}s")
    print(f"Beats:          {ecg_beats} ECG, {scg_beats} SCG")
    print(f"HRV reports:    {len(ecg_hrv)} ECG, {len(scg_hrv)} SCG "
          f"({len(ecg_post)} / {len(scg_post)} after {SETTLE_S:.0f}s settling)")
    if outlier_pairs:
        outlier_times = [f"{float(e['elapsed_s']):.0f}" for e, _ in outlier_pairs]
        print(f"Paired reports: {total_paired} "
              f"({len(outlier_pairs)} outliers at t="
              f"{', '.join(outlier_times)}s)")
    else:
        print(f"Paired reports: {total_paired}")
    print(f"Resp reports:   {len(resp)}")

    if total_paired < 2:
        print(f"\nInsufficient pairs ({total_paired}) for statistical analysis.")
        print(sep)
        return

    # Always show unfiltered results first
    if all_ecg_arrs and total_paired >= 2:
        print_metric_table(all_ecg_arrs, all_scg_arrs,
                           "All pairs", total_paired)

    # Show filtered results only if outliers were actually removed
    if outlier_pairs and len(pairs) >= 2:
        print_metric_table(ecg_arrs, scg_arrs,
                           "After outlier removal", len(pairs))
    elif not outlier_pairs and not all_ecg_arrs:
        # No outliers and no separate unfiltered arrays - just print once
        print_metric_table(ecg_arrs, scg_arrs, "All pairs", len(pairs))

    # Reference norms
    print(f"\n  Reference (Shaffer & Ginsberg 2017, 5-min resting):")
    print(f"    RMSSD: 19-75 ms | SDNN: 50-100 ms")

    # Respiratory
    if resp:
        rates = np.array([float(r["rate_brpm"]) for r in resp])
        print(f"\n  Respiratory rate: {rates.mean():.1f} +/- {rates.std():.1f} "
              f"BrPM (n={len(rates)})")

    print(f"\n  Plots saved to: {session_dir}/")
    print(sep + "\n")


# -- Main ---------------------------------------------------------------------
def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze.py <session_directory>")
        print("Example: python analyze.py data/session_20260410_012050")
        sys.exit(1)

    session_dir = resolve_session_path(sys.argv[1])
    session_name = os.path.basename(session_dir)
    print(f"Analyzing: {session_name}")

    if not HAS_SCIPY:
        print("Note: scipy not installed. p-values will show as N/A.")
        print("  Install with: pip install scipy\n")

    # Load data
    hrv = load_csv(os.path.join(session_dir, "hrv.csv"))
    beats = load_csv(os.path.join(session_dir, "beats.csv"))
    resp = load_csv(os.path.join(session_dir, "resp.csv"))

    ecg_hrv = [r for r in hrv if r["source"] == "ecg"]
    scg_hrv = [r for r in hrv if r["source"] == "scg"]

    # Pair HRV reports and filter outliers
    all_pairs = pair_hrv_reports(ecg_hrv, scg_hrv)
    pairs, outlier_pairs = filter_outliers(all_pairs)

    # Extract metric arrays from all pairs (for unfiltered stats)
    all_ecg_arrs, all_scg_arrs = {}, {}
    for key, _, _ in METRICS:
        all_ecg_arrs[key] = np.array([float(p[0][key]) for p in all_pairs])
        all_scg_arrs[key] = np.array([float(p[1][key]) for p in all_pairs])

    # Extract metric arrays from clean pairs (for filtered stats + plots)
    ecg_arrs, scg_arrs = {}, {}
    for key, _, _ in METRICS:
        ecg_arrs[key] = np.array([float(p[0][key]) for p in pairs])
        scg_arrs[key] = np.array([float(p[1][key]) for p in pairs])

    # Extract outlier arrays for plot marking
    outlier_ecg, outlier_scg = {}, {}
    for key, _, _ in METRICS:
        outlier_ecg[key] = np.array([float(p[0][key]) for p in outlier_pairs])
        outlier_scg[key] = np.array([float(p[1][key]) for p in outlier_pairs])

    # Generate plots
    print("\nGenerating plots...")

    if len(pairs) >= 2:
        plot_bland_altman(ecg_arrs, scg_arrs, session_dir,
                         outlier_ecg, outlier_scg)
        plot_correlations(ecg_arrs, scg_arrs, session_dir)
    else:
        print("  Skipping Bland-Altman and correlation (need >= 2 pairs).")

    plot_bpm_agreement(beats, session_dir)
    plot_respiratory(resp, session_dir)

    # Summary
    print_summary(session_dir, ecg_hrv, scg_hrv, pairs, ecg_arrs, scg_arrs,
                  resp, beats, outlier_pairs, all_ecg_arrs, all_scg_arrs)

    plt.close("all")


if __name__ == "__main__":
    main()
