
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", nargs="?", default="build/feature_windows.csv")
    parser.add_argument("--features", type=int, default=6)
    parser.add_argument("--bins", type=int, default=25)
    parser.add_argument("--output", default="optical_flow_stats.png")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)

    required = [
        "id", "window_start", "flow_x", "flow_y",
        "cost", "iterations", "converged"
    ]
    for col in required:
        if col not in df.columns:
            raise ValueError(f"Missing CSV column: {col}")

    df = df.dropna(subset=required).copy()
    df["converged"] = (
        df["converged"].astype(str).str.lower().isin(["true", "1"])
    )
    df = df.sort_values(["id", "window_start"])

    df["window_id"] = df.groupby("id").cumcount() + 1
    df["time_s"] = (
        df["window_start"] - df["window_start"].min()
    ) / 1e6

    df["flow_mag"] = np.sqrt(
        df["flow_x"] ** 2 + df["flow_y"] ** 2
    )

    chosen_ids = (
        df.groupby("id")
        .size()
        .nlargest(max(args.features, 1))
        .index
    )
    selected = df[df["id"].isin(chosen_ids)]

    fig, axes = plt.subplots(
        2, 2, figsize=(14, 9), constrained_layout=True
    )
    fig.suptitle(
        f"EVIO Optical Flow Diagnostics — {len(df):,} completed windows",
        fontsize=16
    )

    # 1. Cost vs. Window ID
    ax = axes[0, 0]
    for fid, group in selected.groupby("id", sort=False):
        ax.plot(
            group["window_id"],
            group["cost"],
            linewidth=1.2,
            label=f"ID {fid}"
        )
    ax.set(
        title="Cost vs. Window ID",
        xlabel="Window ID (per feature)",
        ylabel="Final cost"
    )
    ax.legend(fontsize=8, ncol=2)
    ax.grid(alpha=0.25)

    # 2. Iterations per Completed Window
    ax = axes[0, 1]
    max_iter = int(df["iterations"].max())
    ax.hist(
        df["iterations"],
        bins=np.arange(-0.5, max_iter + 1.5, 1)
    )
    ax.set(
        title="Iterations per Completed Window",
        xlabel="Iterations",
        ylabel="Number of windows"
    )
    ax.grid(alpha=0.25)

    # 3. Flow Magnitude vs. Time
    # Mean, median, and 90th percentile across all features
    ax = axes[1, 0]

    edges = np.linspace(
        df["time_s"].min(),
        df["time_s"].max(),
        max(2, args.bins) + 1
    )

    flow_data = df.copy()
    flow_data["time_bin"] = pd.cut(
        flow_data["time_s"],
        edges,
        include_lowest=True
    )

    flow_stats = flow_data.groupby(
        "time_bin", observed=True
    ).agg(
        time=("time_s", "mean"),
        mean=("flow_mag", "mean"),
        median=("flow_mag", "median"),
        p90=("flow_mag", lambda x: x.quantile(0.9))
    )

    ax.plot(
        flow_stats["time"],
        flow_stats["mean"],
        label="Mean"
    )
    ax.plot(
        flow_stats["time"],
        flow_stats["median"],
        label="Median"
    )
    ax.plot(
        flow_stats["time"],
        flow_stats["p90"],
        label="90th percentile"
    )

    ax.set(
        title="Flow Magnitude vs. Time",
        xlabel="Camera time (s)",
        ylabel="Flow magnitude (native flow units)"
    )
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25)

    # 4. Convergence Rate vs. Time
    ax = axes[1, 1]

    time_bins = pd.cut(
        df["time_s"],
        bins=max(2, args.bins),
        include_lowest=True
    )

    convergence = df.groupby(
        time_bins, observed=True
    ).agg(
        time=("time_s", "mean"),
        rate=("converged", "mean")
    )

    ax.plot(
        convergence["time"],
        convergence["rate"] * 100,
        marker="o",
        markersize=3
    )

    ax.set(
        title="Convergence Rate vs. Time",
        xlabel="Camera time (s)",
        ylabel="Converged windows (%)",
        ylim=(0, 105)
    )
    ax.grid(alpha=0.25)

    # Automatic figure saving disabled
    # output = Path(args.output)
    # fig.savefig(output, dpi=180)
    # print(f"Saved {output.resolve()}")

    print(
        f"Windows: {len(df):,} | "
        f"Features: {df['id'].nunique():,} | "
        f"Mean iterations: {df['iterations'].mean():.2f}"
    )
    print(f"Convergence rate: {df['converged'].mean() * 100:.1f}%")

    plt.show()


if __name__ == "__main__":
    main()
