import csv
from pathlib import Path

import matplotlib.pyplot as plt


csv_path = Path("results/lookahead_summary.csv")

lookahead = []
rmse = []
samples = []

with csv_path.open() as f:
    reader = csv.DictReader(f)

    for row in reader:
        lookahead.append(float(row["lookahead_m"]))
        rmse.append(float(row["rmse_m"]))
        samples.append(int(row["samples"]))


# RMSE figure
fig, ax = plt.subplots(figsize=(6, 4))

ax.plot(
    lookahead,
    rmse,
    marker="o",
)

for x, y in zip(lookahead, rmse):
    ax.annotate(
        f"{y:.3f}",
        (x, y),
        xytext=(0, 8),
        textcoords="offset points",
        ha="center",
    )

ax.set_xlabel("Lookahead Distance (m)")
ax.set_ylabel("RMSE (m)")
ax.set_title("Pure Pursuit Lookahead Sensitivity - RMSE")
ax.grid(True, alpha=0.3)

fig.tight_layout()
fig.savefig(
    "docs/images/lookahead_rmse.png",
    dpi=180,
)

plt.close(fig)


# Samples figure
fig, ax = plt.subplots(figsize=(6, 4))

ax.plot(
    lookahead,
    samples,
    marker="o",
)

for x, y in zip(lookahead, samples):
    ax.annotate(
        str(y),
        (x, y),
        xytext=(0, 8),
        textcoords="offset points",
        ha="center",
    )

ax.set_xlabel("Lookahead Distance (m)")
ax.set_ylabel("Tracking Samples")
ax.set_title("Pure Pursuit Lookahead Sensitivity - Samples")
ax.grid(True, alpha=0.3)

fig.tight_layout()
fig.savefig(
    "docs/images/lookahead_samples.png",
    dpi=180,
)

plt.close(fig)

print("Generated:")
print("  docs/images/lookahead_rmse.png")
print("  docs/images/lookahead_samples.png")
