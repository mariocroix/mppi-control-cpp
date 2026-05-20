from pathlib import Path
import sys

import matplotlib.pyplot as plt
import pandas as pd


ROOT = Path(__file__).resolve().parent
DATA_DIR = ROOT / "data"
PLOT_DIR = ROOT / "plots"

REQUIRED_COLUMNS = [
    "episode_steps",
    "total_command_time",
    "average_command_time",
    "final_theta",
    "final_theta_dot",
    "final_cost",
]

SERIES = [
    ("Original Python MPPI", "original_python_cpu_sweep.csv"),
    ("C++ API: 3 calls/step", "cpp_api_3_call_cpu_sweep.csv"),
    ("Native C++", "cpp_native_cpu_sweep.csv"),
]

LINE_PLOTS = [
    (
        "average_command_time",
        "Average Command Time vs Episode Length",
        "Episode steps",
        "Average command time (s)",
        "average_command_time_comparison.png",
    ),
    (
        "total_command_time",
        "Total Command Time vs Episode Length",
        "Episode steps",
        "Total command time (s)",
        "total_command_time_comparison.png",
    ),
    (
        "final_cost",
        "Final Cost vs Episode Length",
        "Episode steps",
        "Final cost",
        "final_cost_comparison.png",
    ),
    (
        "final_theta",
        "Final Theta vs Episode Length",
        "Episode steps",
        "Final theta (rad)",
        "final_theta_comparison.png",
    ),
    (
        "final_theta_dot",
        "Final Angular Velocity vs Episode Length",
        "Episode steps",
        "Final theta dot (rad/s)",
        "final_theta_dot_comparison.png",
    ),
]

COLORS = ["#4C78A8", "#F58518", "#777777"]


def load_benchmark_data():
    frames = []

    for label, filename in SERIES:
        path = DATA_DIR / filename
        if not path.exists():
            print(f"Error: missing required CSV file: {path}", file=sys.stderr)
            sys.exit(1)

        df = pd.read_csv(path)
        missing_columns = [col for col in REQUIRED_COLUMNS if col not in df.columns]
        if missing_columns:
            print(
                f"Error: {path} is missing columns: {', '.join(missing_columns)}",
                file=sys.stderr,
            )
            sys.exit(1)

        frames.append((label, df))

    return frames


def save_current_figure(path):
    plt.tight_layout()
    plt.savefig(path, dpi=300, bbox_inches="tight")
    plt.close()
    print(f"Saved {path}")


def plot_line_comparisons(frames):
    saved_paths = []

    for metric, title, xlabel, ylabel, filename in LINE_PLOTS:
        fig, ax = plt.subplots(figsize=(8.0, 5.0))

        for index, (label, df) in enumerate(frames):
            ax.plot(
                df["episode_steps"],
                df[metric],
                marker="o",
                linewidth=2.0,
                markersize=5.0,
                color=COLORS[index],
                label=label,
            )

        ax.set_title(title, fontsize=13, weight="bold")
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True, linestyle="--", linewidth=0.6, alpha=0.7)
        ax.legend(frameon=True, fontsize=9)

        path = PLOT_DIR / filename
        save_current_figure(path)
        saved_paths.append(path)

    return saved_paths


def rows_for_200_steps(frames):
    rows = []

    for label, df in frames:
        row = df[df["episode_steps"] == 200]
        if row.empty:
            print(
                f"Error: CSV for '{label}' does not contain episode_steps == 200",
                file=sys.stderr,
            )
            sys.exit(1)
        rows.append((label, row.iloc[0]))

    return rows


def plot_runtime_bar(rows_200):
    labels = [label for label, _ in rows_200]
    values = [row["average_command_time"] for _, row in rows_200]

    fig, ax = plt.subplots(figsize=(8.0, 5.0))
    ax.bar(labels, values, color=COLORS, edgecolor="#333333", linewidth=0.7)
    ax.set_title("Average Command Time at 200 Steps", fontsize=13, weight="bold")
    ax.set_ylabel("Average command time (s)")
    ax.grid(True, axis="y", linestyle="--", linewidth=0.6, alpha=0.7)
    ax.tick_params(axis="x", rotation=15)

    path = PLOT_DIR / "runtime_bar_200_steps.png"
    save_current_figure(path)
    return path


def plot_final_state_bar(rows_200):
    labels = [label for label, _ in rows_200]
    theta = [row["final_theta"] for _, row in rows_200]
    theta_dot = [row["final_theta_dot"] for _, row in rows_200]

    x_positions = range(len(labels))
    width = 0.36

    fig, ax = plt.subplots(figsize=(8.5, 5.0))
    ax.bar(
        [x - width / 2 for x in x_positions],
        theta,
        width,
        label="final_theta",
        color="#4C78A8",
        edgecolor="#333333",
        linewidth=0.7,
    )
    ax.bar(
        [x + width / 2 for x in x_positions],
        theta_dot,
        width,
        label="final_theta_dot",
        color="#F58518",
        edgecolor="#333333",
        linewidth=0.7,
    )

    ax.set_title("Final State at 200 Steps", fontsize=13, weight="bold")
    ax.set_ylabel("Value")
    ax.set_xticks(list(x_positions))
    ax.set_xticklabels(labels, rotation=15, ha="right")
    ax.grid(True, axis="y", linestyle="--", linewidth=0.6, alpha=0.7)
    ax.legend(frameon=True)

    path = PLOT_DIR / "final_state_bar_200_steps.png"
    save_current_figure(path)
    return path


def plot_summary_table(rows_200):
    summary = pd.DataFrame(
        [
            {
                "Execution path": label,
                "Avg command time (s)": row["average_command_time"],
                "Total command time (s)": row["total_command_time"],
                "Final cost": row["final_cost"],
            }
            for label, row in rows_200
        ]
    )

    display = summary.copy()
    for col in ["Avg command time (s)", "Total command time (s)", "Final cost"]:
        display[col] = display[col].map(lambda value: f"{value:.6g}")

    fig, ax = plt.subplots(figsize=(10.0, 2.8))
    ax.axis("off")
    table = ax.table(
        cellText=display.values,
        colLabels=display.columns,
        cellLoc="center",
        colLoc="center",
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(9.5)
    table.scale(1.0, 1.45)

    for (row_index, _), cell in table.get_celld().items():
        cell.set_edgecolor("#CCCCCC")
        if row_index == 0:
            cell.set_text_props(weight="bold")
            cell.set_facecolor("#EAEAEA")
        else:
            cell.set_facecolor("#FFFFFF")

    ax.set_title("Performance Summary at 200 Steps", fontsize=13, weight="bold", pad=12)

    path = PLOT_DIR / "performance_summary_table.png"
    save_current_figure(path)
    return path, summary


def main():
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    frames = load_benchmark_data()
    saved_paths = []
    saved_paths.extend(plot_line_comparisons(frames))

    rows_200 = rows_for_200_steps(frames)
    saved_paths.append(plot_runtime_bar(rows_200))
    saved_paths.append(plot_final_state_bar(rows_200))
    table_path, summary = plot_summary_table(rows_200)
    saved_paths.append(table_path)

    print("\nSummary at 200 steps:")
    print(summary.to_string(index=False))

    print("\nAll generated plots:")
    for path in saved_paths:
        print(path)


if __name__ == "__main__":
    main()
