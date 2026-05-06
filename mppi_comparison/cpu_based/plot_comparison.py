import os
import pandas as pd
import matplotlib.pyplot as plt

# Create plots folder if it doesn't exist
os.makedirs("plots", exist_ok=True)

# Read CSV files from data folder
orig = pd.read_csv("data/original_python_results.csv")
cpp = pd.read_csv("data/cpp_api_cpu_results.csv")

plots = [
    ("cost", "Cost vs Step", "cost_comparison.png"),
    ("theta", "Theta vs Step", "theta_comparison.png"),
    ("theta_dot", "Theta Dot vs Step", "theta_dot_comparison.png"),
    ("action", "Action vs Step", "action_comparison.png"),
    ("step_time", "Step Time vs Step", "step_time_comparison.png"),
]

for col, title, filename in plots:
    plt.figure()

    plt.plot(
        orig["step"],
        orig[col],
        label="Original Python MPPI"
    )

    plt.plot(
        cpp["step"],
        cpp[col],
        label="C++ Backend via Python API"
    )

    plt.xlabel("Step")
    plt.ylabel(col)
    plt.title(title)
    plt.legend()
    plt.grid(True)

    plt.savefig(
        f"plots/{filename}",
        dpi=200,
        bbox_inches="tight"
    )

    plt.close()

# Runtime comparison bar chart
avg_orig = orig["step_time"].mean()
avg_cpp = cpp["step_time"].mean()

plt.figure()

plt.bar(
    ["Original Python", "C++ API"],
    [avg_orig, avg_cpp]
)

plt.ylabel("Average command time (s)")
plt.title("Average Command Time")
plt.grid(axis="y")

plt.savefig(
    "plots/runtime_bar_comparison.png",
    dpi=200,
    bbox_inches="tight"
)

plt.close()

print("Saved plots:")

for _, _, filename in plots:
    print(f"plots/{filename}")

print("plots/runtime_bar_comparison.png")
print()

print("Average original Python (CPU):", avg_orig)
print("Average C++ API (CPU):", avg_cpp)