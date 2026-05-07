import os
import pandas as pd
import matplotlib.pyplot as plt

os.makedirs("plots", exist_ok=True)

orig = pd.read_csv("data/original_python_cpu_sweep.csv")
cpp = pd.read_csv("data/cpp_api_cpu_sweep.csv")

plots = [
    ("total_command_time", "Total Command Time vs Episode Steps", "sweep_total_time.png"),
    ("average_command_time", "Average Command Time vs Episode Steps", "sweep_average_time.png"),
    ("final_cost", "Final Cost vs Episode Steps", "sweep_final_cost.png"),
    ("final_theta", "Final Theta vs Episode Steps", "sweep_final_theta.png"),
    ("final_theta_dot", "Final Theta Dot vs Episode Steps", "sweep_final_theta_dot.png"),
]

for col, title, filename in plots:
    plt.figure()
    plt.plot(orig["episode_steps"], orig[col], marker="o", label="Original Python MPPI")
    plt.plot(cpp["episode_steps"], cpp[col], marker="o", label="C++ Backend via Python API")
    plt.xlabel("Episode steps")
    plt.ylabel(col)
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.savefig(f"plots/{filename}", dpi=200, bbox_inches="tight")
    plt.close()

print("Saved sweep plots:")
for _, _, filename in plots:
    print(f"plots/{filename}")

print()
print("Original Python sweep:")
print(orig)

print()
print("C++ API sweep:")
print(cpp)