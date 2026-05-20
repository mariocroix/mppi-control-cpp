# CPU Benchmark Comparison

This directory compares three pendulum MPPI execution paths:

- Original Python MPPI reference results from `data/original_python_cpu_sweep.csv`.
- C++ API: 3 calls/step, where Python calls `command()`, `propagate()`, and `evaluate()` separately.
- Native C++ executable with no Python overhead.

All sweep CSVs use:

- `episode_steps`
- `total_command_time`
- `average_command_time`
- `final_theta`
- `final_theta_dot`
- `final_cost`

For the generated C++ benchmark CSVs, `total_command_time` and
`average_command_time` measure the full timed timestep:

- 3-call API: Python call to `command()`, Python call to `propagate()`, and
  Python call to `evaluate()`.
- Native C++: C++ command, propagate, and cost evaluation.

The original Python CSV is fixed reference data from the original implementation.

Run from the repository root after a Release build. The original Python CSV is
kept as fixed reference data; regenerate only the C++ benchmark CSVs here.

```sh
PYTHONPATH=build python tests/benchmark_cpp_api_3_call.py
./build/benchmark_cpp_native
python mppi_comparison/cpu_based/plot_benchmark_results.py
```

CSV files are written to `data/`. Plots are written to `plots/`.
