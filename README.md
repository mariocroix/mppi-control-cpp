# MPPI C++

A compact C++ implementation of Model Predictive Path Integral Control (MPPI), with a smoothed MPPI variant, pybind11 Python bindings, pendulum examples, CPU benchmarking, and reproducible comparison plots.

The project is intentionally small and direct: generic CPU controllers, Eigen-based linear algebra, OpenMP rollout acceleration, CMake builds, and unit tests.

## Features

- `MPPIController` for standard MPPI control.
- `SMPPIController` for smooth action-sequence MPPI.
- Generic dynamics and cost interfaces.
- Pendulum dynamics, running cost, and terminal cost examples.
- pybind11 module named `mppi_cpp`.
- OpenMP-accelerated rollout evaluation.
- Contiguous rollout storage for better cache locality.
- CMake build system.
- C++ unit tests through CTest.
- Python comparison scripts, CPU sweeps, and plotting utilities.

## Repository Structure

```text
.
├── bindings/                 # pybind11 Python module
├── examples/                 # C++ pendulum examples
├── include/mppi/             # Public C++ headers
├── mppi_comparison/
│   └── cpu_based/
│       ├── data/             # Benchmark CSV files
│       ├── plots/            # Generated comparison figures
│       └── plot_*.py         # Plotting scripts
├── src/                      # Controller implementations and demo main
├── tests/                    # C++ tests and Python benchmark scripts
└── CMakeLists.txt
```

## Build Instructions

Dependencies:

- CMake 3.16+
- C++17 compiler
- Eigen3
- OpenMP
- pybind11
- Python with NumPy for Python examples and benchmarks

On macOS, the CMake file expects Homebrew `libomp` under `/opt/homebrew/opt/libomp`.

```bash
cmake -S . -B build
cmake --build build
```

The build produces:

- `build/libmppi.a`
- `build/mppi_cpp*.so`
- `build/mppi_demo`
- `build/pendulum_example`
- `build/smppi_pendulum_example`
- C++ test executables

## Running Examples

Run the standard MPPI pendulum example:

```bash
./build/pendulum_example
```

Run the smoothed MPPI pendulum example:

```bash
./build/smppi_pendulum_example
```

Run the small demo executable:

```bash
./build/mppi_demo
```

## Running Tests

```bash
ctest --test-dir build --output-on-failure
```

You can also run individual test binaries:

```bash
./build/test_mppi_basic
./build/test_smppi_basic
```

## Python Bindings Usage

The Python module is built into `build/` as `mppi_cpp`.

```bash
PYTHONPATH=build python -c "import mppi_cpp; print('import ok')"
```

Minimal pendulum usage:

```python
import numpy as np
import mppi_cpp as m

cfg = m.Config()
cfg.nx = 2
cfg.nu = 1
cfg.K = 200
cfg.T = 15
cfg.lambda_ = 1.0
cfg.uScale = 1.0

cfg.uMin = np.array([-2.0])
cfg.uMax = np.array([2.0])
cfg.uInit = np.array([0.0])
cfg.noiseMu = np.array([0.0])
cfg.noiseSigmaDiag = np.array([10.0])

dynamics = m.PendulumDynamics()
cost = m.PendulumCost()
controller = m.MPPIController(cfg, dynamics, cost)

state = np.array([np.pi, 1.0])
action = controller.command(state)
next_state = dynamics.propagate(state, action, 0)
```

Run the included Python API examples:

```bash
PYTHONPATH=build python tests/test_mppi_api.py
PYTHONPATH=build python tests/test_smppi_api.py
```

## Pendulum Comparison Workflow

The comparison tools live under `mppi_comparison/cpu_based`. Existing CSV files and plots are checked in under `data/` and `plots/`.

Generate a fresh C++ Python API trajectory CSV:

```bash
cd mppi_comparison/cpu_based
PYTHONPATH=../../build python ../../tests/run_cpp_api_results.py
mv cpp_api_results.csv data/cpp_api_cpu_results.csv
```

Generate a fresh C++ CPU sweep CSV:

```bash
cd mppi_comparison/cpu_based
PYTHONPATH=../../build python ../../tests/run_cpp_api_cpu_sweep.py
mv cpp_api_cpu_sweep.csv data/cpp_api_cpu_sweep.csv
```

Regenerate comparison plots:

```bash
cd mppi_comparison/cpu_based
python plot_comparison.py
python plot_sweep_comparison.py
```

The Gym rendering comparison script is available at `tests/run_cpp_api_gym_results.py` and requires `gym` plus a local display/rendering environment.

## Results

The current CPU implementation uses contiguous rollout buffers and OpenMP rollout evaluation for sample counts of `K >= 100`.

Recent local benchmark results for the pendulum workload with horizon `T=15`:

| Samples K | Average command time |
|---:|---:|
| 200 | 0.006748 s |
| 500 | 0.015313 s |
| 1000 | 0.029680 s |
| 2000 | 0.058890 s |

For the included K=200 CPU sweep, average command time is approximately `0.006 s` per command after warmup-length effects. Numerical outputs matched the previous controller behavior during validation.

## Technologies Used

- C++17
- Eigen3
- OpenMP
- CMake
- pybind11
- Python
- NumPy
- pandas and Matplotlib for plotting
- CTest

## Future Work

- CUDA backend.
- GPU rollout acceleration.
- Batched environments for larger-scale benchmarking and training loops.

## Author

Mari Mkrtchyan
