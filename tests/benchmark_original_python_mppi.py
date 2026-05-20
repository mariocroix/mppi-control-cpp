from pathlib import Path

import pandas as pd


CSV_PATH = (
    Path(__file__).resolve().parents[1]
    / "mppi_comparison"
    / "cpu_based"
    / "data"
    / "original_python_cpu_sweep.csv"
)


def main():
    if not CSV_PATH.exists():
        raise FileNotFoundError(f"Missing original Python benchmark CSV: {CSV_PATH}")

    df = pd.read_csv(CSV_PATH)
    print("Original Python MPPI reference benchmark data:")
    print(df.to_string(index=False))
    print(f"\nReference CSV: {CSV_PATH}")
    print("This script does not rewrite the original Python benchmark results.")


if __name__ == "__main__":
    main()
