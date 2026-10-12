#!/usr/bin/env python3
"""Numerical validation for Phase 4 reference Jerk CSV units."""

import csv
import math
import sys


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: check_phase4_csv.py <csv>", file=sys.stderr)
        return 2
    with open(sys.argv[1], newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise AssertionError("CSV contains no data rows")
    required = (
        "jx_ref_mm_s3", "jy_ref_mm_s3", "jz_ref_mm_s3",
        "jerk_ref_magnitude_mm_s3",
    )
    for row_number, row in enumerate(rows, start=2):
        values = [float(row[name]) for name in required]
        if not all(math.isfinite(value) for value in values):
            # Trapezoidal profiles deliberately export NaN for discontinuous
            # theoretical Jerk.  The magnitude must be NaN in that case too.
            if not all(value != value for value in values):
                raise AssertionError(f"non-finite mixed Jerk row {row_number}")
            continue
        expected = math.hypot(values[0], math.hypot(values[1], values[2]))
        if not math.isclose(values[3], expected, rel_tol=1e-9, abs_tol=1e-9):
            raise AssertionError(
                f"Jerk magnitude mismatch at row {row_number}: "
                f"{values[3]} != {expected}"
            )
        # Standard Phase 4 line profile has a 500 mm/s^3 Jerk vector at the
        # beginning of the S-curve.  This catches the historical 0.5 unit bug.
        if max(abs(v) for v in values[:3]) > 1e-12 and values[3] < 1.0:
            raise AssertionError(f"Jerk magnitude appears to be in m/s^3 at row {row_number}")
    print(f"validated {len(rows)} Phase 4 CSV rows")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, ValueError, OSError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)

