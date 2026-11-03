"""Report common steady revolutions and print an angle-comparison command.

This reads prepared time bins; it does not alter acquisitions or render video.
The speed tolerance is a screening choice, not a synchronization accuracy claim.
Revolution numbering follows the first recorded angle, as in the media tools.
"""

import argparse
import csv
import math
import shlex
import statistics
import sys
from collections import Counter
from pathlib import Path


def read_csv(path):
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def ranges(numbers):
    groups = []
    for number in sorted(numbers):
        if groups and number == groups[-1][-1] + 1:
            groups[-1].append(number)
        else:
            groups.append([number])
    return groups


def analyse(rows, tolerance_percent=5.0, minimum_samples=10):
    """Screen forward full turns against the dominant nonzero motor command."""
    if len(rows) < 2:
        raise ValueError("At least two time-bin rows are needed")
    samples = []
    for row in rows:
        values = tuple(float(row[key]) for key in (
            "platform_angle_deg", "motor_speed_command_rpm", "recorded_motor_feedback_rpm",
        ))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Angle, command and feedback must be finite")
        samples.append(values)
    if any(b[0] < a[0] - 0.5 for a, b in zip(samples, samples[1:])):
        raise ValueError("Reversing motion detected; this check requires forward revolutions")
    commands = Counter(round(command, 6) for _, command, _ in samples if abs(command) >= 0.05)
    if not commands:
        raise ValueError("No nonzero motor-speed commands found")
    target = commands.most_common(1)[0][0]
    origin = samples[0][0]
    total_turns = math.floor((max(sample[0] for sample in samples) - origin) / 360.0)
    results = []
    for revolution in range(total_turns):
        start = origin + revolution * 360.0
        end = start + 360.0
        inside = [i for i, sample in enumerate(samples) if start <= sample[0] < end]
        if not inside:
            continue
        # Include samples bracketing the boundaries so a ramp at a crossing
        # cannot disappear merely because it falls just outside the interval.
        window = samples[max(0, inside[0] - 1):min(len(samples), inside[-1] + 2)]
        feedback = [sample[2] for sample in window]
        worst = max(abs(value - target) / abs(target) * 100 for value in feedback)
        command_error = max(abs(sample[1] - target) / abs(target) * 100 for sample in window)
        steady = len(inside) >= minimum_samples and worst <= tolerance_percent and command_error <= 1.0
        results.append(dict(revolution=revolution, count=len(inside),
                            mean=statistics.mean(feedback), minimum=min(feedback), maximum=max(feedback),
                            worst_percent=worst, steady=steady))
    return target, results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", type=Path, required=True)
    parser.add_argument("--runs", nargs="+", default=["run1", "run2", "run3"])
    parser.add_argument("--tolerance-percent", type=float, default=5.0)
    parser.add_argument("--minimum-samples", type=int, default=10)
    args = parser.parse_args()
    if not math.isfinite(args.tolerance_percent) or args.tolerance_percent <= 0 or args.minimum_samples < 2:
        parser.error("Use a positive tolerance and at least two samples")
    if not 2 <= len(args.runs) <= 3 or len(set(args.runs)) != len(args.runs):
        parser.error("Choose two or three distinct run labels")
    catalog_path = args.catalog.expanduser().resolve()
    try:
        catalog = read_csv(catalog_path)
        common = None
        for label in args.runs:
            matches = [row for row in catalog if row["run_label"] == label]
            if len(matches) != 1:
                raise ValueError(f"Expected one catalogue entry for {label}; found {len(matches)}")
            path = Path(matches[0]["time_bins_csv"])
            if not path.is_absolute():
                path = catalog_path.parent / path
            target, results = analyse(read_csv(path), args.tolerance_percent, args.minimum_samples)
            print(f"\n{label}: dominant command {target:g} MOTOR rpm")
            print(" rev   samples   mean rpm    min rpm    max rpm   worst error   steady")
            for result in results:
                print(f"{result['revolution']:4d} {result['count']:9d} {result['mean']:10.3f} "
                      f"{result['minimum']:10.3f} {result['maximum']:10.3f} "
                      f"{result['worst_percent']:11.2f}%   {'YES' if result['steady'] else 'no'}")
            good = {result["revolution"] for result in results if result["steady"]}
            common = good if common is None else common & good
        groups = ranges(common or set())
        if not groups:
            raise ValueError("No common steady full revolution passed. Review the speed table before changing the tolerance.")
        print("\nCommon steady ranges (inclusive, zero-based): "
              + ", ".join(f"{group[0]}-{group[-1]}" for group in groups))
        best = max(groups, key=len)
        selected = best[(len(best) - 1) // 2]
        print(f"Selected revolution {selected}, from the longest common range {best[0]}-{best[-1]}.")
        command = [sys.executable, str(Path(__file__).with_name("vpr_media.py").resolve()),
                   "--catalog", str(catalog_path), "--output-root", str(catalog_path.parent / "angle_verification"),
                   "compare-runs-by-angle", "--instances", *[f"{label}:{selected}" for label in args.runs],
                   "--angle-step", "0.5", "--accumulation-ms", "2", "--fps", "30"]
        print("\nRun this command to render the selected revolution:\n" + shlex.join(command))
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f"[ERROR] {exc}\n")


if __name__ == "__main__":
    main()
