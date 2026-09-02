import argparse
import json
from collections import defaultdict
from pathlib import Path
from statistics import mean, median


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Summarize detection and pose JSONL benchmark output'
    )
    parser.add_argument('paths', nargs='+', help='Prediction JSONL files')
    parser.add_argument(
        '--warmup-frames',
        type=int,
        default=1,
        help='Initial frames excluded from timing statistics',
    )
    return parser


def load_frames(path: Path) -> list[dict]:
    frames = []
    with path.open(encoding='utf-8') as input_file:
        for line_number, line in enumerate(input_file, start=1):
            if not line.strip():
                continue
            try:
                frames.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f'Invalid JSON at {path}:{line_number}') from exc
    if not frames:
        raise ValueError(f'No frames found in {path}')
    return frames


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = round((len(ordered) - 1) * fraction)
    return ordered[index]


def summarize(path: Path, warmup_frames: int) -> dict:
    frames = load_frames(path)
    measured_frames = frames[max(warmup_frames, 0) :] or frames
    people = [person for frame in frames for person in frame.get('people', [])]
    posed_people = [person for person in people if person.get('keypoint_count', 0) > 0]

    timings: dict[str, list[float]] = defaultdict(list)
    for frame in measured_frames:
        for name, value in frame.get('timings_ms', {}).items():
            timings[name].append(float(value))

    timing_summary = {
        name: {
            'mean': mean(values),
            'p50': median(values),
            'p95': percentile(values, 0.95),
        }
        for name, values in timings.items()
        if values
    }
    backend_total = timing_summary.get('backend_total', {}).get('mean', 0.0)

    return {
        'path': str(path),
        'stage': frames[0].get('stage', 'unknown'),
        'backend': frames[0].get('backend', 'unknown'),
        'frames': len(frames),
        'people': len(people),
        'average_people_per_frame': len(people) / len(frames),
        'average_valid_keypoints': (
            mean(person.get('valid_keypoint_count', 0) for person in posed_people)
            if posed_people
            else 0.0
        ),
        'estimated_backend_fps': 1000.0 / backend_total if backend_total > 0 else 0.0,
        'timings': timing_summary,
    }


def print_summary(summary: dict) -> None:
    print(summary['path'])
    print(f"  pipeline: {summary['stage']} / {summary['backend']}")
    print(
        f"  frames: {summary['frames']} | people: {summary['people']} | "
        f"average/frame: {summary['average_people_per_frame']:.2f}"
    )
    print(
        f"  average valid keypoints/person: "
        f"{summary['average_valid_keypoints']:.2f}"
    )
    print(f"  estimated backend FPS: {summary['estimated_backend_fps']:.2f}")
    print('  timings (ms):')
    for name, values in sorted(summary['timings'].items()):
        print(
            f"    {name}: mean={values['mean']:.2f} "
            f"p50={values['p50']:.2f} p95={values['p95']:.2f}"
        )


def main() -> None:
    args = build_parser().parse_args()
    if args.warmup_frames < 0:
        raise ValueError('warmup_frames must not be negative')

    for index, raw_path in enumerate(args.paths):
        if index:
            print()
        print_summary(summarize(Path(raw_path), args.warmup_frames))


if __name__ == '__main__':
    main()
