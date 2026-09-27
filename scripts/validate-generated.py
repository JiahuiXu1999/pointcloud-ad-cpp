#!/usr/bin/env python3
"""Generate labelled millimetre fixtures and validate complete CLI inspections (stdlib only)."""

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import random
import subprocess


def save_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def configuration(backend, workers):
    return {
        "schema_version": "1.0", "profile": "synthetic_demo",
        "input": {"reference_unit": "millimeter", "scan_unit": "millimeter",
                  "reference_frame": "fixture", "scan_frame": "scanner"},
        "preprocess": {"voxel_size_mm": 0.2, "normal_radius_mm": 1.0,
                       "normal_min_neighbors": 12, "boundary_radius_mm": 0.8},
        "registration": {"method": "point_to_plane", "max_iterations": 60,
                         "max_correspondence_distance_mm": 1.0, "huber_delta_mm": 0.3,
                         "translation_epsilon_mm": 0.0001, "rotation_epsilon_rad": 0.00001,
                         "residual_epsilon_mm": 0.00001},
        "registration_gate": {"min_overlap_ratio": 0.7, "max_inlier_rmse_mm": 0.2,
                              "min_valid_pairs": 500, "max_translation_from_initial_mm": 5.0,
                              "max_rotation_from_initial_deg": 5.0},
        "comparison": {"max_search_distance_mm": 0.8, "max_normal_angle_deg": 35.0,
                       "boundary_exclusion_mm": 0.6, "min_valid_coverage_ratio": 0.75},
        "detection": {"positive_threshold_mm": 0.25, "negative_threshold_mm": -0.25,
                      "cluster_tolerance_mm": 0.6, "min_cluster_points": 20,
                      "measurement_error_budget_mm": 0.0},
        "execution": {"backend": backend, "deterministic": True,
                      "thread_count": workers, "random_seed": 5489},
    }


def write_ply(path, points):
    header = ("ply\nformat ascii 1.0\ncomment generated ground truth; units millimetres\n"
              f"element vertex {len(points)}\n"
              "property float x\nproperty float y\nproperty float z\n"
              "property float nx\nproperty float ny\nproperty float nz\nend_header\n")
    body = "".join(" ".join(format(0.0 if abs(v) < 1e-8 else v, ".9g") for v in p) + " 0 0 1\n" for p in points)
    path.write_bytes((header + body).encode("ascii"))
    return hashlib.sha256(path.read_bytes()).hexdigest()


def generate(root, side):
    root.mkdir(parents=True, exist_ok=True)
    points = [((x - side // 2) * 0.25, (y - side // 2) * 0.25, 0.0)
              for y in range(side) for x in range(side)]
    reference_hash = write_ply(root / "reference.ply", points)
    rng = random.Random(5489)
    scans = {
        "normal": points,
        "bump": [(x, y, 0.7 * math.exp(-(x*x + y*y) / 4.5)) for x, y, _ in points],
        "dent": [(x, y, -0.7 * math.exp(-(x*x + y*y) / 4.5)) for x, y, _ in points],
        "missing": [(x, y, z) for x, y, z in points if x*x + y*y > 9.0],
        "noise": [(x, y, rng.uniform(-0.015, 0.015)) for x, y, _ in points],
        "density": [p for i, p in enumerate(points) if p[0] <= 0 or i % 2 == 0],
        "translated": [(x, y, -0.1) for x, y, _ in points],
        "bad_pose": [(x, y, 20.0) for x, y, _ in points],
    }
    scenes = []
    for name, scan in scans.items():
        defect = {"bump": "bump", "dent": "dent", "missing": "missing_material"}.get(name)
        scenes.append({"name": name, "file": f"{name}.ply", "point_count": len(scan),
                       "sha256": write_ply(root / f"{name}.ply", scan),
                       "verdict": "indeterminate" if name == "bad_pose" else "fail" if defect else "pass",
                       "defect_type": defect,
                       "injected_depth_mm": 0.7 if name == "bump" else -0.7 if name == "dent" else 0,
                       "missing_radius_mm": 3.0 if name == "missing" else 0,
                       "translation_z_mm": 0.1 if name == "translated" else 0})
    manifest = {"generator": "pcad-generated-v1", "seed": 5489, "units": "millimetres",
                "side": side, "spacing_mm": 0.25, "normal": [0, 0, 1],
                "coordinate_zero_floor_mm": 1e-8, "noise_uniform_mm": [-0.015, 0.015], "gaussian_sigma_mm": 1.5,
                "reference": {"file": "reference.ply", "point_count": len(points), "sha256": reference_hash},
                "scenes": scenes, "scope": "synthetic verification; not industrial acceptance"}
    save_json(root / "ground-truth.json", manifest)
    return manifest


def business(result):
    result = copy.deepcopy(result)
    result.pop("timings", None)
    result.pop("provenance", None)
    for key in ("requested_backend", "actual_backend", "execution_scope"):
        result["registration"].pop(key, None)
    return result


def equivalent(left, right, tolerance, path="result"):
    if isinstance(left, dict):
        if not isinstance(right, dict) or left.keys() != right.keys():
            raise AssertionError(f"{path}: different fields")
        for key in left:
            equivalent(left[key], right[key], tolerance, f"{path}.{key}")
    elif isinstance(left, list):
        if not isinstance(right, list) or len(left) != len(right):
            raise AssertionError(f"{path}: different lengths")
        for i, (a, b) in enumerate(zip(left, right)):
            equivalent(a, b, tolerance, f"{path}[{i}]")
    elif isinstance(left, float) or isinstance(right, float):
        if not isinstance(left, (int, float)) or not isinstance(right, (int, float)):
            raise AssertionError(f"{path}: different numeric types")
        if not math.isfinite(left) or not math.isfinite(right) or abs(left-right) > tolerance:
            raise AssertionError(f"{path}: {left} != {right} (absolute tolerance {tolerance})")
    elif left != right:
        raise AssertionError(f"{path}: {left!r} != {right!r}")


def check_truth(result, scene):
    if result["verdict"] != scene["verdict"]:
        raise AssertionError(f"{scene['name']}: expected {scene['verdict']}, got {result['verdict']}")
    if result["deviations"]["input_invalid"] != 0:
        raise AssertionError("generated finite samples were unexpectedly discarded")
    regions = result["regions"]
    if scene["defect_type"]:
        matching = [r for r in regions if r["type"] == scene["defect_type"]]
        if not matching:
            raise AssertionError(f"{scene['name']}: missing labelled defect")
        if scene["name"] in ("bump", "dent"):
            region = max(matching, key=lambda r: r["max_abs_mm"])
            if abs(region["max_abs_mm"] - 0.7) > 0.06:
                raise AssertionError("injected peak not recovered within 0.06 mm")
            if math.hypot(region["centroid"]["x"], region["centroid"]["y"]) > 0.5:
                raise AssertionError("defect centroid exceeds 0.5 mm truth tolerance")
            if region["mean_mm"] * scene["injected_depth_mm"] <= 0:
                raise AssertionError("defect sign is wrong")
    elif regions:
        raise AssertionError(f"{scene['name']}: unexpected defect regions")
    if scene["verdict"] == "pass" and result["deviations"]["max_abs_mm"] > 0.03:
        raise AssertionError("normal/noise/pose fixture residual exceeds 0.03 mm")
    if scene["name"] == "translated":
        pose = result["registration"]["final_pose"]["matrix"]
        if abs(pose[11] - 0.1) > 0.002:
            raise AssertionError("known scan-to-reference translation not recovered")
    if scene["name"] == "missing" and result["coverage"]["no_neighbor"] < 100:
        raise AssertionError("known missing core not detected")


def validate(args, manifest):
    oracle = {}
    per_backend = {}
    rows = []
    executable = args.pcad.resolve()
    for backend in args.backends:
        for workers in args.threads:
            config_path = args.output / f"config-{backend}-{workers}.json"
            save_json(config_path, configuration(backend, workers))
            for scene in manifest["scenes"]:
                key = f"{backend}-{workers}/{scene['name']}"
                destination = args.output / "runs" / key
                destination.mkdir(parents=True, exist_ok=True)
                proc = subprocess.run([str(executable), "inspect", str(config_path),
                                       str(args.output / "reference.ply"), str(args.output / scene["file"]),
                                       "--output", str(destination)], capture_output=True, text=True,
                                      encoding="utf-8", errors="replace", timeout=180)
                if proc.returncode != 0:
                    raise RuntimeError(f"{key}: process failure {proc.returncode}: {proc.stderr}")
                result = json.loads((destination / "result.json").read_text(encoding="utf-8"))
                check_truth(result, scene)
                actual = result["registration"]["actual_backend"]
                if backend != "auto" and actual != backend:
                    raise AssertionError(f"{key}: explicit backend was not respected")
                if backend == "auto":
                    large = min(manifest["reference"]["point_count"], scene["point_count"]) >= 65536
                    if not large and actual != "cpu":
                        raise AssertionError(f"{key}: small auto fixture must select CPU")
                    if large and "gpu" in args.backends and actual != "gpu":
                        raise AssertionError(f"{key}: eligible auto fixture must select available GPU")
                metrics = business(result)
                same_key = (actual, scene["name"])
                if same_key in per_backend:
                    equivalent(per_backend[same_key], metrics, 0.0)
                else:
                    per_backend[same_key] = metrics
                if scene["name"] in oracle:
                    equivalent(oracle[scene["name"]], metrics, 1e-5)
                else:
                    oracle[scene["name"]] = metrics
                rows.append({"case": key, "process_exit": proc.returncode, "verdict": result["verdict"],
                             "actual_backend": actual, "regions": len(result["regions"]),
                             "max_abs_mm": result["deviations"]["max_abs_mm"],
                             "coverage_ratio": result["coverage"]["coverage_ratio"]})
                print(f"PASS {key}: {result['verdict']} ({actual})", flush=True)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pcad", type=Path)
    parser.add_argument("--side", type=int, default=65)
    parser.add_argument("--backends", nargs="+", choices=["cpu", "gpu", "auto"], default=["cpu", "auto"])
    parser.add_argument("--threads", nargs="+", type=int, default=[1, 4])
    args = parser.parse_args()
    if args.side < 65 or args.side > 1001 or any(t < 1 or t > 64 for t in args.threads):
        parser.error("side must be 65..1001 and workers 1..64")
    args.output = args.output.resolve()
    manifest = generate(args.output, args.side)
    if args.pcad:
        summary = {"status": "failed", "generator": manifest["generator"], "seed": manifest["seed"],
                   "executable_sha256": hashlib.sha256(args.pcad.read_bytes()).hexdigest(),
                   "ground_truth_sha256": hashlib.sha256((args.output / "ground-truth.json").read_bytes()).hexdigest()}
        try:
            summary["cases"] = validate(args, manifest)
            summary["status"] = "passed"
        except (AssertionError, RuntimeError, OSError, subprocess.TimeoutExpired) as error:
            summary["error"] = str(error)
            raise
        finally:
            save_json(args.output / "validation-summary.json", summary)
    print(f"Ground truth: {args.output / 'ground-truth.json'}")


if __name__ == "__main__":
    main()
