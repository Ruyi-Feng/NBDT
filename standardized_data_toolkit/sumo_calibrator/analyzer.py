from __future__ import annotations

import glob
import os
import random
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd

if __package__:
    from .sumo_models import (
        FPS,
        bounds_dict,
        evaluate_cf,
        evaluate_lc,
        metric_name,
        model_bounds,
        model_parameter_names,
        model_params_dict,
        run_ga,
    )
else:  # pragma: no cover
    from sumo_models import (
        FPS,
        bounds_dict,
        evaluate_cf,
        evaluate_lc,
        metric_name,
        model_bounds,
        model_parameter_names,
        model_params_dict,
        run_ga,
    )


@dataclass
class SampleConfig:
    frame_step: int = 6
    max_rows_per_file: int = 0
    max_cf_samples: int = 12000
    max_lc_samples: int = 12000
    sample_seed: int = 42

    @property
    def dt(self) -> float:
        return self.frame_step / FPS


@dataclass
class GAConfig:
    population: int = 24
    generations: int = 18
    mutation_rate: float = 0.25
    seed: int = 42


@dataclass
class GridSearchConfig:
    population_candidates: List[int] = field(default_factory=lambda: [12, 24])
    generation_candidates: List[int] = field(default_factory=lambda: [10, 18])
    mutation_rate_candidates: List[float] = field(default_factory=lambda: [0.15, 0.25, 0.35])
    seed_candidates: List[int] = field(default_factory=lambda: [42])

    def combinations(self) -> List[Tuple[int, int, float, int]]:
        import itertools
        return list(itertools.product(
            self.population_candidates,
            self.generation_candidates,
            self.mutation_rate_candidates,
            self.seed_candidates,
        ))


def resolve_input_files(csv_files: Sequence[str], input_dir: str) -> List[str]:
    if csv_files:
        files = list(csv_files)
    else:
        files = sorted(glob.glob(os.path.join(input_dir, "*.csv")))
    if not files:
        raise SystemExit(f"No CSV files found. Put input data in {input_dir} or pass CSV files explicitly.")
    return files


def vehicle_lengths(df: pd.DataFrame) -> np.ndarray:
    xs = df[["boundingBox1Xm", "boundingBox2Xm", "boundingBox3Xm", "boundingBox4Xm"]].to_numpy(dtype=float)
    ys = df[["boundingBox1Ym", "boundingBox2Ym", "boundingBox3Ym", "boundingBox4Ym"]].to_numpy(dtype=float)
    edge_lengths = np.hypot(np.roll(xs, -1, axis=1) - xs, np.roll(ys, -1, axis=1) - ys)
    return edge_lengths.max(axis=1)


def load_observations(
    files: Sequence[str],
    frame_step: int,
    max_rows_per_file: int,
) -> Dict[str, Any]:
    columns = [
        "frameNum", "carId", "carCenterXm", "carCenterYm",
        "boundingBox1Xm", "boundingBox1Ym", "boundingBox2Xm", "boundingBox2Ym",
        "boundingBox3Xm", "boundingBox3Ym", "boundingBox4Xm", "boundingBox4Ym",
        "course", "speed", "laneId",
    ]
    dtype = {
        "frameNum": "int64",
        "carId": "string",
        "carCenterXm": "float64",
        "carCenterYm": "float64",
        "boundingBox1Xm": "float64",
        "boundingBox1Ym": "float64",
        "boundingBox2Xm": "float64",
        "boundingBox2Ym": "float64",
        "boundingBox3Xm": "float64",
        "boundingBox3Ym": "float64",
        "boundingBox4Xm": "float64",
        "boundingBox4Ym": "float64",
        "course": "float64",
        "speed": "float64",
        "laneId": "int64",
    }
    frames: List[pd.DataFrame] = []
    rows_read = 0

    for file_index, path in enumerate(files):
        nrows = max_rows_per_file if max_rows_per_file else None
        raw = pd.read_csv(path, usecols=columns, dtype=dtype, nrows=nrows)
        rows_read += len(raw)
        raw = raw[raw["frameNum"] % frame_step == 0].copy()
        if raw.empty:
            continue

        direction = np.where(np.cos(np.deg2rad(raw["course"].to_numpy(dtype=float))) >= 0.0, 1, -1)
        raw["file"] = file_index
        raw["frame"] = raw["frameNum"].astype("int64")
        raw["car"] = str(file_index) + ":" + raw["carId"].astype(str)
        raw["x"] = raw["carCenterXm"].astype(float)
        raw["y"] = raw["carCenterYm"].astype(float)
        raw["speed"] = raw["speed"].clip(lower=0.0)
        raw["lane"] = raw["laneId"].astype("int64")
        raw["direction"] = direction
        raw["length"] = vehicle_lengths(raw)
        raw["s"] = raw["direction"] * raw["x"]
        frames.append(raw[["file", "frame", "car", "x", "y", "s", "speed", "lane", "direction", "length"]])

    observations = pd.concat(frames, ignore_index=True) if frames else pd.DataFrame(
        columns=["file", "frame", "car", "x", "y", "s", "speed", "lane", "direction", "length"])

    return {
        "observations": observations,
        "rows_read": rows_read,
        "rows_kept": len(observations),
        "frame_min": None if observations.empty else int(observations["frame"].min()),
        "frame_max": None if observations.empty else int(observations["frame"].max()),
    }


def nearest_adjacent_ahead(
    ego: pd.DataFrame,
    vehicles: pd.DataFrame,
    lane_offset: int,
) -> pd.DataFrame:
    left = ego[["file", "frame", "direction", "lane", "s", "length"]].copy()
    left["target_lane"] = left["lane"] + lane_offset
    left["_row"] = np.arange(len(left))
    result_gap = np.full(len(left), np.nan)
    result_v = np.full(len(left), np.nan)

    right_groups: Dict[Tuple[Any, ...], Tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    for key, group in vehicles.groupby(["file", "frame", "direction", "lane"], sort=False):
        ordered = group.sort_values("s")
        right_groups[key] = (
            ordered["s"].to_numpy(dtype=float),
            ordered["speed"].to_numpy(dtype=float),
            ordered["length"].to_numpy(dtype=float),
        )

    for key, group in left.groupby(["file", "frame", "direction", "target_lane"], sort=False):
        right = right_groups.get(key)
        if right is None:
            continue
        right_s, right_v, right_length = right
        left_s = group["s"].to_numpy(dtype=float)
        pos = np.searchsorted(right_s, left_s, side="right")
        valid = pos < len(right_s)
        if not np.any(valid):
            continue
        group_rows = group["_row"].to_numpy(dtype=int)
        left_length = group["length"].to_numpy(dtype=float)
        gaps = np.full(len(group), np.nan)
        speeds = np.full(len(group), np.nan)
        valid_pos = pos[valid]
        valid_gap = right_s[valid_pos] - left_s[valid] - 0.5 * (right_length[valid_pos] + left_length[valid])
        valid_gap_mask = valid_gap > 0.5
        valid_indexes = np.flatnonzero(valid)[valid_gap_mask]
        gaps[valid_indexes] = valid_gap[valid_gap_mask]
        speeds[valid_indexes] = right_v[valid_pos[valid_gap_mask]]
        result_gap[group_rows] = gaps
        result_v[group_rows] = speeds

    return pd.DataFrame({"adj_gap": result_gap, "adj_leader_v": result_v})


def build_samples(
    data: Dict[str, Any],
    frame_step: int,
    max_cf_samples: int,
    max_lc_samples: int,
) -> Tuple[pd.DataFrame, pd.DataFrame, List[int]]:
    dt = frame_step / FPS
    obs = data["observations"].copy()
    if obs.empty:
        return pd.DataFrame(), pd.DataFrame(), []

    obs = obs.sort_values(["car", "frame"]).reset_index(drop=True)
    by_car = obs.groupby("car", sort=False)
    obs["next_frame"] = by_car["frame"].shift(-1)
    obs["next_speed"] = by_car["speed"].shift(-1)
    obs["next_lane"] = by_car["lane"].shift(-1)
    obs["next_valid"] = obs["next_frame"].eq(obs["frame"] + frame_step)

    obs = obs.sort_values(["file", "frame", "lane", "direction", "s"]).reset_index(drop=True)
    by_position = obs.groupby(["file", "frame", "lane", "direction"], sort=False)
    obs["leader_s"] = by_position["s"].shift(-1)
    obs["leader_speed"] = by_position["speed"].shift(-1)
    obs["leader_length"] = by_position["length"].shift(-1)
    obs["leader_gap"] = obs["leader_s"] - obs["s"] - 0.5 * (obs["leader_length"] + obs["length"])

    cf = obs.loc[
        obs["next_valid"]
        & obs["leader_gap"].gt(0.5)
        & obs["leader_gap"].le(150.0),
        ["speed", "leader_speed", "leader_gap", "next_speed"],
    ].copy()
    cf["accel"] = (cf["next_speed"] - cf["speed"]) / dt
    cf = cf.loc[cf["accel"].between(-8.0, 5.0), ["speed", "leader_speed", "leader_gap", "accel"]]
    cf = cf.rename(columns={"speed": "v", "leader_speed": "leader_v", "leader_gap": "gap"})

    lc_base = obs.loc[obs["next_valid"] & obs["leader_gap"].notna()].copy()
    lc_base["changed"] = (lc_base["next_lane"] != lc_base["lane"]).astype(int)
    adj_down = nearest_adjacent_ahead(lc_base, obs, -1)
    adj_up = nearest_adjacent_ahead(lc_base, obs, 1)
    lc = lc_base[["speed", "leader_gap", "leader_speed", "changed"]].rename(
        columns={"speed": "v", "leader_gap": "cur_gap", "leader_speed": "cur_leader_v"})
    lc["down_gap"] = adj_down["adj_gap"].to_numpy()
    lc["down_v"] = adj_down["adj_leader_v"].to_numpy()
    lc["up_gap"] = adj_up["adj_gap"].to_numpy()
    lc["up_v"] = adj_up["adj_leader_v"].to_numpy()
    use_up = lc["down_v"].isna() | (lc["up_v"].fillna(-np.inf) > lc["down_v"].fillna(-np.inf))
    lc["adj_gap"] = np.where(use_up, lc["up_gap"], lc["down_gap"])
    lc["adj_leader_v"] = np.where(use_up, lc["up_v"], lc["down_v"])
    lc = lc.dropna(subset=["cur_gap", "cur_leader_v", "adj_gap", "adj_leader_v"])
    lc = lc[["v", "cur_gap", "cur_leader_v", "adj_gap", "adj_leader_v", "changed"]]

    if max_cf_samples and len(cf) > max_cf_samples:
        cf = cf.sample(n=max_cf_samples, random_state=random.randint(0, 2 ** 32 - 1))
    else:
        cf = cf.sample(frac=1.0, random_state=random.randint(0, 2 ** 32 - 1))
    if max_lc_samples and len(lc) > max_lc_samples:
        lc = lc.sample(n=max_lc_samples, random_state=random.randint(0, 2 ** 32 - 1))
    else:
        lc = lc.sample(frac=1.0, random_state=random.randint(0, 2 ** 32 - 1))

    lanes = sorted(obs["lane"].dropna().astype(int).unique().tolist())
    return cf.reset_index(drop=True), lc.reset_index(drop=True), lanes


def baseline_metrics(cf_samples: pd.DataFrame, lc_samples: pd.DataFrame) -> Dict[str, float]:
    metrics: Dict[str, float] = {}
    if not cf_samples.empty:
        accel = cf_samples["accel"].to_numpy(dtype=float)
        metrics["zero_accel_rmse"] = float(np.sqrt(np.mean(accel * accel)))
        metrics["mean_accel_rmse"] = float(np.sqrt(np.mean(np.square(accel - np.mean(accel)))))
    if not lc_samples.empty:
        eps = 1.0e-9
        y = lc_samples["changed"].to_numpy(dtype=float)
        event_rate = float(np.clip(np.mean(y), eps, 1.0 - eps))
        metrics["lane_change_rate"] = event_rate
        metrics["constant_rate_logloss"] = float(
            -np.mean(y * np.log(event_rate) + (1.0 - y) * np.log(1.0 - event_rate)))
    return metrics


def make_evaluator(
    model: str,
    cf_samples: pd.DataFrame,
    lc_samples: pd.DataFrame,
    dt: float,
) -> Callable[[List[float]], float]:
    if model in {"idm", "krauss"}:
        if len(cf_samples) < 100:
            raise SystemExit("Not enough car-following samples were extracted.")
        return lambda p: evaluate_cf(p, cf_samples, model, dt)
    if model == "lc2013":
        if len(lc_samples) < 100:
            raise SystemExit("Not enough lane-change samples were extracted.")
        return lambda p: evaluate_lc(p, lc_samples)
    raise ValueError(f"Unsupported model: {model}")


def selected_models(model: str) -> List[str]:
    if model == "all":
        return ["idm", "krauss", "lc2013"]
    if model in {"idm", "krauss", "lc2013"}:
        return [model]
    raise ValueError(f"Unsupported model: {model}")


class Calibrator:
    """Run a single GA pass per selected model on a fixed sample set."""

    def __init__(
        self,
        input_files: Sequence[str],
        sample_config: SampleConfig,
        ga_config: GAConfig,
    ) -> None:
        self.input_files = list(input_files)
        self.sample_config = sample_config
        self.ga_config = ga_config
        random.seed(sample_config.sample_seed)
        self.data = load_observations(
            self.input_files,
            sample_config.frame_step,
            sample_config.max_rows_per_file,
        )
        self.cf_samples, self.lc_samples, self.lanes = build_samples(
            self.data,
            sample_config.frame_step,
            sample_config.max_cf_samples,
            sample_config.max_lc_samples,
        )

    def calibrate(self, model: str) -> Dict[str, Any]:
        t0 = time.time()
        random.seed(self.ga_config.seed)
        evaluator = make_evaluator(model, self.cf_samples, self.lc_samples, self.sample_config.dt)
        bounds = model_bounds(model)
        params, score, history = run_ga(
            bounds,
            evaluator,
            self.ga_config.population,
            self.ga_config.generations,
            self.ga_config.mutation_rate,
        )
        metric = metric_name(model)
        return {
            "model": model,
            "objective": metric,
            "score": score,
            "parameters": model_params_dict(model, params),
            "raw_params": params,
            "fitness_history": history,
            "ga_config": {
                "population": self.ga_config.population,
                "generations": self.ga_config.generations,
                "mutation_rate": self.ga_config.mutation_rate,
                "seed": self.ga_config.seed,
            },
            "bounds": bounds_dict(bounds, model_parameter_names(model)),
            "runtime_seconds": time.time() - t0,
        }

    def calibrate_many(self, models: Iterable[str]) -> Dict[str, Dict[str, Any]]:
        return {model: self.calibrate(model) for model in models}

    def data_summary(self) -> Dict[str, Any]:
        return {
            "input_files": self.input_files,
            "frame_step": self.sample_config.frame_step,
            "dt_seconds": self.sample_config.dt,
            "max_rows_per_file": self.sample_config.max_rows_per_file,
            "max_cf_samples": self.sample_config.max_cf_samples,
            "max_lc_samples": self.sample_config.max_lc_samples,
            "sample_seed": self.sample_config.sample_seed,
            "rows_read": self.data["rows_read"],
            "sampled_rows": self.data["rows_kept"],
            "frame_min": self.data["frame_min"],
            "frame_max": self.data["frame_max"],
            "lanes": self.lanes,
            "car_following_samples": len(self.cf_samples),
            "lane_change_samples": len(self.lc_samples),
        }


class GridSearchOptimizer:
    """Grid-search GA training parameters and keep the best trial."""

    def __init__(
        self,
        input_files: Sequence[str],
        sample_config: SampleConfig,
        grid_config: GridSearchConfig,
    ) -> None:
        self.input_files = list(input_files)
        self.sample_config = sample_config
        self.grid_config = grid_config
        random.seed(sample_config.sample_seed)
        self.data = load_observations(
            self.input_files,
            sample_config.frame_step,
            sample_config.max_rows_per_file,
        )
        self.cf_samples, self.lc_samples, self.lanes = build_samples(
            self.data,
            sample_config.frame_step,
            sample_config.max_cf_samples,
            sample_config.max_lc_samples,
        )

    def optimize(self, model: str, progress: Optional[Callable[[str], None]] = None) -> Dict[str, Any]:
        t0 = time.time()
        bounds = model_bounds(model)
        metric = metric_name(model)
        evaluator = make_evaluator(model, self.cf_samples, self.lc_samples, self.sample_config.dt)

        best: Optional[Dict[str, Any]] = None
        trials: List[Dict[str, Any]] = []
        combinations = self.grid_config.combinations()

        for index, (population, generation_count, mutation_rate, seed) in enumerate(combinations, start=1):
            random.seed(seed)
            params, score, history = run_ga(
                bounds, evaluator, population, generation_count, mutation_rate,
            )
            trial = {
                "trial": index,
                "population": population,
                "generations": generation_count,
                "mutation_rate": mutation_rate,
                "seed": seed,
                metric: score,
                "fitness_history": history,
                "parameters": model_params_dict(model, params),
            }
            trials.append(trial)
            if progress is not None:
                progress(
                    f"Trial {index}/{len(combinations)}: population={population} "
                    f"generations={generation_count} mutation_rate={mutation_rate:.4f} "
                    f"seed={seed} {metric}={score:.6f}"
                )
            if best is None or score < best["score"]:
                best = {
                    "score": score,
                    "params": params,
                    "population": population,
                    "generations": generation_count,
                    "mutation_rate": mutation_rate,
                    "seed": seed,
                    "fitness_history": history,
                    "trial": index,
                }

        assert best is not None
        best_model_params = {
            "model": model,
            "objective": metric,
            "score": best["score"],
            "parameters": model_params_dict(model, best["params"]),
        }
        training_params = {
            "model": model,
            "objective": metric,
            "best_score": best["score"],
            "best_trial": best["trial"],
            "best_training_params": {
                "population": best["population"],
                "generations": best["generations"],
                "mutation_rate": best["mutation_rate"],
                "seed": best["seed"],
                "fitness_history": best["fitness_history"],
            },
            "search_space": {
                "population_candidates": self.grid_config.population_candidates,
                "generation_candidates": self.grid_config.generation_candidates,
                "mutation_rate_candidates": self.grid_config.mutation_rate_candidates,
                "seed_candidates": self.grid_config.seed_candidates,
            },
            "model_parameter_bounds": bounds_dict(bounds, model_parameter_names(model)),
            "data_params": {
                "input_files": self.input_files,
                "frame_step": self.sample_config.frame_step,
                "dt_seconds": self.sample_config.dt,
                "max_rows_per_file": self.sample_config.max_rows_per_file,
                "max_cf_samples": self.sample_config.max_cf_samples,
                "max_lc_samples": self.sample_config.max_lc_samples,
                "sample_seed": self.sample_config.sample_seed,
                "rows_read": self.data["rows_read"],
                "sampled_rows": self.data["rows_kept"],
                "frame_min": self.data["frame_min"],
                "frame_max": self.data["frame_max"],
                "lanes": self.lanes,
                "car_following_samples": len(self.cf_samples),
                "lane_change_samples": len(self.lc_samples),
            },
            "baseline_metrics": baseline_metrics(self.cf_samples, self.lc_samples),
            "trials": trials,
            "runtime_seconds": time.time() - t0,
        }
        return {"best_model_params": best_model_params, "training_params": training_params}
