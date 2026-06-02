from __future__ import annotations

import math
import random
from typing import Callable, Dict, List, Sequence, Tuple

import numpy as np
import pandas as pd
from deap import base, creator, tools


FPS = 30.0
IDM_DELTA = 4.0

# IDM_BOUNDS tuple order:
# (v0/maxSpeed, tau, minGap, accel, decel)
IDM_BOUNDS: List[Tuple[float, float]] = [
    (10.0, 45.0), (0.3, 4.0), (0.1, 10.0), (0.1, 5.0), (0.5, 8.0),
]

# KRAUSS_BOUNDS tuple order:
# (maxSpeed, tau, minGap, accel, decel, sigma)
KRAUSS_BOUNDS: List[Tuple[float, float]] = [
    (10.0, 45.0), (0.3, 4.0), (0.1, 10.0), (0.1, 5.0), (0.5, 8.0), (0.0, 1.0),
]

# LC2013_BOUNDS tuple order:
# (lcStrategic, lcCooperative, lcSpeedGain, lcKeepRight, lcAssertive, lcImpatience)
LC2013_BOUNDS: List[Tuple[float, float]] = [
    (0.1, 3.0), (0.1, 3.0), (0.1, 5.0), (0.0, 2.0), (0.5, 5.0), (0.0, 1.0),
]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def model_bounds(model: str) -> List[Tuple[float, float]]:
    if model == "idm":
        return IDM_BOUNDS
    if model == "krauss":
        return KRAUSS_BOUNDS
    if model == "lc2013":
        return LC2013_BOUNDS
    raise ValueError(f"Unsupported model: {model}")


def model_parameter_names(model: str) -> List[str]:
    if model == "idm":
        return ["v0", "tau", "minGap", "accel", "decel"]
    if model == "krauss":
        return ["maxSpeed", "tau", "minGap", "accel", "decel", "sigma"]
    if model == "lc2013":
        return ["lcStrategic", "lcCooperative", "lcSpeedGain",
                "lcKeepRight", "lcAssertive", "lcImpatience"]
    raise ValueError(f"Unsupported model: {model}")


def idm_params_dict(params: Sequence[float]) -> Dict[str, float]:
    return {
        "v0": params[0],
        "tau": params[1],
        "minGap": params[2],
        "accel": params[3],
        "decel": params[4],
        "delta": IDM_DELTA,
    }


def krauss_params_dict(params: Sequence[float]) -> Dict[str, float]:
    return {
        "maxSpeed": params[0],
        "tau": params[1],
        "minGap": params[2],
        "accel": params[3],
        "decel": params[4],
        "sigma": params[5],
    }


def lc2013_params_dict(params: Sequence[float]) -> Dict[str, float]:
    return {
        "lcStrategic": params[0],
        "lcCooperative": params[1],
        "lcSpeedGain": params[2],
        "lcKeepRight": params[3],
        "lcAssertive": params[4],
        "lcImpatience": params[5],
    }


def model_params_dict(model: str, params: Sequence[float]) -> Dict[str, float]:
    if model == "idm":
        return idm_params_dict(params)
    if model == "krauss":
        return krauss_params_dict(params)
    if model == "lc2013":
        return lc2013_params_dict(params)
    raise ValueError(f"Unsupported model: {model}")


def bounds_dict(bounds: Sequence[Tuple[float, float]], names: Sequence[str]) -> Dict[str, Dict[str, float]]:
    return {name: {"min": lo, "max": hi} for name, (lo, hi) in zip(names, bounds)}


def metric_name(model: str) -> str:
    if model == "idm":
        return "idm_accel_rmse"
    if model == "krauss":
        return "krauss_accel_rmse"
    if model == "lc2013":
        return "lc2013_logloss"
    raise ValueError(f"Unsupported model: {model}")


def rmse(errors: np.ndarray) -> float:
    if len(errors) == 0:
        return 1.0e9
    return float(np.sqrt(np.mean(np.square(errors))))


def evaluate_cf(params: Sequence[float], samples: pd.DataFrame, model: str, dt: float) -> float:
    if samples.empty:
        return 1.0e9
    v = samples["v"].to_numpy(dtype=float)
    leader_v = samples["leader_v"].to_numpy(dtype=float)
    gap = np.maximum(samples["gap"].to_numpy(dtype=float), 0.1)
    if model == "idm":
        v0, tau, min_gap, accel, decel = params
        dv = v - leader_v
        denom = max(2.0 * math.sqrt(max(accel * decel, 0.01)), 0.1)
        desired_gap = min_gap + np.maximum(0.0, v * tau + v * dv / denom)
        pred = accel * (1.0 - np.power(v / max(v0, 0.1), IDM_DELTA) - np.power(desired_gap / gap, 2.0))
    else:
        max_speed, tau, min_gap, accel, decel, sigma = params
        safe_v = np.maximum(0.0, leader_v + (gap - min_gap) / max(tau, 0.1))
        desired_v = np.minimum(np.minimum(max_speed, v + accel * dt), safe_v)
        braking_v = np.maximum(0.0, v - decel * dt)
        desired_v = np.maximum(braking_v, desired_v)
        pred = (1.0 - sigma) * ((desired_v - v) / dt)
    errors = np.clip(pred, -10.0, 6.0) - samples["accel"].to_numpy(dtype=float)
    return rmse(errors)


def evaluate_lc(params: Sequence[float], samples: pd.DataFrame) -> float:
    if samples.empty:
        return 1.0e9
    eps = 1.0e-9
    strategic, cooperative, speed_gain, keep_right, assertive, impatience = params
    v = samples["v"].to_numpy(dtype=float)
    cur_gap = samples["cur_gap"].to_numpy(dtype=float)
    cur_leader_v = samples["cur_leader_v"].to_numpy(dtype=float)
    adj_gap = samples["adj_gap"].to_numpy(dtype=float)
    adj_leader_v = samples["adj_leader_v"].to_numpy(dtype=float)
    cur_speed_room = np.minimum(cur_leader_v, v + cur_gap)
    adj_speed_room = np.minimum(adj_leader_v, v + adj_gap)
    gain = (adj_speed_room - cur_speed_room) / np.maximum(v + 1.0, 1.0)
    gap_penalty = np.maximum(0.0, 8.0 - adj_gap) / 8.0
    raw_score = (
        speed_gain * gain
        + strategic * (cur_gap < 12.0).astype(float)
        + cooperative * np.maximum(0.0, adj_gap - cur_gap) / 50.0
        + impatience * np.maximum(0.0, 15.0 - cur_gap) / 15.0
        - keep_right * 0.05
        - gap_penalty / max(assertive, 0.1)
    )
    y = samples["changed"].to_numpy(dtype=float)
    event_rate = float(np.clip(np.mean(y), eps, 1.0 - eps))
    # Lane changes are rare in the extracted samples. Calibrate only the intercept
    # so logloss evaluates ranking/separation without forcing an unrealistically
    # high base lane-change probability.
    intercept = math.log(event_rate / (1.0 - event_rate)) - float(np.mean(raw_score))
    score = raw_score + intercept
    p = np.clip(1.0 / (1.0 + np.exp(-np.clip(score, -50.0, 50.0))), eps, 1.0 - eps)
    return float(np.mean(-(y * np.log(p) + (1.0 - y) * np.log(1.0 - p))))


def run_ga(
    bounds: Sequence[Tuple[float, float]],
    evaluator: Callable[[List[float]], float],
    population_size: int,
    generations: int,
    mutation_rate: float,
) -> Tuple[List[float], float, List[float]]:
    if not hasattr(creator, "FitnessMin"):
        creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
    if not hasattr(creator, "Individual"):
        creator.create("Individual", list, fitness=creator.FitnessMin)

    toolbox = base.Toolbox()
    low = [lo for lo, _ in bounds]
    up = [hi for _, hi in bounds]
    span = [hi - lo for lo, hi in bounds]

    def make_individual():
        return creator.Individual(random.uniform(lo, hi) for lo, hi in bounds)

    def evaluate(individual):
        return (evaluator(list(individual)),)

    def mutate_bounded(individual):
        for i, (lo, hi) in enumerate(bounds):
            if random.random() < mutation_rate:
                individual[i] = clamp(individual[i] + random.gauss(0.0, 0.12 * span[i]), lo, hi)
        return (individual,)

    def repair(individual):
        for i in range(len(individual)):
            individual[i] = clamp(individual[i], low[i], up[i])
        return individual

    toolbox.register("individual", make_individual)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    toolbox.register("evaluate", evaluate)
    toolbox.register("mate", tools.cxUniform, indpb=0.5)
    toolbox.register("mutate", mutate_bounded)
    toolbox.register("select", tools.selTournament, tournsize=3)

    population = toolbox.population(n=population_size)
    hall_of_fame = tools.HallOfFame(1)
    history: List[float] = []

    for _ in range(generations):
        invalid = [ind for ind in population if not ind.fitness.valid]
        for ind, fitness in zip(invalid, map(toolbox.evaluate, invalid)):
            ind.fitness.values = fitness

        hall_of_fame.update(population)
        history.append(float(hall_of_fame[0].fitness.values[0]))

        elite_count = max(2, population_size // 5)
        elites = [toolbox.clone(ind) for ind in tools.selBest(population, elite_count)]
        offspring = list(map(toolbox.clone, toolbox.select(population, population_size - elite_count)))

        for child_a, child_b in zip(offspring[::2], offspring[1::2]):
            if random.random() < 0.7:
                toolbox.mate(child_a, child_b)
                repair(child_a)
                repair(child_b)
                if child_a.fitness.valid:
                    del child_a.fitness.values
                if child_b.fitness.valid:
                    del child_b.fitness.values

        for mutant in offspring:
            toolbox.mutate(mutant)
            repair(mutant)
            if mutant.fitness.valid:
                del mutant.fitness.values

        population = elites + offspring

    invalid = [ind for ind in population if not ind.fitness.valid]
    for ind, fitness in zip(invalid, map(toolbox.evaluate, invalid)):
        ind.fitness.values = fitness
    hall_of_fame.update(population)

    best = hall_of_fame[0]
    return list(best), float(best.fitness.values[0]), history
