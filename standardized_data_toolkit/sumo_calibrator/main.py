from __future__ import annotations

import argparse
import json
import os
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

if __package__:
    from .analyzer import (
        Calibrator,
        GAConfig,
        GridSearchConfig,
        GridSearchOptimizer,
        SampleConfig,
        baseline_metrics,
        resolve_input_files,
        selected_models,
    )
    from .sumo_models import IDM_DELTA, metric_name
else:  # pragma: no cover
    from analyzer import (
        Calibrator,
        GAConfig,
        GridSearchConfig,
        GridSearchOptimizer,
        SampleConfig,
        baseline_metrics,
        resolve_input_files,
        selected_models,
    )
    from sumo_models import IDM_DELTA, metric_name


def _load_config(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _ensure_dir(path: str) -> None:
    if path and not os.path.isdir(path):
        os.makedirs(path, exist_ok=True)


def _write_json(path: str, data: Any) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False, sort_keys=True)


def _build_sample_config(cfg: Mapping[str, Any]) -> SampleConfig:
    s = cfg.get("sample", {})
    return SampleConfig(
        frame_step=int(s.get("frame_step", 6)),
        max_rows_per_file=int(s.get("max_rows_per_file", 0)),
        max_cf_samples=int(s.get("max_cf_samples", 12000)),
        max_lc_samples=int(s.get("max_lc_samples", 12000)),
        sample_seed=int(s.get("sample_seed", 42)),
    )


def _build_ga_config(cfg: Mapping[str, Any]) -> GAConfig:
    g = cfg.get("ga", {})
    return GAConfig(
        population=int(g.get("population", 24)),
        generations=int(g.get("generations", 18)),
        mutation_rate=float(g.get("mutation_rate", 0.25)),
        seed=int(g.get("seed", 42)),
    )


def _build_grid_config(cfg: Mapping[str, Any]) -> GridSearchConfig:
    g = cfg.get("grid_search", {})
    return GridSearchConfig(
        population_candidates=[int(v) for v in g.get("population_candidates", [12, 24])],
        generation_candidates=[int(v) for v in g.get("generation_candidates", [10, 18])],
        mutation_rate_candidates=[float(v) for v in g.get("mutation_rate_candidates", [0.15, 0.25, 0.35])],
        seed_candidates=[int(v) for v in g.get("seed_candidates", [42])],
    )


def _lc_attrs(lc_params: Optional[Dict[str, float]]) -> Dict[str, str]:
    if lc_params is None:
        return {}
    return {
        "laneChangeModel": "LC2013",
        "lcStrategic": "%.3f" % lc_params["lcStrategic"],
        "lcCooperative": "%.3f" % lc_params["lcCooperative"],
        "lcSpeedGain": "%.3f" % lc_params["lcSpeedGain"],
        "lcKeepRight": "%.3f" % lc_params["lcKeepRight"],
        "lcAssertive": "%.3f" % lc_params["lcAssertive"],
        "lcImpatience": "%.3f" % lc_params["lcImpatience"],
    }


def _idm_vtype_attrs(idm: Dict[str, float], vtype_id: str = "ga_idm") -> Dict[str, str]:
    return {
        "id": vtype_id,
        "carFollowModel": "IDM",
        "maxSpeed": "%.3f" % idm["v0"],
        "tau": "%.3f" % idm["tau"],
        "minGap": "%.3f" % idm["minGap"],
        "accel": "%.3f" % idm["accel"],
        "decel": "%.3f" % idm["decel"],
        "delta": "%.3f" % idm.get("delta", IDM_DELTA),
    }


def _krauss_vtype_attrs(krauss: Dict[str, float], vtype_id: str = "ga_krauss") -> Dict[str, str]:
    return {
        "id": vtype_id,
        "carFollowModel": "Krauss",
        "maxSpeed": "%.3f" % krauss["maxSpeed"],
        "tau": "%.3f" % krauss["tau"],
        "minGap": "%.3f" % krauss["minGap"],
        "accel": "%.3f" % krauss["accel"],
        "decel": "%.3f" % krauss["decel"],
        "sigma": "%.3f" % krauss["sigma"],
    }


def write_vtypes_xml(
    path: str,
    idm: Optional[Dict[str, float]] = None,
    krauss: Optional[Dict[str, float]] = None,
    lc: Optional[Dict[str, float]] = None,
) -> None:
    root = ET.Element("additional")
    lc_attrs = _lc_attrs(lc)
    if idm is not None:
        attrs = _idm_vtype_attrs(idm)
        attrs.update(lc_attrs)
        ET.SubElement(root, "vType", attrs)
    if krauss is not None:
        attrs = _krauss_vtype_attrs(krauss)
        attrs.update(lc_attrs)
        ET.SubElement(root, "vType", attrs)
    if lc is not None and idm is None and krauss is None:
        attrs = {"id": "ga_lc2013"}
        attrs.update(lc_attrs)
        ET.SubElement(root, "vType", attrs)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def write_best_model_xml(path: str, model: str, params: Dict[str, float]) -> None:
    root = ET.Element("additional")
    if model == "idm":
        ET.SubElement(root, "vType", _idm_vtype_attrs(params, vtype_id="best_idm"))
    elif model == "krauss":
        ET.SubElement(root, "vType", _krauss_vtype_attrs(params, vtype_id="best_krauss"))
    elif model == "lc2013":
        attrs = {"id": "best_lc2013"}
        attrs.update(_lc_attrs(params))
        ET.SubElement(root, "vType", attrs)
    else:
        raise ValueError(f"Unsupported model: {model}")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def _run_calibrate(config: Mapping[str, Any], out_dir: str) -> Dict[str, Any]:
    t0 = time.time()
    model_choice = config.get("model", "all")
    models = selected_models(model_choice)
    sample_cfg = _build_sample_config(config)
    ga_cfg = _build_ga_config(config)
    input_files = resolve_input_files(
        config.get("csv_files", []) or [],
        config.get("input_dir", "input"),
    )
    calibrator = Calibrator(input_files, sample_cfg, ga_cfg)
    results = calibrator.calibrate_many(models)

    data_summary = calibrator.data_summary()
    summary: Dict[str, Any] = {
        "model": model_choice,
        "models": models,
        "runtime_seconds": time.time() - t0,
    }
    summary.update(data_summary)

    params_payload: Dict[str, Any] = {"summary": summary}
    for model, result in results.items():
        params_payload[model] = result["parameters"]
        summary[metric_name(model)] = result["score"]

    _ensure_dir(out_dir)
    _write_json(os.path.join(out_dir, "calibrated_params.json"), params_payload)

    write_vtypes_xml(
        os.path.join(out_dir, "sumo_vtypes.add.xml"),
        idm=results.get("idm", {}).get("parameters") if "idm" in results else None,
        krauss=results.get("krauss", {}).get("parameters") if "krauss" in results else None,
        lc=results.get("lc2013", {}).get("parameters") if "lc2013" in results else None,
    )

    summary_lines = [f"{k}: {summary[k]}" for k in sorted(summary)]
    Path(os.path.join(out_dir, "samples_summary.txt")).write_text(
        "\n".join(summary_lines) + "\n", encoding="utf-8",
    )

    return {"summary": summary, "results": {m: r for m, r in results.items()}}


def _run_optimize(config: Mapping[str, Any], out_dir: str) -> Dict[str, Any]:
    model = config.get("model")
    if model not in {"idm", "krauss", "lc2013"}:
        raise ValueError("optimize mode requires model in {idm, krauss, lc2013}")
    sample_cfg = _build_sample_config(config)
    grid_cfg = _build_grid_config(config)
    input_files = resolve_input_files(
        config.get("csv_files", []) or [],
        config.get("input_dir", "input"),
    )
    optimizer = GridSearchOptimizer(input_files, sample_cfg, grid_cfg)
    result = optimizer.optimize(model, progress=print)

    _ensure_dir(out_dir)
    best_path = os.path.join(out_dir, "best_model_params.json")
    training_path = os.path.join(out_dir, "training_params.json")
    xml_path = os.path.join(out_dir, "best_model_params.add.xml")
    _write_json(best_path, result["best_model_params"])
    _write_json(training_path, result["training_params"])
    write_best_model_xml(xml_path, model, result["best_model_params"]["parameters"])

    print(f"{model} best {result['best_model_params']['objective']}: "
          f"{result['best_model_params']['score']:.6f}")
    return result


def run(config: Mapping[str, Any], out_dir: str) -> Dict[str, Any]:
    mode = config.get("mode", "calibrate")
    if mode == "calibrate":
        return _run_calibrate(config, out_dir)
    if mode == "optimize":
        return _run_optimize(config, out_dir)
    raise ValueError(f"Unsupported mode: {mode}. expected: calibrate or optimize")


def main() -> None:
    parser = argparse.ArgumentParser(description="SUMO calibrator entry")
    parser.add_argument(
        "--config",
        type=str,
        default=str(Path(__file__).with_name("example_config.json")),
        help="Path to calibrator config json",
    )
    parser.add_argument(
        "--out",
        type=str,
        default="calibration_output",
        help="Output directory",
    )
    args = parser.parse_args()

    cfg = _load_config(args.config)
    run(cfg, args.out)
    print(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
