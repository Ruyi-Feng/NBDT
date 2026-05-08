from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional

if __package__:
    from .analyzer import BatchFrontAnalyzer, LaneConfig, PairPeriodAnalyzer, PeriodMetricConfig
else:  # pragma: no cover
    from analyzer import BatchFrontAnalyzer, LaneConfig, PairPeriodAnalyzer, PeriodMetricConfig


def _load_config(config_path: str) -> Dict[str, Any]:
    with open(config_path, "r", encoding="utf-8") as f:
        return json.load(f)


def _build_lane_config(cfg: Dict[str, Any]) -> LaneConfig:
    lane_cfg = cfg.get("lane", {})
    return LaneConfig(
        left_lane_offset=int(lane_cfg.get("left_lane_offset", -1)),
        right_lane_offset=int(lane_cfg.get("right_lane_offset", 1)),
        lane_direction_similarity_deg=float(lane_cfg.get("lane_direction_similarity_deg", 45.0)),
    )


def _build_period_config(cfg: Dict[str, Any]) -> PeriodMetricConfig:
    p_cfg = cfg.get("period_metric", {})
    madr_cfg = p_cfg.get("vehicle_class_madr")
    if isinstance(madr_cfg, dict):
        madr_cfg = {int(k): float(v) for k, v in madr_cfg.items()}
    return PeriodMetricConfig(
        tet_tit_threshold=float(p_cfg.get("tet_tit_threshold", 10.0)),
        vehicle_class_madr=madr_cfg,
    )


def _to_jsonable(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {k: _to_jsonable(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_to_jsonable(v) for v in obj]
    if isinstance(obj, float):
        if obj == float("inf"):
            return None
        return obj
    return obj


def _build_meta(config: Dict[str, Any], frame_rate: float, dt: float) -> Dict[str, Any]:
    return {
        "tracks_file": config.get("tracks_file"),
        "mode": config.get("mode", "batch"),
        "frame_rate": frame_rate,
        "time_step": dt,
        "lane": config.get("lane", {}),
        "period_metric": config.get("period_metric", {}),
    }


def _normalize_period_metrics(period: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    period = period or {}
    return {
        "TIT": period.get("TIT"),
        "TET": period.get("TET"),
        "CPI": period.get("CPI", period.get("CPI")),
    }


def _pair_frame_relation(analyzer: PairPeriodAnalyzer, frame: int, ego_id: int, target_id: int) -> str:
    for tid, relation in analyzer.get_surrounding_vehicles(frame, ego_id):
        if tid == target_id:
            return relation
    return "unknown"


def _format_pair_output(
    result: Dict[str, Any],
    meta: Dict[str, Any],
    analyzer: PairPeriodAnalyzer,
) -> Dict[str, Any]:
    ego_id = str(result["ego_id"])
    target_id = str(result["target_id"])
    ego_id_int = int(result["ego_id"])
    target_id_int = int(result["target_id"])
    instant_metrics: List[Dict[str, Any]] = []
    for row in result.get("frames", []):
        item = dict(row)
        item.pop("ego_id", None)
        item.pop("target_id", None)
        frame = int(item["frame"])
        item["relation"] = _pair_frame_relation(analyzer, frame, ego_id_int, target_id_int)
        instant_metrics.append(item)

    vehicles = {
        ego_id: {
            "targets": {
                target_id: {
                    "start_frame": result.get("start_frame"),
                    "end_frame": result.get("end_frame"),
                    "instant_metrics": instant_metrics,
                    "period_metrics": _normalize_period_metrics(result.get("period")),
                }
            }
        }
    }
    return {"meta": meta, "vehicles": vehicles}


def _format_batch_output(result: Dict[str, Any], meta: Dict[str, Any]) -> Dict[str, Any]:
    per_pair_instant: Dict[str, list] = defaultdict(list)
    for row in result.get("records", []):
        key = f"{row['ego_id']}-{row['target_id']}"
        item = dict(row)
        item.pop("ego_id", None)
        item.pop("target_id", None)
        per_pair_instant[key].append(item)

    vehicles: Dict[str, Any] = {}
    for pair_key, summary in result.get("pairs", {}).items():
        ego_id = str(summary["ego_id"])
        target_id = str(summary["target_id"])
        vehicles.setdefault(ego_id, {"targets": {}})
        vehicles[ego_id]["targets"][target_id] = {
            "start_frame": summary.get("start_frame"),
            "end_frame": summary.get("end_frame"),
            "instant_metrics": per_pair_instant.get(pair_key, []),
            "period_metrics": _normalize_period_metrics(summary.get("period")),
        }
    return {"meta": meta, "vehicles": vehicles}


def run(config: Dict[str, Any]) -> Dict[str, Any]:
    tracks_file = config["tracks_file"]
    mode = config.get("mode", "batch")
    lane_config = _build_lane_config(config)
    period_config = _build_period_config(config)
    start_frame = config.get("start_frame")
    end_frame = config.get("end_frame")

    if mode == "pair":
        pair_cfg = config.get("pair", {})
        analyzer = PairPeriodAnalyzer(
            tracks_file=tracks_file,
            lane_config=lane_config,
            period_config=period_config,
        )
        result = analyzer.analyze(
            ego_id=int(pair_cfg["ego_id"]),
            target_id=int(pair_cfg["target_id"]),
            start_frame=start_frame,
            end_frame=end_frame,
        )
        formatted = _format_pair_output(
            result,
            _build_meta(config, analyzer.data.frame_rate, analyzer.data.dt),
            analyzer,
        )
    elif mode == "batch":
        batch_cfg = config.get("batch", {})
        analyzer = BatchFrontAnalyzer(
            tracks_file=tracks_file,
            lane_config=lane_config,
            period_config=period_config,
        )
        relations = batch_cfg.get("relations", ["left_front", "front", "right_front"])
        result = analyzer.analyze(
            start_frame=start_frame,
            end_frame=end_frame,
            relations=relations,
        )
        formatted = _format_batch_output(
            result,
            _build_meta(config, analyzer.data.frame_rate, analyzer.data.dt),
        )
    else:
        raise ValueError(f"Unsupported mode: {mode}. expected: pair or batch")

    return _to_jsonable(formatted)


def main() -> None:
    parser = argparse.ArgumentParser(description="SSM calculator entry")
    parser.add_argument(
        "--config",
        type=str,
        default=str(Path(__file__).with_name("example_config.json")),
        help="Path to analyzer config json",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="",
        help="Optional output json file path",
    )
    args = parser.parse_args()

    cfg = _load_config(args.config)
    result = run(cfg)
    text = json.dumps(result, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(text, encoding="utf-8")
        print(f"Saved: {args.output}")
    else:
        print(text)


if __name__ == "__main__":
    main()
