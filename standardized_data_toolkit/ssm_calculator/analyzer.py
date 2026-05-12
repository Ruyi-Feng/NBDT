from __future__ import annotations

import bisect
from collections import defaultdict
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import pandas as pd

if __package__:
    from .ssm import InstantSSMCalculator, PeriodSSMCalculator, VehicleState
else:  # pragma: no cover
    from ssm import InstantSSMCalculator, PeriodSSMCalculator, VehicleState


@dataclass
class LaneConfig:
    left_lane_offset: int = -1
    right_lane_offset: int = 1
    lane_direction_similarity_deg: float = 45.0


@dataclass
class PeriodMetricConfig:
    tet_tit_threshold: float = 10.0
    vehicle_class_madr: Optional[Dict[int, float]] = None

    def with_defaults(self) -> "PeriodMetricConfig":
        if self.vehicle_class_madr is None:
            self.vehicle_class_madr = {
                0: 3.0,  # car
                1: 3.0,  # taxi
                2: 2.0,  # bus
                3: 2.0,  # truck
                4: 4.0,  # motorcycle
                5: 10.0,  # pedestrian
                -1: 3.0,  # unknown
            }
        return self


class TrackDataStore:
    """
    标准轨迹 CSV 预处理:
    - 构建 frame -> car_id -> VehicleState
    - 构建 frame -> lane_id -> 按投影距离排序车辆列表
    - 构建 frame -> lane_id -> lane direction(rad)
    """

    def __init__(self, tracks_file: str):
        self.tracks_file = tracks_file
        self.df_tracks = self._load_tracks(tracks_file)
        self.frames = sorted(self.df_tracks["frameNum"].dropna().unique().tolist())
        self.frame_rate = self._infer_frame_rate(self.df_tracks)
        self.dt = 1.0 / self.frame_rate if self.frame_rate > 0 else 0.04

        self.frame_vehicles: Dict[int, Dict[int, VehicleState]] = {}
        self.frame_lane_vehicles: Dict[int, Dict[int, List[int]]] = {}
        self.frame_lane_direction: Dict[int, Dict[int, float]] = {}
        # frame -> lane_id -> {car_id: index in lane_list} for O(1) lookup in
        # ``BaseSSMAnalyzer.get_surrounding_vehicles``.
        self.frame_lane_index: Dict[int, Dict[int, Dict[int, int]]] = {}

        self._preprocess()

    @staticmethod
    def _load_tracks(tracks_file: str) -> pd.DataFrame:
        df = pd.read_csv(tracks_file)
        numeric_cols = [
            "frameNum",
            "carId",
            "carCenterXm",
            "carCenterYm",
            "boundingBox1Xm",
            "boundingBox1Ym",
            "boundingBox2Xm",
            "boundingBox2Ym",
            "boundingBox3Xm",
            "boundingBox3Ym",
            "boundingBox4Xm",
            "boundingBox4Ym",
            "heading",
            "speed",
            "objClass",
            "laneId",
        ]
        for col in numeric_cols:
            if col in df.columns:
                df[col] = pd.to_numeric(df[col], errors="coerce")
        return df

    @staticmethod
    def _infer_frame_rate(df: pd.DataFrame) -> float:
        if "timestamp" in df.columns:
            ts = pd.to_numeric(df["timestamp"], errors="coerce").dropna().sort_values().unique()
            if len(ts) >= 2:
                dt = np.median(np.diff(ts))
                if dt > 0:
                    return float(1.0 / dt)
        if "time" in df.columns:
            ts = pd.to_numeric(df["time"], errors="coerce").dropna().sort_values().unique()
            if len(ts) >= 2:
                dt = np.median(np.diff(ts))
                if dt > 0:
                    return float(1.0 / dt)
        return 25.0

    @staticmethod
    def _vehicle_dimensions(df: pd.DataFrame) -> Tuple[pd.Series, pd.Series]:
        length = 0.5 * (
            np.sqrt((df["boundingBox1Xm"] - df["boundingBox2Xm"]) ** 2 + (df["boundingBox1Ym"] - df["boundingBox2Ym"]) ** 2)
            + np.sqrt((df["boundingBox3Xm"] - df["boundingBox4Xm"]) ** 2 + (df["boundingBox3Ym"] - df["boundingBox4Ym"]) ** 2)
        )
        width = 0.5 * (
            np.sqrt((df["boundingBox1Xm"] - df["boundingBox4Xm"]) ** 2 + (df["boundingBox1Ym"] - df["boundingBox4Ym"]) ** 2)
            + np.sqrt((df["boundingBox2Xm"] - df["boundingBox3Xm"]) ** 2 + (df["boundingBox2Ym"] - df["boundingBox3Ym"]) ** 2)
        )
        return length, width

    def _preprocess(self) -> None:
        df = self.df_tracks.copy()
        df["x"] = df["carCenterXm"]
        df["y"] = df["carCenterYm"]
        length, width = self._vehicle_dimensions(df)
        df["L"] = length
        df["W"] = width

        heading_rad = np.radians(df["heading"])
        df["vx"] = df["speed"] * np.cos(heading_rad)
        df["vy"] = df["speed"] * np.sin(heading_rad)

        df = df.sort_values(["carId", "frameNum"]).reset_index(drop=True)
        df["frame_diff"] = df.groupby("carId")["frameNum"].diff()
        df["speed_diff"] = df.groupby("carId")["speed"].diff()
        df["heading_diff"] = df.groupby("carId")["heading"].diff()

        heading_diff = df["heading_diff"].copy()
        heading_diff[heading_diff > 180] -= 360
        heading_diff[heading_diff < -180] += 360
        df["a"] = np.where(df["frame_diff"] == 1, df["speed_diff"] / self.dt, 0.0)
        df["angle_speed"] = np.where(df["frame_diff"] == 1, heading_diff / self.dt, 0.0)
        df.drop(["frame_diff", "speed_diff", "heading_diff"], axis=1, inplace=True)

        def lane_direction(group: pd.DataFrame) -> float:
            headings = group["heading"].dropna().values
            if len(headings) < 2:
                return np.nan
            sum_cos = np.sum(np.cos(np.radians(headings)))
            sum_sin = np.sum(np.sin(np.radians(headings)))
            return float(np.arctan2(sum_sin, sum_cos))

        lane_dir_df = df.groupby(["frameNum", "laneId"]).apply(lane_direction).reset_index()
        lane_dir_df.columns = ["frameNum", "laneId", "lane_dir_rad"]
        df = df.merge(lane_dir_df, on=["frameNum", "laneId"], how="left")

        mask = df["lane_dir_rad"].notna()
        cos_v = np.cos(df.loc[mask, "lane_dir_rad"])
        sin_v = np.sin(df.loc[mask, "lane_dir_rad"])
        df.loc[mask, "proj_dist"] = df.loc[mask, "x"] * cos_v + df.loc[mask, "y"] * sin_v
        df.loc[mask, "proj_speed"] = df.loc[mask, "vx"] * cos_v + df.loc[mask, "vy"] * sin_v
        delta = np.radians(df.loc[mask, "heading"]) - df.loc[mask, "lane_dir_rad"]
        df.loc[mask, "proj_acc"] = df.loc[mask, "a"] * np.cos(delta)

        self._build_frame_structures(df)

    def _build_frame_structures(self, df: pd.DataFrame) -> None:
        """Vectorized replacement for the original per-frame iterrows loop.

        Builds the same ``frame_vehicles``, ``frame_lane_vehicles`` and
        ``frame_lane_direction`` dicts plus an additional ``frame_lane_index``
        lookup. Behavior is preserved bit-for-bit relative to the original
        implementation.
        """
        # ----- vectorized ordered_bbox per row (order_rect_points) -----
        bb = np.stack([
            df["boundingBox1Xm"].to_numpy(dtype=float),
            df["boundingBox1Ym"].to_numpy(dtype=float),
            df["boundingBox2Xm"].to_numpy(dtype=float),
            df["boundingBox2Ym"].to_numpy(dtype=float),
            df["boundingBox3Xm"].to_numpy(dtype=float),
            df["boundingBox3Ym"].to_numpy(dtype=float),
            df["boundingBox4Xm"].to_numpy(dtype=float),
            df["boundingBox4Ym"].to_numpy(dtype=float),
        ], axis=1).reshape(-1, 4, 2)
        centroid = bb.mean(axis=1, keepdims=True)
        deltas = bb - centroid
        angles = np.arctan2(deltas[..., 1], deltas[..., 0])
        order = np.argsort(angles, axis=1, kind="stable")
        ordered = np.take_along_axis(bb, order[..., None], axis=1)
        a = ordered[:, 0]
        b = ordered[:, 1]
        c = ordered[:, 2]
        cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
        flip = cross > 0
        if flip.any():
            ordered[flip] = ordered[flip][:, [0, 3, 2, 1], :]

        # Sort once globally: (frameNum, laneId, proj_dist). Within ties the sort
        # is stable, so it preserves the original (carId, frameNum) order — which
        # matches what the legacy code produced for lanes without any proj_dist.
        df = df.copy()
        df["__row__"] = np.arange(len(df), dtype=np.int64)
        df.sort_values(["frameNum", "laneId", "proj_dist"], inplace=True,
                       kind="stable", na_position="last")
        row_idx = df["__row__"].to_numpy()

        ordered_sorted = ordered[row_idx]

        # Pre-pull numpy arrays so .item() lookups inside the loop are cheap.
        col_frame = df["frameNum"].to_numpy(dtype=np.int64)
        col_lane_raw = df["laneId"].to_numpy()
        col_lane_is_na = pd.isna(col_lane_raw)
        col_lane = np.where(col_lane_is_na, -1, col_lane_raw).astype(np.int64)
        col_carid = df["carId"].to_numpy(dtype=np.int64)
        col_x = df["x"].to_numpy(dtype=float)
        col_y = df["y"].to_numpy(dtype=float)
        col_vx = df["vx"].to_numpy(dtype=float)
        col_vy = df["vy"].to_numpy(dtype=float)
        col_speed = df["speed"].to_numpy(dtype=float)
        col_heading = df["heading"].to_numpy(dtype=float)
        col_a = df["a"].to_numpy(dtype=float)
        col_angle_speed = df["angle_speed"].to_numpy(dtype=float)
        col_L = df["L"].to_numpy(dtype=float)
        col_W = df["W"].to_numpy(dtype=float)
        col_obj_raw = df["objClass"].to_numpy() if "objClass" in df.columns else np.full(len(df), -1.0)
        col_obj_is_na = pd.isna(col_obj_raw)
        col_obj = np.where(col_obj_is_na, -1, col_obj_raw).astype(np.int64)
        col_bb1x = df["boundingBox1Xm"].to_numpy(dtype=float)
        col_bb1y = df["boundingBox1Ym"].to_numpy(dtype=float)
        col_bb2x = df["boundingBox2Xm"].to_numpy(dtype=float)
        col_bb2y = df["boundingBox2Ym"].to_numpy(dtype=float)
        col_bb3x = df["boundingBox3Xm"].to_numpy(dtype=float)
        col_bb3y = df["boundingBox3Ym"].to_numpy(dtype=float)
        col_bb4x = df["boundingBox4Xm"].to_numpy(dtype=float)
        col_bb4y = df["boundingBox4Ym"].to_numpy(dtype=float)
        col_pd = df["proj_dist"].to_numpy(dtype=float) if "proj_dist" in df.columns else np.full(len(df), np.nan)
        col_ps = df["proj_speed"].to_numpy(dtype=float) if "proj_speed" in df.columns else np.full(len(df), np.nan)
        col_pa = df["proj_acc"].to_numpy(dtype=float) if "proj_acc" in df.columns else np.full(len(df), np.nan)
        col_ldir = df["lane_dir_rad"].to_numpy(dtype=float)

        pd_isnan = np.isnan(col_pd)
        ps_isnan = np.isnan(col_ps)
        pa_isnan = np.isnan(col_pa)
        ldir_isnan = np.isnan(col_ldir)

        n = len(df)
        # Iterate once over rows in (frame, lane, proj_dist) order.
        # The per-(frame, lane) lane_list is the sequence of car_ids in that order.
        prev_frame = -(1 << 62)
        prev_lane = None
        veh_dict: Dict[int, VehicleState] = {}
        lane_dict: Dict[int, List[int]] = {}
        lane_dir_dict: Dict[int, float] = {}
        lane_index_dict: Dict[int, Dict[int, int]] = {}
        cur_lane_list: Optional[List[int]] = None
        cur_lane_index: Optional[Dict[int, int]] = None

        for i in range(n):
            fr = int(col_frame[i])
            ln = int(col_lane[i])

            if fr != prev_frame:
                if prev_frame != -(1 << 62):
                    self.frame_vehicles[prev_frame] = veh_dict
                    self.frame_lane_vehicles[prev_frame] = lane_dict
                    self.frame_lane_direction[prev_frame] = lane_dir_dict
                    self.frame_lane_index[prev_frame] = lane_index_dict
                prev_frame = fr
                prev_lane = None
                veh_dict = {}
                lane_dict = {}
                lane_dir_dict = {}
                lane_index_dict = {}
                cur_lane_list = None
                cur_lane_index = None

            if ln != prev_lane:
                prev_lane = ln
                cur_lane_list = []
                cur_lane_index = {}
                lane_dict[ln] = cur_lane_list
                lane_index_dict[ln] = cur_lane_index
                if not ldir_isnan[i]:
                    lane_dir_dict[ln] = float(col_ldir[i])

            cid = int(col_carid[i])
            cur_lane_index[cid] = len(cur_lane_list)
            cur_lane_list.append(cid)

            ordered_list = [
                (float(ordered_sorted[i, 0, 0]), float(ordered_sorted[i, 0, 1])),
                (float(ordered_sorted[i, 1, 0]), float(ordered_sorted[i, 1, 1])),
                (float(ordered_sorted[i, 2, 0]), float(ordered_sorted[i, 2, 1])),
                (float(ordered_sorted[i, 3, 0]), float(ordered_sorted[i, 3, 1])),
            ]

            state = VehicleState(
                x=float(col_x[i]),
                y=float(col_y[i]),
                vx=float(col_vx[i]),
                vy=float(col_vy[i]),
                speed=float(col_speed[i]),
                heading=float(col_heading[i]),
                acceleration=float(col_a[i]),
                angle_speed=float(col_angle_speed[i]),
                length=float(col_L[i]),
                width=float(col_W[i]),
                lane_id=ln,
                obj_class=int(col_obj[i]),
                bbox=[
                    (float(col_bb1x[i]), float(col_bb1y[i])),
                    (float(col_bb2x[i]), float(col_bb2y[i])),
                    (float(col_bb3x[i]), float(col_bb3y[i])),
                    (float(col_bb4x[i]), float(col_bb4y[i])),
                ],
                proj_dist=None if pd_isnan[i] else float(col_pd[i]),
                proj_speed=None if ps_isnan[i] else float(col_ps[i]),
                proj_acc=None if pa_isnan[i] else float(col_pa[i]),
                ordered_bbox=ordered_list,
            )
            veh_dict[cid] = state

        # flush last frame
        if prev_frame != -(1 << 62):
            self.frame_vehicles[prev_frame] = veh_dict
            self.frame_lane_vehicles[prev_frame] = lane_dict
            self.frame_lane_direction[prev_frame] = lane_dir_dict
            self.frame_lane_index[prev_frame] = lane_index_dict

    def get_vehicle(self, frame: int, car_id: int) -> Optional[VehicleState]:
        return self.frame_vehicles.get(frame, {}).get(car_id)


class BaseSSMAnalyzer:
    def __init__(
        self,
        tracks_file: str,
        lane_config: Optional[LaneConfig] = None,
        period_config: Optional[PeriodMetricConfig] = None,
    ):
        self.data = TrackDataStore(tracks_file)
        self.instant_calculator = InstantSSMCalculator()
        self.period_calculator = PeriodSSMCalculator()
        self.lane_config = lane_config or LaneConfig()
        self.period_config = (period_config or PeriodMetricConfig()).with_defaults()

    @staticmethod
    def _angle_diff_deg(a1: float, a2: float) -> float:
        diff = abs(a1 - a2) % 360
        return 360 - diff if diff > 180 else diff

    def get_surrounding_vehicles(self, frame: int, ego_id: int) -> List[Tuple[int, str]]:
        frame_vehicles = self.data.frame_vehicles.get(frame, {})
        if ego_id not in frame_vehicles:
            return []
        ego = frame_vehicles[ego_id]
        lane_map = self.data.frame_lane_vehicles.get(frame, {})
        lane_dir_map = self.data.frame_lane_direction.get(frame, {})

        ego_lane = ego.lane_id
        if ego_lane not in lane_map:
            return []
        lane_list = lane_map[ego_lane]
        # O(1) index lookup via precomputed map (was list.index() before).
        ego_lane_index = self.data.frame_lane_index.get(frame, {}).get(ego_lane)
        if ego_lane_index is not None and ego_id in ego_lane_index:
            idx = ego_lane_index[ego_id]
        else:
            try:
                idx = lane_list.index(ego_id)
            except ValueError:
                return []

        surrounding: List[Tuple[int, str]] = []
        if idx < len(lane_list) - 1:
            surrounding.append((lane_list[idx + 1], "front"))

        ego_dir = lane_dir_map.get(ego_lane)
        if ego_dir is None or ego.proj_dist is None:
            return surrounding

        for lane_offset, relation in [
            (self.lane_config.left_lane_offset, "left_front"),
            (self.lane_config.right_lane_offset, "right_front"),
        ]:
            lane = ego_lane + lane_offset
            if lane not in lane_map:
                continue
            lane_dir = lane_dir_map.get(lane)
            if lane_dir is None:
                continue
            if (
                self._angle_diff_deg(np.degrees(ego_dir), np.degrees(lane_dir))
                >= self.lane_config.lane_direction_similarity_deg
            ):
                continue
            targets = lane_map[lane]
            proj_list = [self.data.frame_vehicles[frame][cid].proj_dist for cid in targets]
            valid = [(cid, pdist) for cid, pdist in zip(targets, proj_list) if pdist is not None]
            if not valid:
                continue
            valid_ids = [cid for cid, _ in valid]
            valid_proj = [float(pdist) for _, pdist in valid]
            pos = bisect.bisect_left(valid_proj, float(ego.proj_dist))
            if pos < len(valid_ids):
                surrounding.append((valid_ids[pos], relation))

        return surrounding

    def compute_pair_in_frame(self, frame: int, id1: int, id2: int) -> Optional[Dict[str, float]]:
        v1 = self.data.get_vehicle(frame, id1)
        v2 = self.data.get_vehicle(frame, id2)
        if v1 is None or v2 is None:
            return None
        same_lane = v1.lane_id == v2.lane_id
        lane_dir = self.data.frame_lane_direction.get(frame, {}).get(v1.lane_id) if same_lane else None
        if same_lane and lane_dir is None:
            same_lane = False
        return self.instant_calculator.compute_for_pair(v1, v2, same_lane=same_lane, lane_dir_rad=lane_dir)


class PairPeriodAnalyzer(BaseSSMAnalyzer):
    """指定两车+时间段计算."""

    def analyze(
        self,
        ego_id: int,
        target_id: int,
        start_frame: Optional[int] = None,
        end_frame: Optional[int] = None,
    ) -> Dict[str, object]:
        frames = self.data.frames
        if not frames:
            return {"ego_id": ego_id, "target_id": target_id, "frames": [], "period": {}}
        start = start_frame if start_frame is not None else int(frames[0])
        end = end_frame if end_frame is not None else int(frames[-1])

        records: List[Dict[str, object]] = []
        ttc_history: List[float] = []
        drac_history: List[Tuple[int, float, int]] = []

        for frame in frames:
            frame = int(frame)
            if frame < start or frame > end:
                continue
            ssm = self.compute_pair_in_frame(frame, ego_id, target_id)
            if ssm is None:
                continue
            if np.isfinite(ssm["TTC"]):
                ttc_history.append(float(ssm["TTC"]))
            if np.isfinite(ssm["DRAC"]):
                ego = self.data.get_vehicle(frame, ego_id)
                if ego is not None:
                    drac_history.append((frame, float(ssm["DRAC"]), ego.obj_class))
            records.append({"frame": frame, "ego_id": ego_id, "target_id": target_id, **ssm})

        tet, tit = self.period_calculator.compute_tet_tit(
            ttc_history, self.data.dt, self.period_config.tet_tit_threshold
        )
        cpi = self.period_calculator.compute_cpi(
            drac_history, self.period_config.vehicle_class_madr or {}, self.data.dt
        )
        return {
            "ego_id": ego_id,
            "target_id": target_id,
            "start_frame": start,
            "end_frame": end,
            "frames": records,
            "period": {"TET": tet, "TIT": tit, "CPI": cpi},
        }


class BatchFrontAnalyzer(BaseSSMAnalyzer):
    """全量计算: 每帧主车与 left_front/front/right_front."""

    def analyze(
        self,
        start_frame: Optional[int] = None,
        end_frame: Optional[int] = None,
        relations: Sequence[str] = ("left_front", "front", "right_front"),
    ) -> Dict[str, object]:
        frames = self.data.frames
        if not frames:
            return {"records": [], "pairs": {}}
        start = start_frame if start_frame is not None else int(frames[0])
        end = end_frame if end_frame is not None else int(frames[-1])
        allowed = set(relations)

        records: List[Dict[str, object]] = []
        pair_histories: Dict[Tuple[int, int], Dict[str, object]] = {}
        for frame in frames:
            frame = int(frame)
            if frame < start or frame > end:
                continue
            for ego_id in self.data.frame_vehicles.get(frame, {}):
                for target_id, relation in self.get_surrounding_vehicles(frame, ego_id):
                    if relation not in allowed:
                        continue
                    ssm = self.compute_pair_in_frame(frame, ego_id, target_id)
                    if ssm is None:
                        continue
                    row = {"frame": frame, "ego_id": ego_id, "target_id": target_id, "relation": relation, **ssm}
                    records.append(row)

                    key = (ego_id, target_id)
                    if key not in pair_histories:
                        pair_histories[key] = {
                            "relation": relation,
                            "ttc_history": [],
                            "drac_history": [],
                            "start_frame": frame,
                            "end_frame": frame,
                        }
                    pair_histories[key]["end_frame"] = frame
                    if np.isfinite(ssm["TTC"]):
                        pair_histories[key]["ttc_history"].append(float(ssm["TTC"]))
                    if np.isfinite(ssm["DRAC"]):
                        ego = self.data.get_vehicle(frame, ego_id)
                        if ego is not None:
                            pair_histories[key]["drac_history"].append((frame, float(ssm["DRAC"]), ego.obj_class))

        pair_summary: Dict[str, object] = {}
        for (ego_id, target_id), hist in pair_histories.items():
            tet, tit = self.period_calculator.compute_tet_tit(
                hist["ttc_history"], self.data.dt, self.period_config.tet_tit_threshold
            )
            cpi = self.period_calculator.compute_cpi(
                hist["drac_history"], self.period_config.vehicle_class_madr or {}, self.data.dt
            )
            pair_summary[f"{ego_id}-{target_id}"] = {
                "ego_id": ego_id,
                "target_id": target_id,
                "relation": hist["relation"],
                "start_frame": hist["start_frame"],
                "end_frame": hist["end_frame"],
                "period": {"TET": tet, "TIT": tit, "CPI": cpi},
            }

        return {"start_frame": start, "end_frame": end, "records": records, "pairs": pair_summary}
