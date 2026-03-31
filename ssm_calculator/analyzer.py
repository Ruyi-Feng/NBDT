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

        for frame, group in df.groupby("frameNum"):
            frame_int = int(frame)
            veh_dict: Dict[int, VehicleState] = {}
            lane_dict: Dict[int, List[int]] = defaultdict(list)
            lane_dir_dict: Dict[int, float] = {}

            for lane, lane_group in group.groupby("laneId"):
                lane_int = int(lane) if not pd.isna(lane) else -1
                if lane_group["proj_dist"].notna().any():
                    lane_group_sorted = lane_group.sort_values("proj_dist")
                    lane_dict[lane_int] = lane_group_sorted["carId"].astype(int).tolist()
                else:
                    lane_dict[lane_int] = lane_group["carId"].astype(int).tolist()

                lane_dir = lane_group["lane_dir_rad"].iloc[0]
                if not pd.isna(lane_dir):
                    lane_dir_dict[lane_int] = float(lane_dir)

            for _, row in group.iterrows():
                car_id = int(row["carId"])
                state = VehicleState(
                    x=float(row["x"]),
                    y=float(row["y"]),
                    vx=float(row["vx"]),
                    vy=float(row["vy"]),
                    speed=float(row["speed"]),
                    heading=float(row["heading"]),
                    acceleration=float(row["a"]),
                    angle_speed=float(row["angle_speed"]),
                    length=float(row["L"]),
                    width=float(row["W"]),
                    lane_id=int(row["laneId"]) if not pd.isna(row["laneId"]) else -1,
                    obj_class=int(row["objClass"]) if not pd.isna(row["objClass"]) else -1,
                    bbox=[
                        (float(row["boundingBox1Xm"]), float(row["boundingBox1Ym"])),
                        (float(row["boundingBox2Xm"]), float(row["boundingBox2Ym"])),
                        (float(row["boundingBox3Xm"]), float(row["boundingBox3Ym"])),
                        (float(row["boundingBox4Xm"]), float(row["boundingBox4Ym"])),
                    ],
                    proj_dist=float(row["proj_dist"]) if "proj_dist" in row and not pd.isna(row["proj_dist"]) else None,
                    proj_speed=float(row["proj_speed"]) if "proj_speed" in row and not pd.isna(row["proj_speed"]) else None,
                    proj_acc=float(row["proj_acc"]) if "proj_acc" in row and not pd.isna(row["proj_acc"]) else None,
                )
                veh_dict[car_id] = state

            self.frame_vehicles[frame_int] = veh_dict
            self.frame_lane_vehicles[frame_int] = lane_dict
            self.frame_lane_direction[frame_int] = lane_dir_dict

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
