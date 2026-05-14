import os
import math
import numpy as np
import pandas as pd
from PIL import Image

class BasicTransfer:
    def __init__(self, args):
        self.args = args

    def get_all_data(self) -> list:
        file_names = os.listdir(self.args.data_folder)
        data_list = []
        for file_name in file_names:
            data_list.append(os.path.join(self.args.data_folder, file_name))
        return data_list

    def _process_data(self, file_path: str) -> pd.DataFrame:
        raise NotImplementedError

    def _save_data(self, processed_data: pd.DataFrame, file_name: str) -> None:
        file_name = os.path.basename(file_name)
        file_name = file_name.split(".")[0]
        save_path = os.path.join(self.args.save_folder, file_name+".csv")
        processed_data.to_csv(save_path, index=False)

    def run(self) -> None:
        data_list = self.get_all_data()
        for file_path in data_list:
            processed_data = self._process_data(file_path)
            self._save_data(processed_data, file_path)


class HighDTransfer(BasicTransfer):

    def __init__(self, args):
        super(HighDTransfer, self).__init__(args)

    def get_all_data(self) -> list:
        """Override: only return XX_tracks.csv files (skip meta/jpg files)."""
        file_names = os.listdir(self.args.data_folder)
        data_list = []
        for file_name in file_names:
            if file_name.endswith('_tracks.csv'):
                data_list.append(os.path.join(self.args.data_folder, file_name))
        data_list.sort()
        return data_list

    def _process_data(self, file_path: str) -> pd.DataFrame:
        # 对于不同的数据集transfer，补充这个函数即可。返回处理好的dataframe
        # 一般情况保持 basictransfer 不动

        # Read data files
        tracks = pd.read_csv(file_path)

        # Derive corresponding meta file paths from XX_tracks.csv
        prefix = file_path.replace('_tracks.csv', '')
        tracks_meta = pd.read_csv(prefix + '_tracksMeta.csv')

        # Merge class & drivingDirection from tracksMeta
        tracks = tracks.merge(
            tracks_meta[['id', 'class', 'drivingDirection']],
            on='id', how='left'
        )

        # Compute vehicle center coordinates (meters)
        carCenterXm = tracks['x'] + tracks['width'] / 2
        carCenterYm = tracks['y'] + tracks['height'] / 2

        # Compute speed (m/s)
        speed = np.sqrt(tracks['xVelocity']**2 + tracks['yVelocity']**2)

        # Compute heading angle (relative to image X-axis, 0-360 degrees)
        heading = np.degrees(
            np.arctan2(tracks['yVelocity'], tracks['xVelocity'])
        ) % 360

        # Handle stationary vehicles (speed=0): assign heading by drivingDirection
        #   drivingDirection=1 (upper lanes, moving left) -> 180 degrees
        #   drivingDirection=2 (lower lanes, moving right) -> 0 degrees
        stationary = (tracks['xVelocity'] == 0) & (tracks['yVelocity'] == 0)
        heading[stationary & (tracks['drivingDirection'] == 1)] = 180.0
        heading[stationary & (tracks['drivingDirection'] == 2)] = 0.0

        # Compute Oriented Bounding Box 4 corner points
        # highD: width = vehicle length, height = vehicle width
        l = tracks['width'] / 2   # half-length (along heading direction)
        w = tracks['height'] / 2  # half-width  (perpendicular to heading)
        theta = np.radians(heading)
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        lc = l * cos_t
        ls = l * sin_t
        wc = w * cos_t
        ws = w * sin_t
        #   Corner1(front-left)   Corner2(front-right)
        #   Corner4(rear-left)    Corner3(rear-right)
        bb1Xm = carCenterXm + lc + ws
        bb1Ym = carCenterYm + ls - wc
        bb2Xm = carCenterXm + lc - ws
        bb2Ym = carCenterYm + ls + wc
        bb3Xm = carCenterXm - lc - ws
        bb3Ym = carCenterYm - ls + wc
        bb4Xm = carCenterXm - lc + ws
        bb4Ym = carCenterYm - ls - wc

        # Map vehicle class
        # highD: "Car" / "Truck" -> NBDT: 0=car, 3=truck
        class_map = {'Car': 0, 'Truck': 3}
        objClass = tracks['class'].map(class_map).fillna(-1).astype(int)

        # Convert meter coordinates to pixel coordinates on the highway background image
        # Per-recording pix2meter: road_length (m) / image_width (px)
        #   road_length = max(frontSightDistance + backSightDistance) across all vehicles
        #   image_width = XX_highway.png pixel width
        highway_img = Image.open(prefix + '_highway.png')
        img_width = highway_img.size[0]
        highway_img.close()
        road_length = (tracks['frontSightDistance'] + tracks['backSightDistance']).max()
        PIX2METER = road_length / img_width
        carCenterX = carCenterXm / PIX2METER
        carCenterY = carCenterYm / PIX2METER
        bb1X = bb1Xm / PIX2METER
        bb1Y = bb1Ym / PIX2METER
        bb2X = bb2Xm / PIX2METER
        bb2Y = bb2Ym / PIX2METER
        bb3X = bb3Xm / PIX2METER
        bb3Y = bb3Ym / PIX2METER
        bb4X = bb4Xm / PIX2METER
        bb4Y = bb4Ym / PIX2METER

        # Build standard format DataFrame
        # Column order follows NBDT TrajectoryDataFormat wiki specification
        result = pd.DataFrame({
            'frameNum': tracks['frame'],
            'carId': tracks['id'],
            # Pixel coordinates (on highway background image)
            'carCenterX': carCenterX,
            'carCenterY': carCenterY,
            'boundingBox1X': bb1X,
            'boundingBox1Y': bb1Y,
            'boundingBox2X': bb2X,
            'boundingBox2Y': bb2Y,
            'boundingBox3X': bb3X,
            'boundingBox3Y': bb3Y,
            'boundingBox4X': bb4X,
            'boundingBox4Y': bb4Y,
            # Meter coordinates
            'carCenterXm': carCenterXm,
            'carCenterYm': carCenterYm,
            'boundingBox1Xm': bb1Xm,
            'boundingBox1Ym': bb1Ym,
            'boundingBox2Xm': bb2Xm,
            'boundingBox2Ym': bb2Ym,
            'boundingBox3Xm': bb3Xm,
            'boundingBox3Ym': bb3Ym,
            'boundingBox4Xm': bb4Xm,
            'boundingBox4Ym': bb4Ym,
            # Motion attributes
            'heading': heading,
            'course': -1,       # No global north reference in highD
            'speed': speed,
            'objClass': objClass,
            # Geographic coordinates (highD has no GPS data)
            'carCenterLon': -1,
            'carCenterLat': -1,
            # Additional field
            'laneId': tracks['laneId'],
        })

        return result


class InDTransfer(BasicTransfer):
    def __init__(self, args):
        super(InDTransfer, self).__init__(args)

    def get_all_data(self) -> list:
        """Override: only return XX_tracks.csv files."""
        file_names = os.listdir(self.args.data_folder)
        data_list = []
        for file_name in file_names:
            if file_name.endswith('_tracks.csv'):
                data_list.append(os.path.join(self.args.data_folder, file_name))
        data_list.sort()
        return data_list

    def _process_data(self, file_path: str) -> pd.DataFrame:
        # Read tracks.csv
        tracks = pd.read_csv(file_path)

        # Derive corresponding meta file paths
        prefix = file_path.replace('_tracks.csv', '')
        tracks_meta = pd.read_csv(prefix + '_tracksMeta.csv')
        recording_meta = pd.read_csv(prefix + '_recordingMeta.csv')

        # Merge class from tracksMeta
        tracks = tracks.merge(
            tracks_meta[['trackId', 'class']],
            on='trackId', how='left'
        )

        # Center coordinates (already in meters)
        carCenterXm = tracks['xCenter']
        carCenterYm = tracks['yCenter']

        # Heading (already provided, in degrees)
        heading = tracks['heading']

        # Speed (m/s)
        speed = np.sqrt(tracks['xVelocity']**2 + tracks['yVelocity']**2)

        # OBB 4 corner points
        # inD: length=vehicle length (along heading), width=vehicle width
        l = tracks['length'] / 2
        w = tracks['width'] / 2
        theta = np.radians(heading)
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        lc = l * cos_t
        ls = l * sin_t
        wc = w * cos_t
        ws = w * sin_t
        bb1Xm = carCenterXm + lc + ws
        bb1Ym = carCenterYm + ls - wc
        bb2Xm = carCenterXm + lc - ws
        bb2Ym = carCenterYm + ls + wc
        bb3Xm = carCenterXm - lc - ws
        bb3Ym = carCenterYm - ls + wc
        bb4Xm = carCenterXm - lc + ws
        bb4Ym = carCenterYm - ls - wc

        # Map vehicle class (NBDT Wiki: 0=car, 3=truck, 5=pedestrian)
        class_map = {'car': 0, 'truck_bus': 3, 'pedestrian': 5, 'bicycle': -1}
        objClass = tracks['class'].map(class_map).fillna(-1).astype(int)

        # Convert meter coordinates to pixel coordinates on background image
        # Background image is downscaled from original orthophoto:
        #   inD scale_down_factor=12, rounD scale_down_factor=10
        # pixel_x = xCenter / (orthoPxToMeter * scale_down_factor)
        # pixel_y = -yCenter / (orthoPxToMeter * scale_down_factor)  (Y axis inverted)
        PIX2METER = recording_meta['orthoPxToMeter'].iloc[0]
        scale_down = getattr(self.args, 'scale_down_factor', 12)
        effective_scale = PIX2METER * scale_down
        carCenterX = carCenterXm / effective_scale
        carCenterY = -carCenterYm / effective_scale
        bb1X = bb1Xm / effective_scale
        bb1Y = -bb1Ym / effective_scale
        bb2X = bb2Xm / effective_scale
        bb2Y = -bb2Ym / effective_scale
        bb3X = bb3Xm / effective_scale
        bb3Y = -bb3Ym / effective_scale
        bb4X = bb4Xm / effective_scale
        bb4Y = -bb4Ym / effective_scale

        result = pd.DataFrame({
            'frameNum': tracks['frame'],
            'carId': tracks['trackId'],
            'carCenterX': carCenterX,
            'carCenterY': carCenterY,
            'boundingBox1X': bb1X,
            'boundingBox1Y': bb1Y,
            'boundingBox2X': bb2X,
            'boundingBox2Y': bb2Y,
            'boundingBox3X': bb3X,
            'boundingBox3Y': bb3Y,
            'boundingBox4X': bb4X,
            'boundingBox4Y': bb4Y,
            'carCenterXm': carCenterXm,
            'carCenterYm': carCenterYm,
            'boundingBox1Xm': bb1Xm,
            'boundingBox1Ym': bb1Ym,
            'boundingBox2Xm': bb2Xm,
            'boundingBox2Ym': bb2Ym,
            'boundingBox3Xm': bb3Xm,
            'boundingBox3Ym': bb3Ym,
            'boundingBox4Xm': bb4Xm,
            'boundingBox4Ym': bb4Ym,
            'heading': heading,
            'course': -1,
            'speed': speed,
            'objClass': objClass,
            'carCenterLon': -1,
            'carCenterLat': -1,
            'laneId': -1,
        })

        return result

class CitySimTransfer(BasicTransfer):
    def __init__(self, args):
        super(CitySimTransfer, self).__init__(args)

    def get_all_data(self) -> list:
        """Override: return all .csv files."""
        file_names = sorted(os.listdir(self.args.data_folder))
        data_list = [
            os.path.join(self.args.data_folder, f)
            for f in file_names
            if f.lower().endswith('.csv')
        ]
        return data_list

    def _process_data(self, file_path: str) -> pd.DataFrame:
        raw = pd.read_csv(file_path)

        FEET2METER = 0.3048

        # Meter coordinates (convert from feet)
        carCenterXm = raw['carCenterXft'] * FEET2METER
        carCenterYm = raw['carCenterYft'] * FEET2METER
        bb1Xm = raw['boundingBox1Xft'] * FEET2METER
        bb1Ym = raw['boundingBox1Yft'] * FEET2METER
        bb2Xm = raw['boundingBox2Xft'] * FEET2METER
        bb2Ym = raw['boundingBox2Yft'] * FEET2METER
        bb3Xm = raw['boundingBox3Xft'] * FEET2METER
        bb3Ym = raw['boundingBox3Yft'] * FEET2METER
        bb4Xm = raw['boundingBox4Xft'] * FEET2METER
        bb4Ym = raw['boundingBox4Yft'] * FEET2METER

        result = pd.DataFrame({
            'frameNum': raw['frameNum'],
            'carId': raw['carId'],
            # Pixel coordinates (directly from raw)
            'carCenterX': raw['carCenterX'],
            'carCenterY': raw['carCenterY'],
            'boundingBox1X': raw['boundingBox1X'],
            'boundingBox1Y': raw['boundingBox1Y'],
            'boundingBox2X': raw['boundingBox2X'],
            'boundingBox2Y': raw['boundingBox2Y'],
            'boundingBox3X': raw['boundingBox3X'],
            'boundingBox3Y': raw['boundingBox3Y'],
            'boundingBox4X': raw['boundingBox4X'],
            'boundingBox4Y': raw['boundingBox4Y'],
            # Meter coordinates
            'carCenterXm': carCenterXm,
            'carCenterYm': carCenterYm,
            'boundingBox1Xm': bb1Xm,
            'boundingBox1Ym': bb1Ym,
            'boundingBox2Xm': bb2Xm,
            'boundingBox2Ym': bb2Ym,
            'boundingBox3Xm': bb3Xm,
            'boundingBox3Ym': bb3Ym,
            'boundingBox4Xm': bb4Xm,
            'boundingBox4Ym': bb4Ym,
            # Motion attributes
            'heading': raw['course'],
            'course': raw['heading'],
            'speed': raw['speed'],
            'objClass': -1,
            'carCenterLon': -1,
            'carCenterLat': -1,
            'laneId': raw['laneId'],
        })

        return result


class NGSIMTransfer(BasicTransfer):

    def __init__(self, args):
        super().__init__(args)

    def get_all_data(self) -> list:
        file_names = sorted(os.listdir(self.args.data_folder))
        data_list = [
            os.path.join(self.args.data_folder, f)
            for f in file_names
            if f.lower().endswith('.csv')
        ]
        return data_list

    @staticmethod
    def _build_local_frame(df: pd.DataFrame):
        FEET2METER = 0.3048
        gx_m = df['Global_X'].values * FEET2METER
        gy_m = df['Global_Y'].values * FEET2METER

        x_origin = gx_m.min()   # x = 0
        y_origin = gy_m.max()   # y = 0

        local_x_m =  (gx_m - x_origin)
        local_y_m = -(gy_m - y_origin)

        return local_x_m, local_y_m, (x_origin, y_origin)

    @staticmethod
    def _compute_heading_and_course(df: pd.DataFrame,
                                    local_x_m: np.ndarray,
                                    local_y_m: np.ndarray,
                                    max_frame_gap: int = 5) -> tuple:
        FEET2METER = 0.3048
        gx_m = df['Global_X'].values * FEET2METER
        gy_m = df['Global_Y'].values * FEET2METER
        vid = df['Vehicle_ID'].values
        fnum = df['Frame_ID'].values

        n = len(df)
        heading = np.full(n, -1.0)
        course = np.full(n, -1.0)

        for i in range(n):
            has_next = (i + 1 < n and vid[i + 1] == vid[i]
                        and abs(fnum[i + 1] - fnum[i]) <= max_frame_gap)
            has_prev = (i - 1 >= 0 and vid[i - 1] == vid[i]
                        and abs(fnum[i] - fnum[i - 1]) <= max_frame_gap)

            if has_next:
                curr, ref = i, i + 1
            elif has_prev:
                curr, ref = i - 1, i
            else:
                continue  

            dx_img = local_x_m[ref] - local_x_m[curr]
            dy_img = local_y_m[ref] - local_y_m[curr]
            heading[i] = np.degrees(np.arctan2(dy_img, dx_img)) % 360

            dE = gx_m[ref] - gx_m[curr]
            dN = gy_m[ref] - gy_m[curr]
            course[i] = np.degrees(np.arctan2(dE, dN)) % 360

        return heading, course

    @staticmethod
    def _oriented_bbox(cx_m, cy_m, length_m, width_m, heading_deg):
        l = length_m / 2
        w = width_m  / 2
        theta = np.radians(heading_deg)
        cos_t, sin_t = np.cos(theta), np.sin(theta)

        lc, ls = l * cos_t, l * sin_t
        wc, ws = w * cos_t, w * sin_t

        bb1Xm = cx_m + lc + ws
        bb1Ym = cy_m + ls - wc
        bb2Xm = cx_m + lc - ws
        bb2Ym = cy_m + ls + wc
        bb3Xm = cx_m - lc - ws
        bb3Ym = cy_m - ls + wc
        bb4Xm = cx_m - lc + ws
        bb4Ym = cy_m - ls - wc

        return bb1Xm, bb1Ym, bb2Xm, bb2Ym, bb3Xm, bb3Ym, bb4Xm, bb4Ym


    def _process_data(self, file_path: str) -> pd.DataFrame:
        raw = pd.read_csv(file_path)
        raw.columns = raw.columns.str.strip()

        raw = raw.sort_values(['Vehicle_ID', 'Frame_ID']).reset_index(drop=True)
        FEET2METER = 0.3048
        length_m = raw['v_length'].values * FEET2METER   # vehicle length (m)
        width_m  = raw['v_Width'].values  * FEET2METER   # vehicle width  (m)
        speed_fps = raw['v_Vel'].values                   # feet/s
        speed_ms  = speed_fps * FEET2METER                # m/s

        local_x_m, local_y_m, origin = self._build_local_frame(raw)
        carCenterXm = local_x_m
        carCenterYm = local_y_m

        heading, course = self._compute_heading_and_course(
            raw, local_x_m, local_y_m
        )

        (bb1Xm, bb1Ym, bb2Xm, bb2Ym, bb3Xm, bb3Ym,bb4Xm, bb4Ym) = self._oriented_bbox(
             carCenterXm, carCenterYm, length_m, width_m, heading
         )
        PIX2METER_NGSIM = 1.0  # 1 pixel == 1 metre (placeholder)
        carCenterX = carCenterXm / PIX2METER_NGSIM
        carCenterY = carCenterYm / PIX2METER_NGSIM
        bb1X = bb1Xm / PIX2METER_NGSIM
        bb1Y = bb1Ym / PIX2METER_NGSIM
        bb2X = bb2Xm / PIX2METER_NGSIM
        bb2Y = bb2Ym / PIX2METER_NGSIM
        bb3X = bb3Xm / PIX2METER_NGSIM
        bb3Y = bb3Ym / PIX2METER_NGSIM
        bb4X = bb4Xm / PIX2METER_NGSIM
        bb4Y = bb4Ym / PIX2METER_NGSIM

        # Map vehicle class
        NGSIM_CLASS_MAP = {
            1: 4,  # motorcycle
            2: 0,  # auto → car
            3: 3,  # truck
        }
        objClass = raw['v_Class'].map(NGSIM_CLASS_MAP).fillna(-1).astype(int)

        result = pd.DataFrame({
            'frameNum':       raw['Frame_ID'],
            'carId':          raw['Vehicle_ID'],
            'laneId':         raw['Lane_ID'],
            'carCenterX':     carCenterX,
            'carCenterY':     carCenterY,
            'boundingBox1X':  bb1X,
            'boundingBox1Y':  bb1Y,
            'boundingBox2X':  bb2X,
            'boundingBox2Y':  bb2Y,
            'boundingBox3X':  bb3X,
            'boundingBox3Y':  bb3Y,
            'boundingBox4X':  bb4X,
            'boundingBox4Y':  bb4Y,
            # Metre coordinates
            'carCenterXm':    carCenterXm,
            'carCenterYm':    carCenterYm,
            'boundingBox1Xm': bb1Xm,
            'boundingBox1Ym': bb1Ym,
            'boundingBox2Xm': bb2Xm,
            'boundingBox2Ym': bb2Ym,
            'boundingBox3Xm': bb3Xm,
            'boundingBox3Ym': bb3Ym,
            'boundingBox4Xm': bb4Xm,
            'boundingBox4Ym': bb4Ym,
            # Motion
            'heading':        heading,
            'course':         course,
            'speed':          speed_ms,
            'objClass':       objClass,
            # Geographic
            'carCenterLon':   -1,
            'carCenterLat':   -1,
        })

        return result

from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple
import copy
class UTETransfer(BasicTransfer):

    UTE_CLASS_MAP: Dict[int, int] = {0: 0, 2: 2}
    MOT_PIXEL_COLUMNS_NO_HEADER: Sequence[str] = (
        "frame", "car_id", "x", "y", "w", "h", "conf", "_m1", "_m2", "_m3",
    )
    DEFAULT_SMOOTH_WINDOW = 5
    DEFAULT_XAM_S9_MIN_GEOM_FRAMES = 5
    DEFAULT_XAM_S9_MAX_GEOM_MEDIAN_M = 15.0
    XAM_S9_FRENET_FPS = 30.0

    @dataclass(frozen=True)
    class LocationConfig:
        pixel_suffix: str
        frenet_suffix: str
        pix2meter: float
        pixel_rename: Dict[str, str] = field(default_factory=dict)
        frenet_rename: Dict[str, str] = field(default_factory=dict)
        build_frenet_path: Optional[Callable[..., str]] = None
        post_merge: Optional[Callable[..., pd.DataFrame]] = None
        pixel_default_cls: Optional[int] = None
        geom_line_pos_filename: Optional[str] = None
        pixel_mot_no_header: bool = False
        pixel_dedupe_frame_car: bool = False
        frenet_dedupe_frame_car: bool = False

    LOCATION_CONFIG: Dict[str, "UTETransfer.LocationConfig"] = {}

    def __init__(self, args):
        super().__init__(args)
        loc_key = str(getattr(args, "location", "")).lower().strip()
        if not loc_key or loc_key not in self.LOCATION_CONFIG:
            raise ValueError(
                f"Unknown UTE location '{getattr(args, 'location', None)}'. "
                f"Available: {list(self.LOCATION_CONFIG.keys())}"
            )
        self.location_key = loc_key
        self.cfg = self.LOCATION_CONFIG[loc_key]
        self.pix2meter = float(self.cfg.pix2meter)
        self.smooth_win = int(getattr(args, "smooth_window", self.DEFAULT_SMOOTH_WINDOW))
        self.xam_s9_min_geom_frames = int(
            getattr(args, "xam_s9_min_geom_frames", self.DEFAULT_XAM_S9_MIN_GEOM_FRAMES)
        )
        self.xam_s9_max_geom_median_m = float(
            getattr(args, "xam_s9_max_geom_median_m", self.DEFAULT_XAM_S9_MAX_GEOM_MEDIAN_M)
        )
        self._processor = UTETransfer._PROCESSORS[loc_key]

    _PROCESSORS: Dict[str, Callable[..., pd.DataFrame]] = {}

    def get_all_data(self) -> list:
        suffix = self.cfg.pixel_suffix
        files = sorted(
            f for f in os.listdir(self.args.data_folder)
            if f.lower().endswith(suffix.lower())
        )
        return [os.path.join(self.args.data_folder, f) for f in files]

    def _save_data(self, processed_data: pd.DataFrame, file_name: str) -> None:
        save_path = os.path.join(self.args.save_folder, self.location_key + ".csv")
        out = self._integerize_ute_csv_columns(processed_data)
        out.to_csv(save_path, index=False)
        print(f"  Saved → {save_path}")

    @staticmethod
    def _default_build_frenet_path(pixel_path: str, cfg: "UTETransfer.LocationConfig") -> str:
        return pixel_path[: -len(cfg.pixel_suffix)] + cfg.frenet_suffix

    def _load_ute_pixel_csv(self, pixel_path: str, cfg: LocationConfig) -> pd.DataFrame:
        if cfg.pixel_mot_no_header:
            pix = pd.read_csv(
                pixel_path,
                header=None,
                names=list(self.MOT_PIXEL_COLUMNS_NO_HEADER),
            )
        else:
            pix = pd.read_csv(pixel_path)
            pix.columns = pix.columns.str.strip()
        pix = pix.rename(columns=dict(cfg.pixel_rename))
        if cfg.pixel_dedupe_frame_car:
            pix = pix.sort_values(["car_id", "frame_id"], kind="mergesort")
            pix = (
                pix.drop_duplicates(subset=["frame_id", "car_id"], keep="last")
                .reset_index(drop=True)
            )
        if cfg.pixel_default_cls is not None and "cls" not in pix.columns:
            pix = pix.copy()
            pix["cls"] = cfg.pixel_default_cls
        return pix

    @staticmethod
    def _build_xamn5_mapping(
        pix: pd.DataFrame,
        fre: pd.DataFrame,
        fps: float = 25.0,
        min_overlap: float = 0.5,
    ) -> Dict[int, int]:
        from scipy.optimize import linear_sum_assignment

        pix = pix.copy()
        pix["time_s"] = (pix["frame_id"] - 1) / fps

        pix_stats = (
            pix.groupby("car_id")
            .agg(t_start=("time_s", "min"), t_end=("time_s", "max"))
            .reset_index()
        )
        fre_stats = (
            fre.groupby("Vehicle ID")
            .agg(t_start=("Time(s)", "min"), t_end=("Time(s)", "max"))
            .reset_index()
        )

        pix_ids = pix_stats["car_id"].tolist()
        fre_ids = fre_stats["Vehicle ID"].tolist()
        n_pix = len(pix_ids)
        n_fre = len(fre_ids)

        overlap = np.zeros((n_pix, n_fre), dtype=float)
        for i, prow in pix_stats.iterrows():
            pt0, pt1 = prow["t_start"], prow["t_end"]
            for j, frow in fre_stats.iterrows():
                ft0, ft1 = frow["t_start"], frow["t_end"]
                inter = max(0.0, min(pt1, ft1) - max(pt0, ft0))
                union = max(pt1, ft1) - min(pt0, ft0)
                if union > 0:
                    overlap[i, j] = inter / union

        row_ind, col_ind = linear_sum_assignment(-overlap)

        mapping: Dict[int, int] = {}
        for r, c in zip(row_ind, col_ind):
            if overlap[r, c] >= min_overlap:
                mapping[int(pix_ids[r])] = int(fre_ids[c])

        return mapping

    @staticmethod
    def _xamn5_align_frenet_to_pixel(
        pix: pd.DataFrame,
        fre: pd.DataFrame,
        fps: float = 25.0,
    ) -> pd.DataFrame:
        pix = pix.copy()
        pix["time_s"] = (pix["frame_id"] - 1) / fps

        fre_attrs = ["car_id", "time", "lane_id", "lon_m", "lat_m",
                     "speed_kmh", "length_m", "width_m"]
        fre = fre[fre_attrs].copy()

        result_rows = []

        for cid, pix_grp in pix.groupby("car_id", sort=False):
            fre_grp = fre[fre["car_id"] == cid].sort_values("time")
            if fre_grp.empty:
                for _, row in pix_grp.iterrows():
                    r = row.to_dict()
                    r.update({c: np.nan for c in fre_attrs if c not in r})
                    result_rows.append(r)
                continue

            fre_times = fre_grp["time"].values

            for _, prow in pix_grp.iterrows():
                t_px = prow["time_s"]
                idx = np.searchsorted(fre_times, t_px)
                idx = np.clip(idx, 0, len(fre_times) - 1)
                if idx > 0 and abs(fre_times[idx - 1] - t_px) < abs(fre_times[idx] - t_px):
                    idx -= 1
                frow = fre_grp.iloc[idx]

                r = prow.to_dict()
                r["lane_id"] = frow["lane_id"]
                r["lon_m"] = frow["lon_m"]
                r["lat_m"] = frow["lat_m"]
                r["speed_kmh"] = frow["speed_kmh"]
                r["length_m"] = frow["length_m"]
                r["width_m"] = frow["width_m"]
                result_rows.append(r)

        df = pd.DataFrame(result_rows)
        df = df.sort_values(["car_id", "frame_id"]).reset_index(drop=True)
        return df

    def _process_xamn5(self, pixel_path: str) -> pd.DataFrame:
        build = self.cfg.build_frenet_path or self._default_build_frenet_path
        frenet_path = build(pixel_path, self.cfg)
        if not os.path.exists(frenet_path):
            raise FileNotFoundError(f"Frenet file not found: {frenet_path}")

        pix = self._load_ute_pixel_csv(pixel_path, self.cfg)

        fre_raw = pd.read_csv(frenet_path)
        fre_raw.columns = fre_raw.columns.str.strip()

        mapping = self._build_xamn5_mapping(
            pix[["frame_id", "car_id", "x", "y", "w", "h", "cls"]],
            fre_raw,
            fps=25.0,
        )
        print(f"  XAM-N5: matched {len(mapping)} pixel cars to frenet vehicles.")
        print(f"  pixel car_ids : {sorted(pix['car_id'].unique().tolist())}")
        print(f"  frenet veh_ids: {sorted(fre_raw['Vehicle ID'].unique().tolist())}")
        print(f"  mapping       : {mapping}")

        pix["car_id"] = pix["car_id"].map(mapping)
        pix = pix.dropna(subset=["car_id"]).copy()
        pix["car_id"] = pix["car_id"].astype(int)

        fre = fre_raw.rename(columns={
            "Vehicle ID": "car_id",
            "Lane ID": "lane_id",
            "Time(s)": "time",
            "Longitudinal distance(m)": "lon_m",
            "Lateral distance(m)": "lat_m",
            "Velocity(km/h)": "speed_kmh",
            "Acceleration(m/s^2)": "accel",
            "Vehicle length(m)": "length_m",
            "Vehicle width(m)": "width_m",
        })

        df = self._xamn5_align_frenet_to_pixel(pix, fre, fps=25.0)

        cx_px = df["x"].values + df["w"].values / 2.0
        cy_px = df["y"].values + df["h"].values / 2.0
        return self._assemble(df, cx_px, cy_px)

    @staticmethod
    def _build_xams9_id_mapping(
        pix: pd.DataFrame,
        fre: pd.DataFrame,
        min_overlap: float = 0.5,
    ) -> Dict[int, int]:
        from scipy.optimize import linear_sum_assignment

        pix_stats = (
            pix.groupby("car_id", sort=False)
            .agg(t_start=("frame_id", "min"), t_end=("frame_id", "max"))
            .reset_index()
        )
        fre_stats = (
            fre.groupby("car_id", sort=False)
            .agg(t_start=("frame_id", "min"), t_end=("frame_id", "max"))
            .reset_index()
        )

        pix_ids = pix_stats["car_id"].tolist()
        fre_ids = fre_stats["car_id"].tolist()
        n_pix = len(pix_ids)
        n_fre = len(fre_ids)

        overlap = np.zeros((n_pix, n_fre), dtype=float)
        for i in range(n_pix):
            pt0 = float(pix_stats.iloc[i]["t_start"])
            pt1 = float(pix_stats.iloc[i]["t_end"])
            for j in range(n_fre):
                ft0 = float(fre_stats.iloc[j]["t_start"])
                ft1 = float(fre_stats.iloc[j]["t_end"])
                inter = max(0.0, min(pt1, ft1) - max(pt0, ft0))
                union = max(pt1, ft1) - min(pt0, ft0)
                if union > 0:
                    overlap[i, j] = inter / union

        row_ind, col_ind = linear_sum_assignment(-overlap)
        mapping: Dict[int, int] = {}
        for r, c in zip(row_ind, col_ind):
            if overlap[r, c] >= min_overlap:
                mapping[int(pix_ids[r])] = int(fre_ids[c])

        return mapping

    @staticmethod
    def _dedupe_frenet_frame_car(fre: pd.DataFrame) -> pd.DataFrame:
        sort_cols = ["car_id", "frame_id"]
        if "time" in fre.columns:
            sort_cols.append("time")
        return fre.sort_values(sort_cols).drop_duplicates(
            subset=["frame_id", "car_id"], keep="last"
        )

    @staticmethod
    def _xams9_align_frenet_to_pixel(
        pix: pd.DataFrame,
        fre: pd.DataFrame,
        fps: float,
    ) -> pd.DataFrame:
        pix = pix.copy()
        pix["time_s"] = pix["frame_id"].astype(float) / float(fps)

        fre_attrs = ["car_id", "time", "lane_id", "lon_m", "lat_m",
                     "speed_kmh", "length_m", "width_m"]
        fre = fre[fre_attrs].copy()

        result_rows = []
        for cid, pix_grp in pix.groupby("car_id", sort=False):
            fre_grp = fre[fre["car_id"] == cid].sort_values("time")
            if fre_grp.empty:
                for _, prow in pix_grp.iterrows():
                    r = prow.to_dict()
                    r.update({c: np.nan for c in fre_attrs if c not in r})
                    result_rows.append(r)
                continue

            fre_times = fre_grp["time"].values

            for _, prow in pix_grp.iterrows():
                t_px = prow["time_s"]
                idx = np.searchsorted(fre_times, t_px)
                idx = int(np.clip(idx, 0, len(fre_times) - 1))
                if idx > 0 and abs(fre_times[idx - 1] - t_px) < abs(fre_times[idx] - t_px):
                    idx -= 1
                frow = fre_grp.iloc[idx]

                r = prow.to_dict()
                r["lane_id"] = frow["lane_id"]
                r["lon_m"] = frow["lon_m"]
                r["lat_m"] = frow["lat_m"]
                r["speed_kmh"] = frow["speed_kmh"]
                r["length_m"] = frow["length_m"]
                r["width_m"] = frow["width_m"]
                result_rows.append(r)

        df = pd.DataFrame(result_rows)
        df = df.sort_values(["car_id", "frame_id"]).reset_index(drop=True)
        return df

    @staticmethod
    def _frenet_solve_xy(x: np.ndarray, imx: float, imy: float, lane_center) -> List[float]:
        x0 = float(x[0])
        y0 = float(x[1])
        pc = lane_center[0]
        eq1 = np.polyval(pc, x0) - y0
        dfx = np.polyval(np.polyder(pc), x0)
        eq2 = dfx * (imy - y0) + (imx - x0)
        return [eq1, eq2]

    class LinePosFrenet:
        """Pixel → Frenet (lon, lat) in metres using ``line_pos.npy`` and ``pix2meter``."""

        default_dt = 1.0

        def __init__(self, line_npy_path: str, pix2meter: float):
            self.pix2meter = float(pix2meter)
            self.line_pos: List[List[List[float]]] = [[]]
            self.line_fit: List[np.ndarray] = []
            self.center_fit: List[Tuple[np.ndarray, float, float]] = []
            self.total_in_pixel: List[float] = []
            self.lane = 0
            self._load_line_mtx(line_npy_path)
            self._build_lanes()

        def _load_line_mtx(self, line_path: str) -> None:
            line_mtx = np.load(line_path)
            self.line_pos = [[]]
            for row in line_mtx:
                lid = int(row[2])
                while lid >= len(self.line_pos):
                    self.line_pos.append([])
                self.line_pos[lid].append([float(row[0]), float(row[1])])

        def _build_lanes(self) -> None:
            self.lane = len(self.line_pos) - 1
            self.line_fit = []
            for line_id in range(self.lane + 1):
                pts = self.line_pos[line_id]
                npt = len(pts)
                x = np.array([p[0] for p in pts], dtype=float)
                y = np.array([p[1] for p in pts], dtype=float)
                deg = max(0, npt - 1)
                self.line_fit.append(np.polyfit(x, y, deg))
            self._center_plot()

        def _center_plot(self) -> None:
            self.center_fit = []
            self.total_in_pixel = []
            for i in range(self.lane):
                cov = min(len(self.line_fit[i]), len(self.line_fit[i + 1]))
                if len(self.line_fit[i]) > len(self.line_fit[i + 1]):
                    cen_fit = copy.deepcopy(self.line_fit[i])
                    for j in range(-cov, 0):
                        cen_fit[j] = cen_fit[j] + self.line_fit[i + 1][j]
                else:
                    cen_fit = copy.deepcopy(self.line_fit[i + 1])
                    for j in range(-cov, 0):
                        cen_fit[j] = cen_fit[j] + self.line_fit[i][j]
                cen_fit = cen_fit / 2.0
                x_o = max(self.line_pos[i][0][0], self.line_pos[i + 1][0][0])
                x_d = min(self.line_pos[i][-1][0], self.line_pos[i + 1][-1][0])
                self.center_fit.append([cen_fit, x_o, x_d])
                total_length = self._cumu(np.array([x_d, 0.0], dtype=float), i)
                self.total_in_pixel.append(total_length)

        def _which_lane(self, pos: Tuple[float, float]) -> int:
            min_dis = 1e9
            lane_id = -1
            imx, imy = float(pos[0]), float(pos[1])
            for i in range(len(self.center_fit)):
                cen_fit, _, _ = self.center_fit[i]
                y = np.polyval(cen_fit, imx)
                dis = abs(imy - y)
                if dis < min_dis:
                    min_dis = dis
                    lane_id = i
            return lane_id

        def _cumu(self, fre_pos: np.ndarray, lane_center_idx: int) -> float:
            cen_fit, x_o, _ = self.center_fit[lane_center_idx]
            x_target = float(fre_pos[0])
            if x_target <= x_o:
                return 0.0
            x = np.arange(x_o, x_target, self.default_dt)
            if len(x) < 2:
                return 0.0
            pc = np.poly1d(cen_fit)
            y = pc(x)
            piece = [
                float(np.hypot(x[k] - x[k - 1], y[k] - y[k - 1]))
                for k in range(1, len(x))
            ]
            return float(sum(piece))

        def _frenet_calcu(
            self,
            pos: Tuple[float, float],
            fre_pos: np.ndarray,
            lane_id: int,
        ) -> Tuple[float, float]:
            lat = float(np.hypot(pos[0] - fre_pos[0], pos[1] - fre_pos[1]))
            if pos[1] - fre_pos[1] > 0:
                lat *= -1.0
            lon = self._cumu(fre_pos, lane_id)
            return lon, lat

        def pixel_to_lonlat_m(self, imx: float, imy: float) -> Optional[Tuple[float, float, int]]:
            from scipy.optimize import fsolve

            pos = (float(imx), float(imy))
            lane_id = self._which_lane(pos)
            if lane_id < 0:
                return None
            lane_center = self.center_fit[lane_id]
            pc = lane_center[0]
            d_at = np.polyval(np.polyder(pc), imx)
            if abs(d_at) < 1e-12:
                fre_pos = np.array([imx, np.polyval(pc, imx)], dtype=float)
            else:
                sol = fsolve(
                    lambda z: UTETransfer._frenet_solve_xy(z, pos[0], pos[1], lane_center),
                    [imx, imy],
                )
                fre_pos = np.asarray(sol, dtype=float).flatten()[:2]
            if not np.all(np.isfinite(fre_pos)):
                return None
            lon_px, lat_px = self._frenet_calcu(pos, fre_pos, lane_id)
            s = 1.0 / self.pix2meter
            return float(lon_px * s), float(lat_px * s), int(lane_id)

    @classmethod
    def _build_xams9_id_mapping_geom(
        cls,
        pix: pd.DataFrame,
        fre: pd.DataFrame,
        frenet_px: "UTETransfer.LinePosFrenet",
        min_common_frames: int,
        max_median_m: float,
    ) -> Dict[int, int]:
        from scipy.optimize import linear_sum_assignment

        mot_feat: Dict[int, Dict[int, Tuple[float, float]]] = {}
        for (fr, cid), sub in pix.groupby(["frame_id", "car_id"], sort=False):
            cx = float((sub["x"] + sub["w"] / 2.0).mean())
            cy = float((sub["y"] + sub["h"] / 2.0).mean())
            ll = frenet_px.pixel_to_lonlat_m(cx, cy)
            if ll is None:
                continue
            lon_m, lat_m, _lid = ll
            mot_feat.setdefault(int(cid), {})[int(fr)] = (lon_m, lat_m)

        fre_feat: Dict[int, Dict[int, Tuple[float, float]]] = {}
        for row in fre.itertuples(index=False):
            fre_feat.setdefault(int(row.car_id), {})[int(row.frame_id)] = (
                float(row.lon_m),
                float(row.lat_m),
            )

        mot_ids = sorted(mot_feat.keys())
        fre_ids = sorted(fre_feat.keys())
        n_m, n_f = len(mot_ids), len(fre_ids)
        if n_m == 0 or n_f == 0:
            return {}

        big = max(1e6, max_median_m * 1.0e3)
        cost = np.full((n_m, n_f), big, dtype=np.float64)

        for i, mi in enumerate(mot_ids):
            mf = mot_feat[mi]
            for j, fj in enumerate(fre_ids):
                ff = fre_feat[fj]
                common = mf.keys() & ff.keys()
                if len(common) < min_common_frames:
                    continue
                dlist = [
                    float(np.hypot(mf[f][0] - ff[f][0], mf[f][1] - ff[f][1]))
                    for f in common
                ]
                cost[i, j] = float(np.median(dlist))

        r, c = linear_sum_assignment(cost)
        mapping: Dict[int, int] = {}
        for ri, ci in zip(r, c):
            if cost[ri, ci] < max_median_m:
                mapping[int(mot_ids[ri])] = int(fre_ids[ci])
        return mapping

    def _process_xams9(self, pixel_path: str) -> pd.DataFrame:
        build = self.cfg.build_frenet_path or self._default_build_frenet_path
        frenet_path = build(pixel_path, self.cfg)
        if not os.path.exists(frenet_path):
            raise FileNotFoundError(f"Frenet file not found: {frenet_path}")

        pix = self._load_ute_pixel_csv(pixel_path, self.cfg)

        fre = pd.read_csv(frenet_path)
        fre.columns = fre.columns.str.strip()
        fre = fre.rename(columns=self.cfg.frenet_rename)
        fre = self._dedupe_frenet_frame_car(fre)

        data_dir = os.path.dirname(os.path.abspath(pixel_path))
        n_pix_cars = int(pix["car_id"].nunique())
        line_name = self.cfg.geom_line_pos_filename
        line_path = os.path.join(data_dir, line_name) if line_name else ""

        if line_name and os.path.isfile(line_path):
            frenet_px = self.LinePosFrenet(line_path, self.pix2meter)
            mapping = self._build_xams9_id_mapping_geom(
                pix,
                fre,
                frenet_px,
                min_common_frames=self.xam_s9_min_geom_frames,
                max_median_m=self.xam_s9_max_geom_median_m,
            )
            print(
                f"  XAM-S9: matched {len(mapping)} / {n_pix_cars} MOT tracks "
                f"to frenet car_id (geometry + Hungarian, line_pos={line_name!r})."
            )
        else:
            mapping = self._build_xams9_id_mapping(
                pix[["frame_id", "car_id"]],
                fre[["frame_id", "car_id"]],
                min_overlap=0.5,
            )
            if line_name:
                why = f"line_pos file missing: {line_path}"
            else:
                why = "geom_line_pos_filename not set on location config"
            print(
                f"  XAM-S9: matched {len(mapping)} / {n_pix_cars} MOT tracks "
                f"to frenet car_id (frame-span IoU fallback; {why})."
            )

        pix["car_id"] = pix["car_id"].map(mapping)
        pix = pix.dropna(subset=["car_id"]).copy()
        pix["car_id"] = pix["car_id"].astype(int)

        df = self._xams9_align_frenet_to_pixel(pix, fre, fps=self.XAM_S9_FRENET_FPS)

        n_miss = df["lane_id"].isna().sum()
        if n_miss:
            print(f"  XAM-S9: warning — {n_miss} rows still missing frenet after time-align.")

        cx_px = df["x"].values + df["w"].values / 2.0
        cy_px = df["y"].values + df["h"].values / 2.0
        return self._assemble(df, cx_px, cy_px)

    def _process_merge_pixel_frenet(self, pixel_path: str) -> pd.DataFrame:
        build = self.cfg.build_frenet_path or self._default_build_frenet_path
        frenet_path = build(pixel_path, self.cfg)
        if not os.path.exists(frenet_path):
            raise FileNotFoundError(f"Frenet file not found: {frenet_path}")

        pix = self._load_ute_pixel_csv(pixel_path, self.cfg)

        fre = pd.read_csv(frenet_path)
        fre.columns = fre.columns.str.strip()
        fre = fre.rename(columns=self.cfg.frenet_rename)
        if self.cfg.frenet_dedupe_frame_car:
            fre = self._dedupe_frenet_frame_car(fre)

        df = pix.merge(
            fre[["frame_id", "car_id", "lane_id",
                 "lon_m", "lat_m", "speed_kmh", "length_m", "width_m"]],
            on=["frame_id", "car_id"],
            how="left",
        )
        df = df.sort_values(["car_id", "frame_id"]).reset_index(drop=True)

        if self.cfg.post_merge is not None:
            df = self.cfg.post_merge(df, self.cfg)

        cx_px = df["x"].values + df["w"].values / 2.0
        cy_px = df["y"].values + df["h"].values / 2.0
        return self._assemble(df, cx_px, cy_px)

    def _compute_heading(
        self,
        df: pd.DataFrame,
        cx_px: np.ndarray,
        cy_px: np.ndarray,
        max_frame_gap: int = 5,
    ) -> np.ndarray:
        vid = df["car_id"].values
        fnum = df["frame_id"].values
        n = len(df)
        heading = np.full(n, -1.0)

        for cid in df["car_id"].unique():
            mask = np.where(vid == cid)[0]
            if len(mask) == 1:
                continue

            segments, seg = [], [mask[0]]
            for k in range(1, len(mask)):
                if abs(fnum[mask[k]] - fnum[mask[k - 1]]) <= max_frame_gap:
                    seg.append(mask[k])
                else:
                    segments.append(seg)
                    seg = [mask[k]]
            segments.append(seg)

            for seg in segments:
                m = len(seg)
                if m == 1:
                    continue

                t = fnum[seg].astype(float)
                cx = cx_px[seg]
                cy = cy_px[seg]

                t0, t1 = t[0], t[-1]
                if t1 == t0:
                    continue
                tn = (t - t0) / (t1 - t0)

                deg = max(1, min(self.smooth_win - 1, m - 1, 3))
                px = np.polyfit(tn, cx, deg)
                py = np.polyfit(tn, cy, deg)
                dpx = np.polyder(px)
                dpy = np.polyder(py)

                for j, idx in enumerate(seg):
                    dx = np.polyval(dpx, tn[j])
                    dy = np.polyval(dpy, tn[j])
                    if dx == 0 and dy == 0:
                        continue
                    heading[idx] = np.degrees(np.arctan2(dy, dx)) % 360

        return heading

    @staticmethod
    def _oriented_bbox(cx, cy, length_px, width_px, heading_deg):
        l = length_px / 2
        w = width_px / 2
        theta = np.radians(heading_deg)
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        lc, ls = l * cos_t, l * sin_t
        wc, ws = w * cos_t, w * sin_t

        bb1x = cx + lc + ws; bb1y = cy + ls - wc
        bb2x = cx + lc - ws; bb2y = cy + ls + wc
        bb3x = cx - lc - ws; bb3y = cy - ls + wc
        bb4x = cx - lc + ws; bb4y = cy - ls - wc

        return bb1x, bb1y, bb2x, bb2y, bb3x, bb3y, bb4x, bb4y

    def _assemble(self, df: pd.DataFrame, cx_px: np.ndarray, cy_px: np.ndarray) -> pd.DataFrame:
        heading = self._compute_heading(df, cx_px, cy_px)

        p2m = self.pix2meter
        length_px = df["length_m"].values * p2m
        width_px = df["width_m"].values * p2m

        (bb1X, bb1Y, bb2X, bb2Y,
         bb3X, bb3Y, bb4X, bb4Y) = self._oriented_bbox(
            cx_px, cy_px, length_px, width_px, heading
        )

        carCenterXm = cx_px / p2m; carCenterYm = cy_px / p2m
        bb1Xm = bb1X / p2m; bb1Ym = bb1Y / p2m
        bb2Xm = bb2X / p2m; bb2Ym = bb2Y / p2m
        bb3Xm = bb3X / p2m; bb3Ym = bb3Y / p2m
        bb4Xm = bb4X / p2m; bb4Ym = bb4Y / p2m

        speed_ms = df["speed_kmh"].values / 3.6
        objClass = df["cls"].map(self.UTE_CLASS_MAP).fillna(-1).astype(int)

        return pd.DataFrame({
            "frameNum": df["frame_id"],
            "carId": df["car_id"],
            "laneId": df["lane_id"],
            "carCenterX": cx_px,
            "carCenterY": cy_px,
            "boundingBox1X": bb1X, "boundingBox1Y": bb1Y,
            "boundingBox2X": bb2X, "boundingBox2Y": bb2Y,
            "boundingBox3X": bb3X, "boundingBox3Y": bb3Y,
            "boundingBox4X": bb4X, "boundingBox4Y": bb4Y,
            "carCenterXm": carCenterXm, "carCenterYm": carCenterYm,
            "boundingBox1Xm": bb1Xm, "boundingBox1Ym": bb1Ym,
            "boundingBox2Xm": bb2Xm, "boundingBox2Ym": bb2Ym,
            "boundingBox3Xm": bb3Xm, "boundingBox3Ym": bb3Ym,
            "boundingBox4Xm": bb4Xm, "boundingBox4Ym": bb4Ym,
            "heading": heading,
            "course": -1,
            "speed": speed_ms,
            "objClass": objClass,
            "carCenterLon": -1,
            "carCenterLat": -1,
        })

    @staticmethod
    def _integerize_ute_csv_columns(df: pd.DataFrame) -> pd.DataFrame:
        out = df.copy()
        int_cols = (
            "frameNum", "carId", "laneId", "course", "objClass",
            "carCenterLon", "carCenterLat",
        )
        for c in int_cols:
            if c not in out.columns:
                continue
            s = pd.to_numeric(out[c], errors="coerce")
            if s.isna().any():
                continue
            out[c] = s.astype(np.int64)
        return out

    def _process_data(self, pixel_path: str) -> pd.DataFrame:
        return self._processor(self, pixel_path)


_cfg_ute = UTETransfer.LocationConfig

UTETransfer.LOCATION_CONFIG.update({
    "kzm6": _cfg_ute(
        pix2meter=14.5,
        pixel_suffix="KZM6pixel.csv",
        frenet_suffix="KZM6frenet.csv",
        pixel_rename={"time/s": "time", "class": "cls"},
        frenet_rename={
            "frame": "frame_id",
            "longitudinal distance<m>": "lon_m",
            "lateral distance<m>": "lat_m",
            "speed<km/h>": "speed_kmh",
            "acceleration<m/s2>": "accel",
            "length": "length_m",
            "width": "width_m",
        },
    ),
    "xam-n5": _cfg_ute(
        pix2meter=14.5,
        pixel_suffix="XAM-N5pixel.csv",
        frenet_suffix="XAM-N5frenet.csv",
        pixel_rename={"class": "cls"},
        frenet_rename={},
    ),
    "pkdd8": _cfg_ute(
        pix2meter=14.5,
        pixel_suffix="MOT格式图像坐标.csv",
        frenet_suffix="TJU8.csv",
        pixel_rename={"frame": "frame_id"},
        frenet_rename={
            "frame": "frame_id",
            "longitudinal distance<m>": "lon_m",
            "lateral distance<m>": "lat_m",
            "speed<km/h>": "speed_kmh",
            "acceleration<m/s2>": "accel",
            "length": "length_m",
            "width": "width_m",
        },
        pixel_default_cls=0,
    ),
    "xam-s9": _cfg_ute(
        pix2meter=10.72,
        pixel_suffix="UTE_KZM9.csv",
        frenet_suffix="KZM9frenet.csv",
        pixel_rename={"frame": "frame_id"},
        frenet_rename={
            "frame": "frame_id",
            "lane": "lane_id",
            "longitude": "lon_m",
            "latitude": "lat_m",
            "speed": "speed_kmh",
            "acceleration": "accel",
            "length": "length_m",
            "width": "width_m",
        },
        pixel_default_cls=0,
        geom_line_pos_filename="line_pos.npy",
    ),
    "rml7": _cfg_ute(
        pix2meter=10.72,
        pixel_suffix="MOTC-c.csv",
        frenet_suffix="RML7.csv",
        pixel_mot_no_header=True,
        pixel_dedupe_frame_car=True,
        frenet_dedupe_frame_car=True,
        pixel_rename={"frame": "frame_id"},
        frenet_rename={
            "frame": "frame_id",
            "longitude": "lon_m",
            "latitude": "lat_m",
            "speed<km/h>": "speed_kmh",
            "acceleration<m/s2>": "accel",
            "length": "length_m",
            "width": "width_m",
        },
        pixel_default_cls=0,
    ),
})

UTETransfer._PROCESSORS = {
    "kzm6": UTETransfer._process_merge_pixel_frenet,
    "pkdd8": UTETransfer._process_merge_pixel_frenet,
    "rml7": UTETransfer._process_merge_pixel_frenet,
    "xam-n5": UTETransfer._process_xamn5,
    "xam-s9": UTETransfer._process_xams9,
}


class WaymoTransfer(BasicTransfer):
    """Convert Waymo Motion Dataset (v1.3.0) TFRecord scenarios into the NBDT standard format.

    Waymo Motion does not provide a per-vehicle, per-timestep ``lane_id`` and has
    no closed lane polygons (unlike Argoverse 2). Lane association is therefore
    obtained by centerline matching: for each (vehicle, frame) the centre point
    is projected onto every ``LaneCenter.polyline`` segment, and the nearest
    lane within a 2 m distance threshold is taken as ``laneId``.
    """

    # Vehicles are matched to a lane only when the perpendicular distance from
    # the vehicle centre to the nearest LaneCenter polyline is <= sqrt(MAX_LANE_DIST2) m.
    MAX_LANE_DIST2 = 4.0  # i.e. 2.0 m
    # Drop polyline segments shorter than this (in m^2) to avoid divide-by-zero.
    MIN_SEG_LEN2 = 0.1
    # File prefix used by the public Waymo Motion v1.3.0 release.
    TFRECORD_PREFIX = "training_20s.tfrecord-"

    def __init__(self, args):
        super().__init__(args)

    def get_all_data(self) -> list:
        """Return all Waymo Motion TFRecord files under ``data_folder``.

        Accepts both the standard ``training_20s.tfrecord-*`` shards and any
        other ``*.tfrecord*`` files, sorted alphabetically for reproducibility.
        """
        file_names = sorted(os.listdir(self.args.data_folder))
        data_list = []
        for f in file_names:
            if f.startswith(self.TFRECORD_PREFIX) or ".tfrecord" in f:
                data_list.append(os.path.join(self.args.data_folder, f))
        return data_list

    @staticmethod
    def _build_lane_segments(scenario):
        """Concatenate every LaneCenter polyline in a scenario into flat segment arrays.

        Returns ``(seg_A, seg_AB, seg_denom, seg_lane_ids)`` or ``None`` when the
        scenario has no usable lane geometry. ``seg_A`` are segment start points,
        ``seg_AB`` are segment vectors, ``seg_denom`` is ``|AB|^2`` for fast
        projection, and ``seg_lane_ids`` carries the source lane id for each
        segment.
        """
        A_list, AB_list, lane_id_list = [], [], []
        for mf in scenario.map_features:
            if not mf.HasField("lane"):
                continue
            pts = mf.lane.polyline
            if len(pts) < 2:
                continue
            poly = np.fromiter(
                (c for p in pts for c in (p.x, p.y)),
                dtype=np.float64,
                count=2 * len(pts),
            ).reshape(-1, 2)
            A_list.append(poly[:-1])
            AB_list.append(poly[1:] - poly[:-1])
            lane_id_list.append(np.full(len(poly) - 1, mf.id, dtype=np.int64))

        if not A_list:
            return None

        seg_A = np.concatenate(A_list)
        seg_AB = np.concatenate(AB_list)
        seg_denom = (seg_AB * seg_AB).sum(axis=1)
        seg_lane_ids = np.concatenate(lane_id_list)

        ok = seg_denom > WaymoTransfer.MIN_SEG_LEN2
        if not np.any(ok):
            return None

        return seg_A[ok], seg_AB[ok], seg_denom[ok], seg_lane_ids[ok]

    @staticmethod
    def _match_lane(cx, cy, seg_arrays, max_dist2=MAX_LANE_DIST2):
        """Vectorised nearest-lane lookup for a single point ``(cx, cy)``.

        Returns the lane id of the closest LaneCenter segment whose squared
        perpendicular distance is at most ``max_dist2``; otherwise ``-1``.
        """
        seg_A, seg_AB, seg_denom, seg_lane_ids = seg_arrays
        ap_x = cx - seg_A[:, 0]
        ap_y = cy - seg_A[:, 1]
        t = (ap_x * seg_AB[:, 0] + ap_y * seg_AB[:, 1]) / seg_denom
        valid_t = (t >= 0.0) & (t <= 1.0)
        if not np.any(valid_t):
            return -1
        seg_A_v = seg_A[valid_t]
        seg_AB_v = seg_AB[valid_t]
        t_v = t[valid_t]
        lane_ids_v = seg_lane_ids[valid_t]

        proj_x = seg_A_v[:, 0] + t_v * seg_AB_v[:, 0]
        proj_y = seg_A_v[:, 1] + t_v * seg_AB_v[:, 1]
        dx = cx - proj_x
        dy = cy - proj_y
        d2 = dx * dx + dy * dy

        best = int(np.argmin(d2))
        if d2[best] <= max_dist2:
            return int(lane_ids_v[best])
        return -1

    def _extract_track_records(self, scenario, tf_file: str) -> list:
        """Flatten one Waymo ``Scenario`` proto into a list of per-frame records."""
        records = []
        egoid = scenario.tracks[scenario.sdc_track_index].id

        seg_arrays = self._build_lane_segments(scenario)
        has_lanes = seg_arrays is not None

        for track in scenario.tracks:
            obj_id = track.id
            obj_type = track.object_type  # 1=VEHICLE, 2=PED, 3=CYCLIST, 4=OTHER
            # vtype: 1=HDV, 2=AV (ego), 3=others (non-vehicles)
            vtype = 3
            if obj_type == 1:
                vtype = 2 if obj_id == egoid else 1

            for j, state in enumerate(track.states):
                if not state.valid:
                    continue
                cx = state.center_x
                cy = state.center_y

                lane_id = -1
                if obj_type == 1 and has_lanes:
                    lane_id = self._match_lane(cx, cy, seg_arrays)

                heading = state.heading
                half_l = state.length / 2.0
                half_w = state.width / 2.0
                cos_h = math.cos(heading)
                sin_h = math.sin(heading)

                # Vehicle local frame: x forward, y left.
                # BB1 front-right, BB2 rear-right, BB3 rear-left, BB4 front-left.
                corners_local = (
                    ( half_l, -half_w),
                    (-half_l, -half_w),
                    (-half_l,  half_w),
                    ( half_l,  half_w),
                )
                corners_world = [
                    (cx + lx * cos_h - ly * sin_h,
                     cy + lx * sin_h + ly * cos_h)
                    for lx, ly in corners_local
                ]

                records.append({
                    "scenarioId": scenario.scenario_id,
                    "tfFile": tf_file,
                    "frameNum": j,
                    "carId": obj_id,
                    "laneId": lane_id,
                    "carCenterXm": cx,
                    "carCenterYm": cy,
                    "carCenterX": -1,
                    "carCenterY": -1,
                    "velocity_x": state.velocity_x,
                    "velocity_y": state.velocity_y,
                    "speed": (state.velocity_x ** 2 + state.velocity_y ** 2) ** 0.5,
                    "heading": math.degrees(heading) % 360.0,
                    "course": (90.0 - math.degrees(heading)) % 360.0,
                    "length": state.length,
                    "width": state.width,
                    "boundingBox1Xm": corners_world[0][0],
                    "boundingBox1Ym": corners_world[0][1],
                    "boundingBox2Xm": corners_world[1][0],
                    "boundingBox2Ym": corners_world[1][1],
                    "boundingBox3Xm": corners_world[2][0],
                    "boundingBox3Ym": corners_world[2][1],
                    "boundingBox4Xm": corners_world[3][0],
                    "boundingBox4Ym": corners_world[3][1],
                    "boundingBox1X": -1,
                    "boundingBox1Y": -1,
                    "boundingBox2X": -1,
                    "boundingBox2Y": -1,
                    "boundingBox3X": -1,
                    "boundingBox3Y": -1,
                    "boundingBox4X": -1,
                    "boundingBox4Y": -1,
                    "objClass": -1,
                    # 0=UNSET, 1=VEHICLE, 2=PEDESTRIAN, 3=CYCLIST, 4=OTHER
                    "obj_type": obj_type,
                    # 1=HDV, 2=AV (ego data collector), 3=others
                    "vtype": vtype,
                    "carCenterLon": -1,
                    "carCenterLat": -1,
                })

        return records

    def _process_data(self, file_path: str) -> pd.DataFrame:
        # Heavy deps are imported lazily so non-Waymo users do not need them.
        import tensorflow as tf
        from waymo_open_dataset.protos import scenario_pb2

        tf_file = os.path.basename(file_path)
        dataset = tf.data.TFRecordDataset(file_path, compression_type="")

        all_records = []
        for raw_record in dataset:
            scenario = scenario_pb2.Scenario()
            scenario.ParseFromString(raw_record.numpy())
            recs = self._extract_track_records(scenario, tf_file)
            if recs:
                all_records.extend(recs)

        print(f"  Waymo: {tf_file} -> {len(all_records)} records")
        return pd.DataFrame(all_records)
