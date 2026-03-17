import os
import math
import pandas as pd
import numpy as np
from collections import defaultdict
import bisect

from box_distance import calculate_nearest_points, order_rect_points
from ssm_core import SSMCalculator

def load_metadata(meta_file):
    df = pd.read_csv(meta_file)
    row = df.iloc[0]
    frame_rate = row['frameRate']
    upper_marks = [float(x) for x in str(row['upperLaneMarkings']).split(';')] if pd.notna(row['upperLaneMarkings']) else []
    lower_marks = [float(x) for x in str(row['lowerLaneMarkings']).split(';')] if pd.notna(row['lowerLaneMarkings']) else []
    return frame_rate, upper_marks, lower_marks

def load_tracks(tracks_file):
    df = pd.read_csv(tracks_file)
    numeric_cols = ['frameNum', 'carId', 'carCenterX', 'carCenterY',
                    'boundingBox1X', 'boundingBox1Y', 'boundingBox2X', 'boundingBox2Y',
                    'boundingBox3X', 'boundingBox3Y', 'boundingBox4X', 'boundingBox4Y',
                    'carCenterXm', 'carCenterYm',
                    'boundingBox1Xm', 'boundingBox1Ym', 'boundingBox2Xm', 'boundingBox2Ym',
                    'boundingBox3Xm', 'boundingBox3Ym', 'boundingBox4Xm', 'boundingBox4Ym',
                    'heading', 'course', 'speed', 'objClass', 'carCenterLon', 'carCenterLat', 'laneId']
    for col in numeric_cols:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors='coerce')
    return df

def compute_vehicle_dimensions(row):
    """计算车辆的长度和宽度"""
    L = 0.5 * (np.sqrt((row['boundingBox1Xm'] - row['boundingBox2Xm'])**2 + (row['boundingBox1Ym'] - row['boundingBox2Ym'])**2)
               + np.sqrt((row['boundingBox3Xm'] - row['boundingBox4Xm'])**2 + (row['boundingBox3Ym'] - row['boundingBox4Ym'])**2))
    W = 0.5 * (np.sqrt((row['boundingBox1Xm'] - row['boundingBox4Xm'])**2 + (row['boundingBox1Ym'] - row['boundingBox4Ym'])**2)
               + np.sqrt((row['boundingBox2Xm'] - row['boundingBox3Xm'])**2 + (row['boundingBox2Ym'] - row['boundingBox3Ym'])**2))
    return L, W

def get_velocity_components(speed, heading_deg):
    rad = np.radians(heading_deg)
    vx = speed * np.cos(rad)
    vy = speed * np.sin(rad)
    return vx, vy

def angle_diff(a1, a2):
    diff = abs(a1 - a2) % 360
    if diff > 180:
        diff = 360 - diff
    return diff

class SafetyAnalyzer:
    def __init__(self, meta_file, tracks_file):
        self.meta_file = meta_file
        self.tracks_file = tracks_file
        self.frame_rate, self.upper_marks, self.lower_marks = load_metadata(meta_file)
        self.dt = 1.0 / self.frame_rate
        self.df_tracks = load_tracks(tracks_file)
        self.frames = sorted(self.df_tracks['frameNum'].unique())

        # 存储预处理后的数据
        self.frame_vehicles = {}          # frame -> {car_id: info}
        self.frame_lane_vehicles = {}     # frame -> {lane_id: [car_id, ...]} 按投影距离排序
        self.frame_lane_direction = {}    # frame -> {lane_id: direction_rad}

        self._preprocess_pandas()
        self.calculator = SSMCalculator()

    def _preprocess_pandas(self):
        print("Preprocessing data")
        df = self.df_tracks.copy()

        df['x'] = df['carCenterXm']
        df['y'] = df['carCenterYm']

        L, W = compute_vehicle_dimensions(df)
        df['L'] = L
        df['W'] = W
        rad = np.radians(df['heading'])
        df['vx'] = df['speed'] * np.cos(rad)
        df['vy'] = df['speed'] * np.sin(rad)

        df = df.sort_values(['carId', 'frameNum']).reset_index(drop=True)

        # 计算加速度和角速度
        df['frame_diff'] = df.groupby('carId')['frameNum'].diff()
        df['speed_diff'] = df.groupby('carId')['speed'].diff()
        df['heading_diff'] = df.groupby('carId')['heading'].diff()

        # 处理角度跳变
        heading_diff = df['heading_diff'].copy()
        mask_gt = heading_diff > 180
        mask_lt = heading_diff < -180
        heading_diff[mask_gt] -= 360
        heading_diff[mask_lt] += 360

        df['a'] = np.where(df['frame_diff'] == 1, df['speed_diff'] / self.dt, 0.0)
        df['angle_speed'] = np.where(df['frame_diff'] == 1, heading_diff / self.dt, 0.0)

        # 第一帧的加速度/角速度默认为0
        df.drop(['frame_diff', 'speed_diff', 'heading_diff'], axis=1, inplace=True)

        # 按帧和车道分组估计车道方向
        def lane_direction(group):
            headings = group['heading'].values
            if len(headings) < 2:
                return np.nan
            sum_cos = np.sum(np.cos(np.radians(headings)))
            sum_sin = np.sum(np.sin(np.radians(headings)))
            return np.arctan2(sum_sin, sum_cos)

        lane_dir_df = df.groupby(['frameNum', 'laneId']).apply(lane_direction).reset_index()
        lane_dir_df.columns = ['frameNum', 'laneId', 'lane_dir_rad']
        # 将车道方向合并回原数据
        df = df.merge(lane_dir_df, on=['frameNum', 'laneId'], how='left')

        # 计算投影距离、速度、加速度
        mask = df['lane_dir_rad'].notna()
        cosθ = np.cos(df.loc[mask, 'lane_dir_rad'])
        sinθ = np.sin(df.loc[mask, 'lane_dir_rad'])
        df.loc[mask, 'proj_dist'] = df.loc[mask, 'x'] * cosθ + df.loc[mask, 'y'] * sinθ
        df.loc[mask, 'proj_speed'] = df.loc[mask, 'vx'] * cosθ + df.loc[mask, 'vy'] * sinθ
        # 计算投影加速度：需要 heading 与车道方向的夹角
        delta = np.radians(df.loc[mask, 'heading']) - df.loc[mask, 'lane_dir_rad']
        df.loc[mask, 'proj_acc'] = df.loc[mask, 'a'] * np.cos(delta)

        # 构建 frame_vehicles 和 frame_lane_* 字典
        for frame, group in df.groupby('frameNum'):
            veh_dict = {}
            lane_dict = defaultdict(list)
            lane_dir_dict = {}

            # 按 laneId 分组，用于构建排序列表
            for lane, lane_group in group.groupby('laneId'):
                # 按投影距离排序
                if lane_group['proj_dist'].notna().any():
                    lane_group_sorted = lane_group.sort_values('proj_dist')
                    car_ids = lane_group_sorted['carId'].tolist()
                    lane_dict[lane] = car_ids
                else:
                    lane_dict[lane] = lane_group['carId'].tolist()
                # 车道方向（取该车道第一个有效值）
                lane_dir = lane_group['lane_dir_rad'].iloc[0]
                if not pd.isna(lane_dir):
                    lane_dir_dict[lane] = lane_dir

            # 构建每辆车的 info 字典
            for _, row in group.iterrows():
                carId = int(row['carId'])
                info = {
                    'x': row['x'], 'y': row['y'],
                    'vx': row['vx'], 'vy': row['vy'],
                    'L': row['L'], 'W': row['W'],
                    'laneId': int(row['laneId']) if not pd.isna(row['laneId']) else -1,
                    'class': int(row['objClass']) if not pd.isna(row['objClass']) else -1,
                    'heading': row['heading'],
                    'speed': row['speed'],
                    'a': row['a'],
                    'angle_speed': row['angle_speed'],
                    'bbox': [
                        (row['boundingBox1Xm'], row['boundingBox1Ym']),
                        (row['boundingBox2Xm'], row['boundingBox2Ym']),
                        (row['boundingBox3Xm'], row['boundingBox3Ym']),
                        (row['boundingBox4Xm'], row['boundingBox4Ym'])
                    ],
                    'proj_dist': row.get('proj_dist', None),
                    'proj_speed': row.get('proj_speed', None),
                    'proj_acc': row.get('proj_acc', None)
                }
                veh_dict[carId] = info

            self.frame_vehicles[frame] = veh_dict
            self.frame_lane_vehicles[frame] = dict(lane_dict)
            self.frame_lane_direction[frame] = lane_dir_dict

        print("Preprocessing done.")

    def get_vehicle_info(self, frame, car_id):
        if frame in self.frame_vehicles and car_id in self.frame_vehicles[frame]:
            return self.frame_vehicles[frame][car_id]
        return None

    def get_surrounding_vehicles(self, frame, ego_id):
        if frame not in self.frame_vehicles or ego_id not in self.frame_vehicles[frame]:
            return []
        ego_info = self.frame_vehicles[frame][ego_id]
        ego_lane = ego_info['laneId']
        if ego_lane not in self.frame_lane_vehicles.get(frame, {}):
            return []

        lane_list = self.frame_lane_vehicles[frame][ego_lane]
        try:
            idx = lane_list.index(ego_id)
        except ValueError:
            return []

        surrounding = []
        # 同车道前方
        if idx < len(lane_list)-1:
            surrounding.append((lane_list[idx+1], 'front'))

        ego_dir = self.frame_lane_direction[frame].get(ego_lane)
        if ego_dir is None:
            return surrounding

        # 左车道
        left_lane = ego_lane - 1
        if left_lane in self.frame_lane_vehicles.get(frame, {}):
            left_dir = self.frame_lane_direction[frame].get(left_lane)
            if left_dir is not None and abs(angle_diff(np.degrees(ego_dir), np.degrees(left_dir))) < 45:
                left_list = self.frame_lane_vehicles[frame][left_lane]
                ego_proj = ego_info['proj_dist']
                proj_left = [self.frame_vehicles[frame][cid]['proj_dist'] for cid in left_list]
                pos = bisect.bisect_left(proj_left, ego_proj)
                if pos < len(left_list):
                    surrounding.append((left_list[pos], 'left_front'))

        # 右车道
        right_lane = ego_lane + 1
        if right_lane in self.frame_lane_vehicles.get(frame, {}):
            right_dir = self.frame_lane_direction[frame].get(right_lane)
            if right_dir is not None and abs(angle_diff(np.degrees(ego_dir), np.degrees(right_dir))) < 45:
                right_list = self.frame_lane_vehicles[frame][right_lane]
                ego_proj = ego_info['proj_dist']
                proj_right = [self.frame_vehicles[frame][cid]['proj_dist'] for cid in right_list]
                pos = bisect.bisect_left(proj_right, ego_proj)
                if pos < len(right_list):
                    surrounding.append((right_list[pos], 'right_front'))

        return surrounding

    def get_frame_range(self):
        return self.frames

    def get_upper_lower_marks(self):
        return self.upper_marks, self.lower_marks

    def compute_ssm_for_pair(self, frame, id1, id2):
        v1 = self.get_vehicle_info(frame, id1)
        v2 = self.get_vehicle_info(frame, id2)
        if v1 is None or v2 is None:
            return None

        same_lane = (v1['laneId'] == v2['laneId'])
        lane_dir = None
        proj_dist1 = proj_dist2 = proj_speed1 = proj_speed2 = proj_acc1 = proj_acc2 = None
        if same_lane:
            lane = v1['laneId']
            lane_dir = self.frame_lane_direction[frame].get(lane)
            if lane_dir is None:
                same_lane = False
            else:
                proj_dist1 = v1.get('proj_dist')
                proj_dist2 = v2.get('proj_dist')
                proj_speed1 = v1.get('proj_speed')
                proj_speed2 = v2.get('proj_speed')
                proj_acc1 = v1.get('proj_acc')
                proj_acc2 = v2.get('proj_acc')

        result = self.calculator.compute_for_pair(
            v1, v2,
            same_lane=same_lane,
            proj_dist1=proj_dist1, proj_dist2=proj_dist2,
            proj_speed1=proj_speed1, proj_speed2=proj_speed2,
            proj_acc1=proj_acc1, proj_acc2=proj_acc2,
            lane_dir_rad=lane_dir
        )
        return result