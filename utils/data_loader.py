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
    L = 0.5 * (math.sqrt((row['boundingBox1Xm'] - row['boundingBox2Xm'])**2 + (row['boundingBox1Ym'] - row['boundingBox2Ym'])**2)
               + math.sqrt((row['boundingBox3Xm'] - row['boundingBox4Xm'])**2 + (row['boundingBox3Ym'] - row['boundingBox4Ym'])**2))
    W = 0.5 * (math.sqrt((row['boundingBox1Xm'] - row['boundingBox4Xm'])**2 + (row['boundingBox1Ym'] - row['boundingBox4Ym'])**2)
               + math.sqrt((row['boundingBox2Xm'] - row['boundingBox3Xm'])**2 + (row['boundingBox2Ym'] - row['boundingBox3Ym'])**2))
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
    """加载数据，预处理，提供查询接口"""
    def __init__(self, meta_file, tracks_file):
        self.meta_file = meta_file
        self.tracks_file = tracks_file
        self.frame_rate, self.upper_marks, self.lower_marks = load_metadata(meta_file)
        self.dt = 1.0 / self.frame_rate
        self.df_tracks = load_tracks(tracks_file)
        self.frames = sorted(self.df_tracks['frameNum'].unique())

        # 存储每帧的车辆信息
        self.frame_vehicles = {}          # frame -> {car_id: info}
        # 存储每帧每个车道的排序车辆ID列表和车道方向
        self.frame_lane_vehicles = {}     # frame -> {lane_id: [car_id, ...]} 按投影距离排序
        self.frame_lane_direction = {}    # frame -> {lane_id: direction_rad}

        self._preprocess()
        self.calculator = SSMCalculator()  # 新增：创建SSM计算器实例

    def _preprocess(self):
        """逐帧预处理，计算投影并建立索引"""
        print("Preprocessing data...")
        # 第一遍：收集原始信息
        for frame in self.frames:
            df_frame = self.df_tracks[self.df_tracks['frameNum'] == frame]
            vehicles = {}
            for _, row in df_frame.iterrows():
                carId = int(row['carId'])
                x = row['carCenterXm']
                y = row['carCenterYm']
                speed = row['speed']
                heading = row['heading']
                vx, vy = get_velocity_components(speed, heading)
                L, W = compute_vehicle_dimensions(row)
                laneId = row['laneId'] if pd.notna(row['laneId']) else -1
                obj_class = int(row['objClass']) if pd.notna(row['objClass']) else -1

                bbox = [
                    (row['boundingBox1Xm'], row['boundingBox1Ym']),
                    (row['boundingBox2Xm'], row['boundingBox2Ym']),
                    (row['boundingBox3Xm'], row['boundingBox3Ym']),
                    (row['boundingBox4Xm'], row['boundingBox4Ym'])
                ]

                # 加速度和角速度暂时设为0，后面统一计算
                a = 0.0
                angle_speed = 0.0

                info = {
                    'x': x, 'y': y, 'vx': vx, 'vy': vy, 'L': L, 'W': W,
                    'laneId': laneId, 'class': obj_class, 'heading': heading,
                    'speed': speed, 'a': a, 'angle_speed': angle_speed,
                    'bbox': bbox
                }
                vehicles[carId] = info
            self.frame_vehicles[frame] = vehicles

        # 第二遍：计算加速度和角速度（基于前后帧）
        for frame in self.frames:
            if frame == 0:
                continue
            prev_vehicles = self.frame_vehicles.get(frame-1, {})
            curr_vehicles = self.frame_vehicles[frame]
            for cid, info in curr_vehicles.items():
                if cid in prev_vehicles:
                    prev = prev_vehicles[cid]
                    dv = info['speed'] - prev['speed']
                    info['a'] = dv / self.dt
                    d_heading = info['heading'] - prev['heading']
                    if d_heading > 180:
                        d_heading -= 360
                    elif d_heading < -180:
                        d_heading += 360
                    info['angle_speed'] = d_heading / self.dt

        # 第三遍：按车道分组，估计方向，计算投影
        for frame in self.frames:
            vehicles = self.frame_vehicles[frame]
            lane_groups = defaultdict(list)
            for cid, info in vehicles.items():
                lane = info['laneId']
                lane_groups[lane].append((cid, info))

            lane_directions = {}
            lane_sorted_lists = defaultdict(list)

            for lane, group in lane_groups.items():
                # 估计车道方向
                heading_list = [info['heading'] for _, info in group]
                dir_rad = self._estimate_direction_from_headings(heading_list)
                if dir_rad is None:
                    continue
                lane_directions[lane] = dir_rad
                cosθ, sinθ = np.cos(dir_rad), np.sin(dir_rad)

                # 计算每个车辆的投影距离、速度、加速度
                proj_list = []
                for cid, info in group:
                    proj_dist = info['x'] * cosθ + info['y'] * sinθ
                    proj_speed = info['vx'] * cosθ + info['vy'] * sinθ
                    delta = np.radians(info['heading']) - dir_rad
                    proj_acc = info['a'] * np.cos(delta)
                    info['proj_dist'] = proj_dist
                    info['proj_speed'] = proj_speed
                    info['proj_acc'] = proj_acc
                    proj_list.append((proj_dist, cid))

                proj_list.sort()
                lane_sorted_lists[lane] = [cid for _, cid in proj_list]

            self.frame_lane_vehicles[frame] = dict(lane_sorted_lists)
            self.frame_lane_direction[frame] = lane_directions

        print("Preprocessing done.")

    def _estimate_direction_from_headings(self, headings):
        """从航向列表估计平均方向（弧度）"""
        if len(headings) < 2:
            return None
        sum_cos = 0.0
        sum_sin = 0.0
        for h in headings:
            rad = np.radians(h)
            sum_cos += np.cos(rad)
            sum_sin += np.sin(rad)
        mean_cos = sum_cos / len(headings)
        mean_sin = sum_sin / len(headings)
        return np.arctan2(mean_sin, mean_cos)

    def get_vehicle_info(self, frame, car_id):
        """返回指定帧的车辆信息字典，如果不存在返回 None"""
        if frame in self.frame_vehicles and car_id in self.frame_vehicles[frame]:
            return self.frame_vehicles[frame][car_id]
        return None

    def get_surrounding_vehicles(self, frame, ego_id):
        """
        返回主车周围最多6辆车的列表，每个元素为 (target_id, relation)
        relation 取值为 'front', 'rear', 'left_front', 'left_rear', 'right_front', 'right_rear'
        """
        if frame not in self.frame_vehicles or ego_id not in self.frame_vehicles[frame]:
            return []
        ego_info = self.frame_vehicles[frame][ego_id]
        ego_lane = ego_info['laneId']
        if ego_lane not in self.frame_lane_vehicles[frame]:
            return []

        lane_list = self.frame_lane_vehicles[frame][ego_lane]
        try:
            idx = lane_list.index(ego_id)
        except ValueError:
            return []

        surrounding = []

        # 同一车道的前后
        # if idx > 0:
        #     surrounding.append((lane_list[idx-1], 'rear'))
        if idx < len(lane_list)-1:
            surrounding.append((lane_list[idx+1], 'front'))

        # 左右车道
        ego_dir = self.frame_lane_direction[frame].get(ego_lane)
        if ego_dir is None:
            return surrounding

        # 左车道（laneId - 1）
        left_lane = ego_lane - 1
        if left_lane in self.frame_lane_vehicles[frame]:
            left_dir = self.frame_lane_direction[frame].get(left_lane)
            if left_dir is not None and abs(angle_diff(np.degrees(ego_dir), np.degrees(left_dir))) < 45:  # 同向阈值45度
                left_list = self.frame_lane_vehicles[frame][left_lane]
                ego_proj = ego_info['proj_dist']
                proj_left = [self.frame_vehicles[frame][cid]['proj_dist'] for cid in left_list]
                pos = bisect.bisect_left(proj_left, ego_proj)
                # if pos > 0:
                #     surrounding.append((left_list[pos-1], 'left_rear'))
                if pos < len(left_list):
                    surrounding.append((left_list[pos], 'left_front'))

        # 右车道（laneId + 1）
        right_lane = ego_lane + 1
        if right_lane in self.frame_lane_vehicles[frame]:
            right_dir = self.frame_lane_direction[frame].get(right_lane)
            if right_dir is not None and abs(angle_diff(np.degrees(ego_dir), np.degrees(right_dir))) < 45:
                right_list = self.frame_lane_vehicles[frame][right_lane]
                ego_proj = ego_info['proj_dist']
                proj_right = [self.frame_vehicles[frame][cid]['proj_dist'] for cid in right_list]
                pos = bisect.bisect_left(proj_right, ego_proj)
                # if pos > 0:
                #     surrounding.append((right_list[pos-1], 'right_rear'))
                if pos < len(right_list):
                    surrounding.append((right_list[pos], 'right_front'))

        return surrounding

    def get_frame_range(self):
        """返回所有帧列表"""
        return self.frames

    def get_upper_lower_marks(self):
        return self.upper_marks, self.lower_marks

    # ===== 计算两车之间的SSM指标 =====
    def compute_ssm_for_pair(self, frame, id1, id2):
        """计算两车在指定帧的SSM指标"""
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
                same_lane = False  # 无法确定方向，则视为不同车道
            else:
                proj_dist1 = v1.get('proj_dist')
                proj_dist2 = v2.get('proj_dist')
                proj_speed1 = v1.get('proj_speed')
                proj_speed2 = v2.get('proj_speed')
                proj_acc1 = v1.get('proj_acc')
                proj_acc2 = v2.get('proj_acc')

        # 使用内部 calculator 计算
        result = self.calculator.compute_for_pair(
            v1, v2,
            same_lane=same_lane,
            proj_dist1=proj_dist1, proj_dist2=proj_dist2,
            proj_speed1=proj_speed1, proj_speed2=proj_speed2,
            proj_acc1=proj_acc1, proj_acc2=proj_acc2,
            lane_dir_rad=lane_dir
        )
        return result