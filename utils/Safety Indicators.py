import os
import math
from collections import defaultdict

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from box_distance import calculate_nearest_points, order_rect_points

# ========================= 配置参数 =========================
# 通用
TTC_THRESHOLD = 5               # TTC阈值（秒），用于TIT/TET
MIN_VEHICLES_FOR_LANE_DIR = 2    # 至少需要多少辆车才能估计车道方向

# 车辆最大可用减速度 (m/s²) 用于 CPI
MADR_CAR = 3
MADR_TAXI = 3
MADR_TRUCK = 2
MADR_BUS = 2
MADR_MOTO = 4
MADR_PED = 10.0
MADR_UNKNOWN = 3
VEHICLE_CLASS_MADR = {
    0: MADR_CAR,      # car
    1: MADR_TAXI,     # taxi
    2: MADR_BUS,      # bus
    3: MADR_TRUCK,    # truck
    4: MADR_MOTO,     # motorcycle
    5: MADR_PED,      # pedestrian
    -1: MADR_UNKNOWN   # unknown
}

# 小值打印阈值
PET_PRINT_THRESHOLD = 0.2        # PET 小于此值打印
TTC2D_PRINT_THRESHOLD = 0.3      # 2D TTC 小于此值打印

# 同向判断阈值（用于2D TTC）
HEADING_DIFF_THRESHOLD = 45      # 航向差小于此值视为同向

# PET 角度范围（只有在此范围内才计算PET，排除同向和对向）
PET_MIN_ANGLE = 10
PET_MAX_ANGLE = 150

# 可视化开关
ENABLE_VISUALIZATION = False
VISUALIZATION_OUTPUT_DIR = "./visualization"
SHOW_VISUALIZATION = False
SAVE_VISUALIZATION = True
# ============================================================


# -------------------- 数据加载与预处理 --------------------
def load_metadata(meta_file):
    """读取元数据，返回帧率和车道标记"""
    df = pd.read_csv(meta_file)
    row = df.iloc[0]
    frame_rate = row['frameRate']
    upper_marks = [float(x) for x in str(row['upperLaneMarkings']).split(';')] if pd.notna(row['upperLaneMarkings']) else []
    lower_marks = [float(x) for x in str(row['lowerLaneMarkings']).split(';')] if pd.notna(row['lowerLaneMarkings']) else []
    return frame_rate, upper_marks, lower_marks


def load_tracks(tracks_file):
    """读取轨迹数据，返回DataFrame"""
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
    """
    根据boundingBox计算车辆在X和Y方向的尺寸（米）。
    采用平面直角坐标系下两点间距离的计算公式。
    """
    L = 0.5 * (math.sqrt((row['boundingBox1Xm'] - row['boundingBox2Xm'])**2 + (row['boundingBox1Ym'] - row['boundingBox2Ym'])**2)
               + math.sqrt((row['boundingBox3Xm'] - row['boundingBox4Xm'])**2 + (row['boundingBox3Ym'] - row['boundingBox4Ym'])**2))
    W = 0.5 * (math.sqrt((row['boundingBox1Xm'] - row['boundingBox4Xm'])**2 + (row['boundingBox1Ym'] - row['boundingBox4Ym'])**2)
               + math.sqrt((row['boundingBox2Xm'] - row['boundingBox3Xm'])**2 + (row['boundingBox2Ym'] - row['boundingBox3Ym'])**2))
    return L, W


def get_velocity_components(speed, heading_deg):
    """
    根据速度大小和航向（相对于图像X轴，度）计算vx, vy。
    heading=0时车辆沿X轴正向（向右）。
    """
    rad = np.radians(heading_deg)
    vx = speed * np.cos(rad)
    vy = speed * np.sin(rad)
    return vx, vy


def estimate_lane_direction(vehicles_in_lane):
    """
    根据同一车道内所有车辆的heading估计车道的主方向（度）。
    使用向量平均法，避免角度循环问题。
    返回角度（度），若无法估计则返回None。
    """
    if len(vehicles_in_lane) < MIN_VEHICLES_FOR_LANE_DIR:
        return None
    sum_cos = 0.0
    sum_sin = 0.0
    for v in vehicles_in_lane:
        rad = np.radians(v['heading'])
        sum_cos += np.cos(rad)
        sum_sin += np.sin(rad)
    mean_cos = sum_cos / len(vehicles_in_lane)
    mean_sin = sum_sin / len(vehicles_in_lane)
    mean_rad = np.arctan2(mean_sin, mean_cos)
    mean_deg = np.degrees(mean_rad)
    mean_deg = mean_deg % 360
    return mean_deg


# -------------------- 一维安全指标计算 --------------------
def ttc_from_gap_and_dv(gap, dv):
    """根据间隙和相对速度计算TTC（恒速假设）"""
    if gap <= 0 or dv <= 0:
        return np.inf
    return gap / dv


def drac_from_gap_and_dv(gap, dv):
    """根据间隙和相对速度计算DRAC"""
    if gap <= 0 or dv <= 0:
        return np.inf
    return dv**2 / (2.0 * gap)


def compute_mttc(gap, dv, da):
    """
    根据间隙gap、相对速度dv、相对加速度da计算MTTC。
    公式：MTTC = ( -dv ± sqrt(dv^2 + 2*da*gap) ) / da   (da != 0)
    若da=0，则退化为TTC公式。
    返回最小正时间，若无解则返回inf。
    """
    if abs(da) < 1e-6:
        if dv <= 0 or gap <= 0:
            return np.inf
        return gap / dv
    discriminant = dv**2 + 2 * da * gap
    if discriminant < 0:
        return np.inf
    sqrt_disc = math.sqrt(discriminant)
    t1 = (-dv - sqrt_disc) / da
    t2 = (-dv + sqrt_disc) / da
    candidates = [t for t in (t1, t2) if t > 1e-6]
    if not candidates:
        return np.inf
    return min(candidates)


def compute_cai(v_f, a_f, v_l, a_l, mttc):
    """
    计算冲突指数 CAI
    CAI = 0.5 * ((v_f + a_f * MTTC) ** 2 - (v_l + a_l * MTTC) ** 2) / MTTC
    """
    if np.isinf(mttc) or mttc <= 0:
        return np.inf
    term_f = v_f + a_f * mttc
    term_l = v_l + a_l * mttc
    return 0.5 * (term_f**2 - term_l**2) / mttc


# -------------------- 二维安全指标（ACT） --------------------
class ACT:
    """用于计算二维碰撞时间（ACT）的类"""
    def __init__(self):
        self.show_collision = False

    def project_to_line(self, v, a, theta, alpha):
        delta = theta - alpha
        v_proj = v * np.cos(delta)
        a_proj = a * np.cos(delta)
        return v_proj, a_proj

    def get_act(self, v, a, w, distance, scale=1.0):
        """
        求解一元二次方程：0.5 * a * t^2 + (v + w * distance) * t - distance = 0
        在(0, +inf)区间的解。
        """
        distance = distance * scale
        if a == 0:
            x = distance / (v + w * distance)
            if x > 0:
                return x
            else:
                return np.inf
        ass = (v + w * distance) ** 2 + 2 * a * distance
        if ass < 0:
            return np.inf
        else:
            x1 = (-v - w * distance + np.sqrt(ass)) / a
            x2 = (-v - w * distance - np.sqrt(ass)) / a
            if x1 > 0 and x2 > 0:
                return min(x1, x2)
            elif x1 > 0 and x2 < 0:
                return x1
            elif x1 < 0 and x2 > 0:
                return x2
            else:
                return np.inf

    def get_direction_vector(self, point1, point2):
        direction_vector = np.array(point2) - np.array(point1)
        return math.atan2(direction_vector[1], direction_vector[0])

    def run(self, veh1, param1, veh2, param2, scale=1.0):
        """
        计算两车之间的预期碰撞时间。
        参数:
            veh1, veh2: 四个顶点坐标的列表
            param1, param2: (speed, acceleration, heading_deg, angular_speed_deg)
        返回:
            act (秒)
        """
        point1, point2, distance = calculate_nearest_points(veh1, veh2)
        alpha = self.get_direction_vector(point1, point2)
        v1, a1 = self.project_to_line(param1[0], param1[1], np.radians(param1[2]), alpha)
        v2, a2 = self.project_to_line(param2[0], param2[1], np.radians(param2[2]), alpha)
        v = v1 - v2
        a = a1 - a2
        w = np.radians(param1[3] + param2[3])
        act = self.get_act(v, a, w, distance, scale)
        return act


# -------------------- PET 计算 --------------------
def compute_pet(veh1, param1, veh2, param2, angle_diff):
    """
    计算两车基于直线轨迹的 PET（后侵入时间）
    只有当 angle_diff 在 [PET_MIN_ANGLE, PET_MAX_ANGLE] 范围内时才计算。
    参数:
        veh1, veh2: 四个顶点坐标的列表
        param1, param2: (speed, acceleration, heading_deg, angular_speed_deg)
        angle_diff: 两车航向差（度）
    返回:
        PET (秒) 或 np.inf
    """
    if angle_diff < PET_MIN_ANGLE or angle_diff > PET_MAX_ANGLE:
        return np.inf

    cx1 = sum(p[0] for p in veh1) / 4
    cy1 = sum(p[1] for p in veh1) / 4
    cx2 = sum(p[0] for p in veh2) / 4
    cy2 = sum(p[1] for p in veh2) / 4

    angle1 = np.radians(param1[2])
    angle2 = np.radians(param2[2])

    def intersection_point(x1, y1, theta1, x2, y2, theta2):
        # 射线参数方程求交点
        A1 = np.sin(theta1)
        B1 = -np.cos(theta1)
        C1 = A1 * x1 + B1 * y1
        A2 = np.sin(theta2)
        B2 = -np.cos(theta2)
        C2 = A2 * x2 + B2 * y2
        det = A1 * B2 - A2 * B1
        if abs(det) < 1e-6:
            return None
        x = (B2 * C1 - B1 * C2) / det
        y = (A1 * C2 - A2 * C1) / det
        t = (x - x1) / np.cos(theta1) if abs(np.cos(theta1)) > 1e-6 else (y - y1) / np.sin(theta1)
        s = (x - x2) / np.cos(theta2) if abs(np.cos(theta2)) > 1e-6 else (y - y2) / np.sin(theta2)
        if t >= 0 and s >= 0:
            return (x, y), t, s
        return None

    result = intersection_point(cx1, cy1, angle1, cx2, cy2, angle2)
    if result is None:
        return np.inf
    point, t1, t2 = result

    v1 = param1[0]
    v2 = param2[0]
    dir1 = (np.cos(angle1), np.sin(angle1))
    dir2 = (np.cos(angle2), np.sin(angle2))
    vec1 = (point[0] - cx1, point[1] - cy1)
    vec2 = (point[0] - cx2, point[1] - cy2)

    # 确保速度方向指向交点
    if np.dot(dir1, vec1) < 0 or np.dot(dir2, vec2) < 0:
        return np.inf

    dist1 = math.hypot(vec1[0], vec1[1])
    dist2 = math.hypot(vec2[0], vec2[1])
    if v1 <= 0 or v2 <= 0:
        return np.inf
    time1 = dist1 / v1
    time2 = dist2 / v2
    return abs(time1 - time2)


# -------------------- 可视化 --------------------
def visualize_event(frame, v1, v2, id1, id2, point1, point2, pet, ttc, upper_marks, lower_marks):
    """绘制小值事件的场景图像"""
    bbox1 = order_rect_points(v1['bbox'])
    bbox2 = order_rect_points(v2['bbox'])
    all_x = [p[0] for p in bbox1] + [p[0] for p in bbox2] + [point1[0], point2[0]]
    all_y = [p[1] for p in bbox1] + [p[1] for p in bbox2] + [point1[1], point2[1]]
    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    margin = 10
    x_min -= margin
    x_max += margin
    y_min -= margin
    y_max += margin

    plt.figure(figsize=(10, 8))
    for y in upper_marks:
        plt.axhline(y=y, color='gray', linestyle='--', linewidth=1, label='Upper lane' if y == upper_marks[0] else "")
    for y in lower_marks:
        plt.axhline(y=y, color='black', linestyle='--', linewidth=1, label='Lower lane' if y == lower_marks[0] else "")

    rect1 = plt.Polygon(bbox1, closed=True, fill=None, edgecolor='red', linewidth=2, label=f"Vehicle {v1['class']} (ID:{id1})")
    plt.gca().add_patch(rect1)
    rect2 = plt.Polygon(bbox2, closed=True, fill=None, edgecolor='blue', linewidth=2, label=f"Vehicle {v2['class']} (ID:{id2})")
    plt.gca().add_patch(rect2)

    plt.scatter([point1[0], point2[0]], [point1[1], point2[1]], color='green', s=50, zorder=5, label='Nearest points')
    plt.plot([point1[0], point2[0]], [point1[1], point2[1]], color='green', linestyle='-', linewidth=2, label='Nearest line')

    plt.title(f"Frame {frame} - PET: {pet:.4f}s, 2D TTC: {ttc:.4f}s")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend(loc='upper right')
    plt.axis('equal')
    plt.grid(True, linestyle=':', alpha=0.6)

    if SAVE_VISUALIZATION:
        os.makedirs(VISUALIZATION_OUTPUT_DIR, exist_ok=True)
        filename = f"frame_{frame}_v{id1}_{id2}.png"
        filepath = os.path.join(VISUALIZATION_OUTPUT_DIR, filename)
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"可视化已保存: {filepath}")
    if SHOW_VISUALIZATION:
        plt.show()
    plt.close()


# -------------------- 主函数 --------------------
def main():
    # 文件路径（请根据实际情况修改）
    meta_file = r"E:\xizihao\Internship\zhiling\work\NBDT\utils\01_recordingMeta.csv"
    tracks_file = r"E:\xizihao\Internship\zhiling\work\NBDT\utils\01_tracks.csv"
    # 加载元数据
    frame_rate, upper_marks, lower_marks = load_metadata(meta_file)
    dt = 1.0 / frame_rate
    print(f"帧率: {frame_rate} fps, 时间步长: {dt:.4f} s")
    print(f"车道标记 - 上边界: {upper_marks}, 下边界: {lower_marks}")

    # 加载轨迹数据
    df_tracks = load_tracks(tracks_file)
    frames = sorted(df_tracks['frameNum'].unique())
    total_frames = len(frames)

    # 数据结构
    frame_vehicles = defaultdict(dict)           # frame -> {carId: info}
    vehicle_trajectories = defaultdict(list)      # carId -> 历史轨迹
    prev_frame_info = defaultdict(dict)           # 用于计算加速度/角速度

    act_calculator = ACT()

    # 结果存储
    ttc_1d_series = defaultdict(list)      # (follow_id, lead_id) -> list of (frame, ttc)
    mttc_1d_series = defaultdict(list)     # (follow_id, lead_id) -> list of (frame, mttc)
    cai_series = defaultdict(list)          # (follow_id, lead_id) -> list of (frame, cai)
    drac_series = defaultdict(list)         # follow_id -> list of (frame, drac, class)
    ttc_2d_results = []                     # (frame, id1, id2, act)
    pet_results = []                         # (frame, id1, id2, pet)

    processed_pairs_1d = 0

    # 逐帧处理
    for frame_idx, frame in enumerate(frames):
        df_frame = df_tracks[df_tracks['frameNum'] == frame]
        vehicles_this_frame = {}

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

            # 加速度和角速度（基于前一帧）
            a = 0.0
            angle_speed = 0.0
            if carId in prev_frame_info:
                prev = prev_frame_info[carId]
                if prev['frame'] == frame - 1:
                    dv = speed - prev['speed']
                    a = dv / dt
                    d_heading = heading - prev['heading']
                    if d_heading > 180:
                        d_heading -= 360
                    elif d_heading < -180:
                        d_heading += 360
                    angle_speed = d_heading / dt

            prev_frame_info[carId] = {'speed': speed, 'heading': heading, 'frame': frame}

            info = {
                'x': x, 'y': y, 'vx': vx, 'vy': vy, 'L': L, 'W': W,
                'laneId': laneId, 'class': obj_class, 'heading': heading,
                'speed': speed, 'a': a, 'angle_speed': angle_speed,
                'bbox': bbox
            }
            vehicles_this_frame[carId] = info
            vehicle_trajectories[carId].append({'frame': frame, **info})

        frame_vehicles[frame] = vehicles_this_frame
        car_ids = list(vehicles_this_frame.keys())
        n = len(car_ids)

        # ========== 1D 指标（同车道） ==========
        lane_groups = defaultdict(list)
        for cid in car_ids:
            info = vehicles_this_frame[cid]
            lane = info['laneId']
            lane_groups[lane].append((cid, info))

        for lane, group in lane_groups.items():
            # 估计车道方向
            lane_dir_deg = estimate_lane_direction([info for (_, info) in group])
            if lane_dir_deg is None:
                continue
            rad = np.radians(lane_dir_deg)
            cosθ, sinθ = np.cos(rad), np.sin(rad)

            # 投影距离、速度、加速度
            proj_list = []
            for cid, info in group:
                proj_dist = info['x'] * cosθ + info['y'] * sinθ
                proj_speed = info['vx'] * cosθ + info['vy'] * sinθ
                heading_rad = np.radians(info['heading'])
                delta = heading_rad - rad
                proj_acc = info['a'] * np.cos(delta)
                proj_list.append((cid, info, proj_dist, proj_speed, proj_acc))

            proj_list.sort(key=lambda x: x[2])  # 按距离从小到大（后车在前）

            for i in range(len(proj_list) - 1):
                follow_id, follow_info, follow_dist, follow_speed, follow_acc = proj_list[i]
                lead_id, lead_info, lead_dist, lead_speed, lead_acc = proj_list[i + 1]

                # 必须同向且后车速度大于前车
                if follow_speed * lead_speed <= 0 or follow_speed <= lead_speed:
                    continue

                gap = lead_dist - follow_dist - (lead_info['L'] / 2 + follow_info['L'] / 2)
                dv = follow_speed - lead_speed
                da = follow_acc - lead_acc

                # TTC
                ttc = ttc_from_gap_and_dv(gap, dv)
                if not np.isinf(ttc):
                    ttc_1d_series[(follow_id, lead_id)].append((frame, ttc))

                # MTTC
                mttc = compute_mttc(gap, dv, da)
                if not np.isinf(mttc):
                    mttc_1d_series[(follow_id, lead_id)].append((frame, mttc))
                    cai = compute_cai(follow_speed, follow_acc, lead_speed, lead_acc, mttc)
                    cai_series[(follow_id, lead_id)].append((frame, cai))

                # DRAC
                drac = drac_from_gap_and_dv(gap, dv)
                if not np.isinf(drac):
                    drac_series[follow_id].append((frame, drac, follow_info['class']))

                processed_pairs_1d += 1

        # ========== 2D 指标（所有车辆对，但只考虑同向） ==========
        for i in range(n):
            for j in range(i + 1, n):
                id1 = car_ids[i]
                id2 = car_ids[j]
                v1 = vehicles_this_frame[id1]
                v2 = vehicles_this_frame[id2]

                # 航向差
                diff = abs(v1['heading'] - v2['heading']) % 360
                if diff > 180:
                    diff = 360 - diff
                if diff > HEADING_DIFF_THRESHOLD:
                    continue

                # 2D TTC
                param1 = (v1['speed'], v1['a'], v1['heading'], v1['angle_speed'])
                param2 = (v2['speed'], v2['a'], v2['heading'], v2['angle_speed'])
                act = act_calculator.run(v1['bbox'], param1, v2['bbox'], param2)
                ttc_2d_results.append((frame, id1, id2, act))

                # PET
                pet = compute_pet(v1['bbox'], param1, v2['bbox'], param2, diff)
                pet_results.append((frame, id1, id2, pet))

                # 小值打印与可视化
                need_print = False
                if not np.isinf(pet) and pet < PET_PRINT_THRESHOLD:
                    need_print = True
                if not np.isinf(act) and act < TTC2D_PRINT_THRESHOLD:
                    need_print = True

                if need_print:
                    point1, point2, distance = calculate_nearest_points(v1['bbox'], v2['bbox'])
                    print("\n===== 小值事件 =====")
                    print(f"帧: {frame}")
                    print(f"车辆1 ID: {id1}, 类型: {v1['class']}, 速度: {v1['speed']:.2f} m/s, "
                          f"航向: {v1['heading']:.2f}°, 加速度: {v1['a']:.2f} m/s², 中心: ({v1['x']:.2f}, {v1['y']:.2f})")
                    print(f"车辆2 ID: {id2}, 类型: {v2['class']}, 速度: {v2['speed']:.2f} m/s, "
                          f"航向: {v2['heading']:.2f}°, 加速度: {v2['a']:.2f} m/s², 中心: ({v2['x']:.2f}, {v2['y']:.2f})")
                    print(f"最近点1: ({point1[0]:.2f}, {point1[1]:.2f}), 最近点2: ({point2[0]:.2f}, {point2[1]:.2f}), 距离: {distance:.2f} m")
                    print(f"PET: {pet:.4f} s")
                    print(f"2D TTC: {act:.4f} s")
                    print("========================")

                    if ENABLE_VISUALIZATION:
                        visualize_event(frame, v1, v2, id1, id2, point1, point2, pet, act,
                                        upper_marks, lower_marks)

    print("逐帧计算完成。")
    print(f"总共处理了 {processed_pairs_1d} 个跟驰对。")

    # ========== 输出统计结果 ==========
    # TIT/TET
    print("\n--- TIT/TET (基于一维TTC序列) ---")
    tet_nonzero = 0
    tit_nonzero = 0
    for (follow_id, lead_id), ttc_list in ttc_1d_series.items():
        if len(ttc_list) < 2:
            continue
        ttc_list.sort(key=lambda x: x[0])
        TET = 0.0
        TIT = 0.0
        for frame, ttc in ttc_list:
            if ttc < TTC_THRESHOLD:
                TET += dt
                TIT += (1.0 / ttc - 1.0 / TTC_THRESHOLD) * dt
        if TET > 0:
            tet_nonzero += 1
        if TIT > 0:
            tit_nonzero += 1
        print(f"车对 ({follow_id}->{lead_id}): TET = {TET:.4f} s, TIT = {TIT:.4f} s")
    print(f"总计: {tet_nonzero} 个车对 TET > 0, {tit_nonzero} 个车对 TIT > 0")

    # CPI
    print("\n--- CPI (每辆后车) ---")
    cpi_per_vehicle = {}
    for follow_id, drac_list in drac_series.items():
        drac_list.sort(key=lambda x: x[0])
        total_time = len(drac_list) * dt
        risky_time = 0.0
        veh_class = drac_list[0][2]
        madr = VEHICLE_CLASS_MADR.get(veh_class, MADR_CAR)
        for _, drac, _ in drac_list:
            if drac >= madr:
                risky_time += dt
        cpi = risky_time / total_time if total_time > 0 else 0
        cpi_per_vehicle[follow_id] = cpi
        print(f"车辆 {follow_id} (class {veh_class}, MADR={madr}): CPI = {cpi:.4f}")
    nonzero_cpi = sum(1 for cpi in cpi_per_vehicle.values() if cpi > 0)
    print(f"总计: {nonzero_cpi} 辆后车 CPI > 0")

    # 2D TTC
    print("\n--- 2D TTC (ACT) 统计 ---")
    valid_ttc2 = [t for _, _, _, t in ttc_2d_results if t < 20]
    if valid_ttc2:
        print(f"有效2D TTC数量: {len(valid_ttc2)}")
        print(f"最小TTC: {min(valid_ttc2):.4f} s, 平均TTC: {np.mean(valid_ttc2):.4f} s")
    else:
        print("无有效2D TTC")

    # PET
    print("\n--- PET 统计 ---")
    valid_pet = [p for _, _, _, p in pet_results if not np.isinf(p)]
    if valid_pet:
        print(f"有效PET数量: {len(valid_pet)}")
        print(f"最小PET: {min(valid_pet):.4f} s, 平均PET: {np.mean(valid_pet):.4f} s")
    else:
        print("无有效PET")

    # MTTC
    print("\n--- MTTC 统计 (基于一维) ---")
    mttc_all = []
    for ttc_list in mttc_1d_series.values():
        for _, m in ttc_list:
            if not np.isinf(m):
                mttc_all.append(m)
    if mttc_all:
        print(f"有效MTTC数量: {len(mttc_all)}")
        print(f"最小MTTC: {min(mttc_all):.4f} s, 平均MTTC: {np.mean(mttc_all):.4f} s")
    else:
        print("无有效MTTC")

    # CAI
    print("\n--- CAI 统计 (基于一维) ---")
    cai_all = []
    for cai_list in cai_series.values():
        for _, c in cai_list:
            if not np.isinf(c):
                cai_all.append(c)
    if cai_all:
        print(f"有效CAI数量: {len(cai_all)}")
        print(f"最小CAI: {min(cai_all):.4f}, 平均CAI: {np.mean(cai_all):.4f}")
        cai_neg = sum(1 for c in cai_all if c < 0)
        print(f"其中负值数量: {cai_neg}")
    else:
        print("无有效CAI")


if __name__ == "__main__":
    main()