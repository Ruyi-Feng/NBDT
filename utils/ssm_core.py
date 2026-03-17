import math
import numpy as np
from box_distance import calculate_nearest_points

# ========================= Configuration Parameters =========================
TTC_THRESHOLD = 5.0
MTTC_THRESHOLD = 5.0
DRAC_THRESHOLD = 3.4
CAI_THRESHOLD = 10.0
PET_THRESHOLD = 0.2
TTC2D_THRESHOLD = 0.3

THRESHOLD_MIN_ANGLE = 5      # 用于PET的角度下限
THRESHOLD_MAX_ANGLE = 150    # 用于PET的角度上限
# ============================================================================

def ttc_from_gap_and_dv(gap, dv):
    if gap <= 0 or dv <= 0:
        return np.inf
    return gap / dv

def drac_from_gap_and_dv(gap, dv):
    if gap <= 0 or dv <= 0:
        return np.inf
    return dv**2 / (2.0 * gap)

def compute_mttc(gap, dv, da):
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
    if np.isinf(mttc) or mttc <= 0:
        return np.inf
    term_f = v_f + a_f * mttc
    term_l = v_l + a_l * mttc
    return 0.5 * (term_f**2 - term_l**2) / mttc

def compute_cpi_from_drac_history(drac_history, vehicle_class_madr, dt):
    """
    根据某辆车的DRAC历史记录计算CPI。
    参数：
        drac_history: list of (frame, drac, class)
        vehicle_class_madr: dict, 车辆类型到MADR的映射
        dt: 时间步长
    返回：
        CPI值
    """
    if not drac_history:
        return 0.0
    drac_history.sort(key=lambda x: x[0])
    total_time = len(drac_history) * dt
    veh_class = drac_history[0][2]
    madr = vehicle_class_madr.get(veh_class, vehicle_class_madr.get(-1, 3.0))
    risky_time = sum(dt for _, drac, _ in drac_history if drac >= madr)
    cpi = risky_time / total_time if total_time > 0 else 0
    return cpi

def compute_tet_tit_from_history(ttc_history, dt, threshold):
    """
    根据一段连续帧的 TTC 历史计算 TET 和 TIT。
    ttc_history: list of float，按帧顺序排列的有效 TTC 值（已过滤 inf）
    dt: 时间步长（秒）
    threshold: TTC 阈值
    返回 (TET, TIT)
    """
    TET = 0.0
    TIT = 0.0
    for ttc in ttc_history:
        if ttc < threshold:
            TET += dt
            TIT += (1.0 / ttc - 1.0 / threshold) * dt
    return TET, TIT

# 基于Bounding Box的2D TTC计算
def compute_2d_ttc_bbox(veh1, veh2, param1, param2, scale=1.0):
    """
    计算两车（视为有朝向的矩形）在当前运动状态下的碰撞时间。
    基于最近点连线的投影，并考虑角速度的影响。
    
    参数：
        veh1, veh2: 四个顶点坐标的列表（无序）
        param1, param2: (speed, acceleration, heading_deg, angular_speed_deg)
        scale: 距离缩放因子
    返回：
        碰撞时间
    """
    point1, point2, distance = calculate_nearest_points(veh1, veh2)
    if distance <= 0:
        return 0.0 

    # 计算最近点连线的方向角（从 point1 指向 point2）
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    alpha = math.atan2(dy, dx)

    # 将两车的速度、加速度投影到连线方向
    v1, a1 = _project_to_line(param1[0], param1[1], np.radians(param1[2]), alpha)
    v2, a2 = _project_to_line(param2[0], param2[1], np.radians(param2[2]), alpha)

    v_rel = v1 - v2
    a_rel = a1 - a2

    # 角速度影响：两车角速度之和（弧度/秒），用于修正相对速度
    w = np.radians(param1[3] + param2[3])

    # 求解二次方程： 0.5 * a_rel * t^2 + (v_rel + w * distance) * t - distance = 0
    distance = distance * scale
    if a_rel == 0:
        # 线性情况
        if v_rel + w * distance <= 0:
            return np.inf
        t = distance / (v_rel + w * distance)
        return t if t > 0 else np.inf

    disc = (v_rel + w * distance) ** 2 + 2 * a_rel * distance
    if disc < 0:
        return np.inf
    sqrt_disc = math.sqrt(disc)
    t1 = (- (v_rel + w * distance) - sqrt_disc) / a_rel
    t2 = (- (v_rel + w * distance) + sqrt_disc) / a_rel
    candidates = [t for t in (t1, t2) if t > 1e-6]
    if not candidates:
        return np.inf
    return min(candidates)

def _project_to_line(v, a, theta, alpha):
    #将速度和加速度投影到给定方向 alpha 上
    delta = theta - alpha
    v_proj = v * np.cos(delta)
    a_proj = a * np.cos(delta)
    return v_proj, a_proj

def ray_intersection(p1, theta1, p2, theta2):
    """
    计算两条射线的交点。
    参数：
        p1, p2: 起点 (x, y)
        theta1, theta2: 方向角（弧度）
    返回：
        如果存在交点且都在射线上，返回 (x, y, t1, t2)，其中 t1, t2 分别为从起点到交点的距离
        否则返回 None
    """
    x1, y1 = p1
    x2, y2 = p2
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
    t1 = (x - x1) / np.cos(theta1) if abs(np.cos(theta1)) > 1e-6 else (y - y1) / np.sin(theta1)
    t2 = (x - x2) / np.cos(theta2) if abs(np.cos(theta2)) > 1e-6 else (y - y2) / np.sin(theta2)
    if t1 >= 0 and t2 >= 0:
        return (x, y), t1, t2
    return None

def compute_pet(v1xcenter, v1ycenter, param1, v2xcenter, v2ycenter, param2, angle_diff):
    #计算后侵入时间
    if angle_diff < THRESHOLD_MIN_ANGLE or angle_diff > THRESHOLD_MAX_ANGLE:
        return np.inf

    angle1 = np.radians(param1[2])
    angle2 = np.radians(param2[2])

    result = ray_intersection((v1xcenter, v1ycenter), angle1, (v2xcenter, v2ycenter), angle2)
    if result is None:
        return np.inf
    point, t1, t2 = result

    v1 = param1[0]
    v2 = param2[0]
    dir1 = (np.cos(angle1), np.sin(angle1))
    dir2 = (np.cos(angle2), np.sin(angle2))
    vec1 = (point[0] - v1xcenter, point[1] - v1ycenter)
    vec2 = (point[0] - v2xcenter, point[1] - v2ycenter)

    if np.dot(dir1, vec1) < 0 or np.dot(dir2, vec2) < 0:
        return np.inf

    dist1 = math.hypot(vec1[0], vec1[1])
    dist2 = math.hypot(vec2[0], vec2[1])
    if v1 <= 0 or v2 <= 0:
        return np.inf
    time1 = dist1 / v1
    time2 = dist2 / v2
    return abs(time1 - time2)


class SSMCalculator:
    def compute_for_pair(self, veh1, veh2, same_lane=False,
                         proj_dist1=None, proj_dist2=None,
                         proj_speed1=None, proj_speed2=None,
                         proj_acc1=None, proj_acc2=None,
                         lane_dir_rad=None):
        """
        计算两车之间的所有安全替代指标。
        参数：
            veh1, veh2: 车辆信息字典（必须包含 'x','y','vx','vy','speed','heading','a','angle_speed','L','W','bbox'）
            same_lane: 是否同车道
            proj_dist1, proj_dist2: 沿车道方向的投影距离（仅同车道时需要）
            proj_speed1, proj_speed2: 沿车道方向的投影速度
            proj_acc1, proj_acc2: 沿车道方向的投影加速度
            lane_dir_rad: 车道方向（弧度）
        返回：
            字典，包含 'TTC','MTTC','DRAC','CAI','PET','2D_TTC'
        """
        result = {
            'TTC': np.inf,
            'MTTC': np.inf,
            'DRAC': np.inf,
            'CAI': np.inf,
            'PET': np.inf,
            '2D_TTC': np.inf
        }

        param1 = (veh1['speed'], veh1['a'], veh1['heading'], veh1['angle_speed'])
        param2 = (veh2['speed'], veh2['a'], veh2['heading'], veh2['angle_speed'])
        result['2D_TTC'] = compute_2d_ttc_bbox(veh1['bbox'], veh2['bbox'], param1, param2)

        # 计算PET
        diff = self._angle_diff(veh1['heading'], veh2['heading'])
        if THRESHOLD_MIN_ANGLE <= diff <= THRESHOLD_MAX_ANGLE:
            param1 = (veh1['speed'], veh1['a'], veh1['heading'], veh1['angle_speed'])
            param2 = (veh2['speed'], veh2['a'], veh2['heading'], veh2['angle_speed'])
            pet = compute_pet(veh1['x'], veh1['y'], param1, veh2['x'], veh2['y'], param2, diff)
            result['PET'] = pet

        # 1D 指标
        if same_lane and proj_dist1 is not None and proj_dist2 is not None:
            # 根据投影距离确定前后车
            if proj_dist1 < proj_dist2:
                follow = veh1
                lead = veh2
                follow_dist = proj_dist1
                lead_dist = proj_dist2
                follow_speed = proj_speed1
                lead_speed = proj_speed2
                follow_acc = proj_acc1 if proj_acc1 is not None else veh1['a'] * np.cos(np.radians(veh1['heading']) - lane_dir_rad)
                lead_acc = proj_acc2 if proj_acc2 is not None else veh2['a'] * np.cos(np.radians(veh2['heading']) - lane_dir_rad)
            else:
                follow = veh2
                lead = veh1
                follow_dist = proj_dist2
                lead_dist = proj_dist1
                follow_speed = proj_speed2
                lead_speed = proj_speed1
                follow_acc = proj_acc2 if proj_acc2 is not None else veh2['a'] * np.cos(np.radians(veh2['heading']) - lane_dir_rad)
                lead_acc = proj_acc1 if proj_acc1 is not None else veh1['a'] * np.cos(np.radians(veh1['heading']) - lane_dir_rad)

            gap = lead_dist - follow_dist - (follow['L']/2 + lead['L']/2)
            dv = follow_speed - lead_speed
            da = follow_acc - lead_acc

            if dv > 0 and gap > 0:
                result['TTC'] = gap / dv
                result['MTTC'] = compute_mttc(gap, dv, da)
                if not np.isinf(result['MTTC']):
                    result['CAI'] = compute_cai(follow_speed, follow_acc, lead_speed, lead_acc, result['MTTC'])
                result['DRAC'] = dv**2 / (2.0 * gap)

        return result

    @staticmethod
    def _angle_diff(a1, a2):
        diff = abs(a1 - a2) % 360
        if diff > 180:
            diff = 360 - diff
        return diff