import math
import numpy as np
from box_distance import calculate_nearest_points, order_rect_points, line_segment_intersection

# 2D TTC：一阶闭合项与相对加速度剪枝阈值
_FIRST_ORDER_CLOSING_EPS = 1e-6
_FIRST_ORDER_ACC_EPS = 1e-6

# PET：在后车车头线段上等距离采样点数（越大越精细但越慢）
PET_HEAD_SAMPLE_NUM = 5

# 0 正面 / 1 追尾 / 2 刮擦 / 3 斜碰
CONFLICT_TYPE_CODES = {
    0: "head_on",
    1: "rear_end",
    2: "side_swipe",
    3: "angled",
}
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


def _front_edge_from_ordered_bbox(ordered_bbox, heading_deg):
    """有序矩形上，取「车头」边：边中点在航向轴上投影相对质心最大的一条边。"""
    rad = math.radians(heading_deg)
    fx, fy = math.cos(rad), math.sin(rad)
    cx = sum(p[0] for p in ordered_bbox) / 4.0
    cy = sum(p[1] for p in ordered_bbox) / 4.0
    best = None
    max_proj = float("-inf")
    n = len(ordered_bbox)
    for i in range(n):
        ax, ay = ordered_bbox[i]
        bx, by = ordered_bbox[(i + 1) % n]
        mx, my = 0.5 * (ax + bx), 0.5 * (ay + by)
        proj = (mx - cx) * fx + (my - cy) * fy
        if proj > max_proj:
            max_proj = proj
            best = ((ax, ay), (bx, by))
    return best[0], best[1]


def _rear_edge_from_ordered_bbox(ordered_bbox, heading_deg):
    """有序矩形上，取「车尾」边：边中点在航向轴上投影相对质心最小的一条边。"""
    rad = math.radians(heading_deg)
    fx, fy = math.cos(rad), math.sin(rad)
    cx = sum(p[0] for p in ordered_bbox) / 4.0
    cy = sum(p[1] for p in ordered_bbox) / 4.0
    best = None
    min_proj = float("inf")
    n = len(ordered_bbox)
    for i in range(n):
        ax, ay = ordered_bbox[i]
        bx, by = ordered_bbox[(i + 1) % n]
        mx, my = 0.5 * (ax + bx), 0.5 * (ay + by)
        proj = (mx - cx) * fx + (my - cy) * fy
        if proj < min_proj:
            min_proj = proj
            best = ((ax, ay), (bx, by))
    return best[0], best[1]


def _point_in_forward_strip(q, r0, r1, dx, dy):
    """
    半无限条带：车尾线段 r0—r1，沿单位方向 (dx,dy) 向前延伸。
    q = r0 + a*(r1-r0) + t*(dx,dy), a∈[0,1], t≥0
    """
    wx, wy = r1[0] - r0[0], r1[1] - r0[1]
    det = wx * dy - wy * dx
    if abs(det) < 1e-12:
        return False
    vx, vy = q[0] - r0[0], q[1] - r0[1]
    a = (vx * dy - vy * dx) / det
    t = (wx * vy - wy * vx) / det
    return -1e-9 <= a <= 1.0 + 1e-9 and t >= -1e-9


def _ray_intersect_segment(ox, oy, dx, dy, ax, ay, bx, by):
    """射线 O + t*D, t≥0 与线段 AB 是否相交。"""
    wx, wy = bx - ax, by - ay
    cross_dw = dx * wy - dy * wx
    if abs(cross_dw) < 1e-12:
        return False
    t = ((ax - ox) * wy - (ay - oy) * wx) / cross_dw
    u = ((ax - ox) * dy - (ay - oy) * dx) / cross_dw
    return t >= -1e-9 and 0.0 <= u <= 1.0


def _two_rays_intersect_forward(o1x, o1y, d1x, d1y, o2x, o2y, d2x, d2y):
    """两射线 O1+t*D1、O2+s*D2，t,s≥0 是否相交。"""
    cross_d = d1x * d2y - d1y * d2x
    if abs(cross_d) < 1e-12:
        return False
    dx, dy = o2x - o1x, o2y - o1y
    t = (dx * d2y - dy * d2x) / cross_d
    s = (dx * d1y - dy * d1x) / cross_d
    return t >= -1e-9 and s >= -1e-9


def _rear_forward_strips_intersect(veh1, veh2, heading1_deg, heading2_deg):
    """
    两车各自：车尾线段 + 沿航向的半无限条带（2D 平面上的「行驶区域」）。
    若相交则返回 True；用于 2D_TTC 先验（同向跟驰时两平行行驶区域常仍相交）。
    """
    o1 = order_rect_points(list(veh1))
    o2 = order_rect_points(list(veh2))
    r0a, r1a = _rear_edge_from_ordered_bbox(o1, heading1_deg)
    r0b, r1b = _rear_edge_from_ordered_bbox(o2, heading2_deg)
    t1 = math.radians(heading1_deg)
    t2 = math.radians(heading2_deg)
    dax, day = math.cos(t1), math.sin(t1)
    dbx, dby = math.cos(t2), math.sin(t2)

    if _point_in_forward_strip(r0a, r0b, r1b, dbx, dby) or _point_in_forward_strip(
        r1a, r0b, r1b, dbx, dby
    ):
        return True
    if _point_in_forward_strip(r0b, r0a, r1a, dax, day) or _point_in_forward_strip(
        r1b, r0a, r1a, dax, day
    ):
        return True

    for ox, oy in (r0a, r1a):
        if _ray_intersect_segment(ox, oy, dax, day, r0b[0], r0b[1], r1b[0], r1b[1]):
            return True
        if _two_rays_intersect_forward(ox, oy, dax, day, r0b[0], r0b[1], dbx, dby):
            return True
        if _two_rays_intersect_forward(ox, oy, dax, day, r1b[0], r1b[1], dbx, dby):
            return True

    for ox, oy in (r0b, r1b):
        if _ray_intersect_segment(ox, oy, dbx, dby, r0a[0], r0a[1], r1a[0], r1a[1]):
            return True
        if _two_rays_intersect_forward(ox, oy, dbx, dby, r0a[0], r0a[1], dax, day):
            return True
        if _two_rays_intersect_forward(ox, oy, dbx, dby, r1a[0], r1a[1], dax, day):
            return True

    if line_segment_intersection(r0a, r1a, r0b, r1b) is not None:
        return True
    return False


# 基于Bounding Box的2D TTC计算
def compute_2d_ttc_bbox(veh1, veh2, param1, param2, scale=1.0):
    """
    计算两车（视为有朝向的矩形）在当前运动状态下的碰撞时间。
    基于最近点连线的投影，并考虑角速度的影响。

    不需要计算时返回 inf：① 车尾条带不相交；② 沿最近边一阶闭合项 v_rel+w*d 非正且 a_rel 非正。
    否则解 0.5*a_rel*t^2 + (v_rel+w*d)*t - d = 0（d 为缩放后间距）。
    
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

    if not _rear_forward_strips_intersect(veh1, veh2, param1[2], param2[2]):
        return np.inf

    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    alpha = math.atan2(dy, dx)

    v1, a1 = _project_to_line(param1[0], param1[1], np.radians(param1[2]), alpha)
    v2, a2 = _project_to_line(param2[0], param2[1], np.radians(param2[2]), alpha)
    v_rel = v1 - v2
    a_rel = a1 - a2
    w = np.radians(param1[3] + param2[3])
    d_scaled = distance * scale
    closing = v_rel + w * d_scaled
    if closing <= _FIRST_ORDER_CLOSING_EPS and a_rel <= _FIRST_ORDER_ACC_EPS:
        return np.inf

    # 求解二次方程： 0.5 * a_rel * t^2 + (v_rel + w * d_scaled) * t - d_scaled = 0
    if a_rel == 0:
        # 线性情况
        if v_rel + w * d_scaled <= 0:
            return np.inf
        t = d_scaled / (v_rel + w * d_scaled)
        return t if t > 0 else np.inf

    disc = (v_rel + w * d_scaled) ** 2 + 2 * a_rel * d_scaled
    if disc < 0:
        return np.inf
    sqrt_disc = math.sqrt(disc)
    t1 = (- (v_rel + w * d_scaled) - sqrt_disc) / a_rel
    t2 = (- (v_rel + w * d_scaled) + sqrt_disc) / a_rel
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


def _bbox_head_tail_edges(veh, theta_rad):
    #veh: 四顶点；theta_rad: 航向弧度。返回 (前边线段, 后边线段)，每段为 ((x1,y1),(x2,y2))
    veh_s = sorted(veh)
    if 0.5 < (theta_rad / np.pi) < 1.5:
        return veh_s[:2], veh_s[2:]
    return veh_s[2:], veh_s[:2]


def _point_on_segment(line, point):
    # 点是否落在线段上。
    (x1, y1), (x2, y2) = line
    px, py = point
    l1 = np.array([px - x1, py - y1], dtype=float)
    l2 = np.array([x2 - x1, y2 - y1], dtype=float)
    cross = np.cross(l1, l2)
    if cross < 1e-3:
        if min(x1, x2) <= px <= max(x1, x2) and min(y1, y2) <= py <= max(y1, y2):
            return True
        norm_l2 = np.linalg.norm(l2)
        if norm_l2 > 1e-9 and abs(cross / norm_l2) < 2:
            return True
        return False
    return False


def classify_conflict_type(point1, point2, veh1, veh2, theta1_rad, theta2_rad):
    """
    根据两车间最近点落在各自矩形前边/后边及航向关系，划分冲突类型。
    无法归类时返回 None。
    """
    head_line1, tail_line1 = _bbox_head_tail_edges(veh1, theta1_rad)
    head_line2, tail_line2 = _bbox_head_tail_edges(veh2, theta2_rad)

    if _point_on_segment(head_line1, point1):
        if _point_on_segment(head_line2, point2):
            return 0
        if _point_on_segment(tail_line2, point2):
            return 1
        if (theta1_rad - theta2_rad) < (0.5 * np.pi):
            return 2
        return 3
    if _point_on_segment(head_line2, point2):
        if _point_on_segment(tail_line1, point1):
            return 1
        if (theta1_rad - theta2_rad) < (0.5 * np.pi):
            return 2
        return 3
    return None


def conflict_type_label(code):
    if code is None:
        return None
    return CONFLICT_TYPE_CODES.get(code)


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


def _ray_segment_intersection_distance(origin, direction, a, b):
    """
    射线 origin + t*direction (t>=0) 与线段 a-b 的交点参数 t。
    有交点则返回 t（单位为米，direction 需为单位向量），否则返回 None。
    """
    ox, oy = origin
    dx, dy = direction
    ax, ay = a
    bx, by = b
    wx, wy = bx - ax, by - ay
    det = dx * wy - dy * wx
    if abs(det) < 1e-12:
        return None
    t = ((ax - ox) * wy - (ay - oy) * wx) / det
    u = ((ax - ox) * dy - (ay - oy) * dx) / det
    if t >= 0 and 0 <= u <= 1:
        return t
    return None


def _head_intrusion_time(follow_bbox, follow_param, lead_bbox, lead_param):
    """
    计算「后车车头线段」侵入「前车 bounding box」的时间（取最早接触）。
    返回 inf 表示该方向下不会侵入。
    """
    follow_ordered = order_rect_points(list(follow_bbox))
    lead_ordered = order_rect_points(list(lead_bbox))
    fh0, fh1 = _front_edge_from_ordered_bbox(follow_ordered, follow_param[2])

    theta_f = math.radians(follow_param[2])
    dir_f = (math.cos(theta_f), math.sin(theta_f))

    v_follow = follow_param[0]
    theta_l = math.radians(lead_param[2])
    v_lead_proj = lead_param[0] * math.cos(theta_l - theta_f)
    rel_speed = v_follow - v_lead_proj
    if rel_speed <= 1e-6:
        return np.inf

    # 计算射线 origin + t*dir 与 lead_bbox 四条边的最小正交点距离（t 即沿 dir 的距离）。
    def ray_to_bbox_distance(origin):
        best = None
        for i in range(4):
            a = lead_ordered[i]
            b = lead_ordered[(i + 1) % 4]
            t = _ray_segment_intersection_distance(origin, dir_f, a, b)
            if t is None:
                continue
            if best is None or t < best:
                best = t
        return best

    # 用离散采样：在 follow 车头线段上等距离取若干点，取最早侵入时间。
    if PET_HEAD_SAMPLE_NUM < 2:
        sample_num = 2
    else:
        sample_num = PET_HEAD_SAMPLE_NUM
    rx, ry = (fh1[0] - fh0[0], fh1[1] - fh0[1])
    unique = []
    for i in range(sample_num):
        a = i / (sample_num - 1)
        q = (fh0[0] + a * rx, fh0[1] + a * ry)
        unique.append(q)

    best_time = np.inf
    for q in unique:
        dist = ray_to_bbox_distance(q)
        if dist is None:
            continue
        # rel_speed 是沿车头方向的相对逼近速度，因此侵入时间 = dist / rel_speed
        t = dist / rel_speed
        if t > 1e-6 and t < best_time:
            best_time = t

    return best_time


def compute_pet(veh1_bbox, param1, veh2_bbox, param2, angle_diff=None):
    """
    计算 PET：定义为「后车车头侵入前车车尾边」的时间。
    对 1->2 与 2->1 两个方向分别计算，取较小正值；无侵入则返回 inf。
    """
    if not _rear_forward_strips_intersect(veh1_bbox, veh2_bbox, param1[2], param2[2]):
        return np.inf
    t12 = _head_intrusion_time(veh1_bbox, param1, veh2_bbox, param2)
    t21 = _head_intrusion_time(veh2_bbox, param2, veh1_bbox, param1)
    return min(t12, t21)
    

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
            '2D_TTC': np.inf,
            'conflict_type': None,
        }

        param1 = (veh1['speed'], veh1['a'], veh1['heading'], veh1['angle_speed'])
        param2 = (veh2['speed'], veh2['a'], veh2['heading'], veh2['angle_speed'])
        result['2D_TTC'] = compute_2d_ttc_bbox(veh1['bbox'], veh2['bbox'], param1, param2)

        p1, p2, _dist_nn = calculate_nearest_points(veh1['bbox'], veh2['bbox'])
        th1 = np.radians(veh1['heading'])
        th2 = np.radians(veh2['heading'])
        code = classify_conflict_type(p1, p2, veh1['bbox'], veh2['bbox'], th1, th2)
        result['conflict_type'] = conflict_type_label(code)

        # 计算PET
        result['PET'] = compute_pet(veh1['bbox'], param1, veh2['bbox'], param2)

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
                result['TTC'] = ttc_from_gap_and_dv(gap, dv)
                result['MTTC'] = compute_mttc(gap, dv, da)
                if not np.isinf(result['MTTC']):
                    result['CAI'] = compute_cai(follow_speed, follow_acc, lead_speed, lead_acc, result['MTTC'])
                result['DRAC'] = drac_from_gap_and_dv(gap, dv)

        return result

    @staticmethod
    def _angle_diff(a1, a2):
        diff = abs(a1 - a2) % 360
        if diff > 180:
            diff = 360 - diff
        return diff