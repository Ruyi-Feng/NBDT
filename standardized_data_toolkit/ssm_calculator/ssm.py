from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

Point = Tuple[float, float]
BBox = Sequence[Point]

_FIRST_ORDER_CLOSING_EPS = 1e-6
_FIRST_ORDER_ACC_EPS = 1e-6
PET_HEAD_SAMPLE_NUM = 5

CONFLICT_TYPE_CODES = {
    0: "head_on",
    1: "rear_end",
    2: "side_swipe",
    3: "angled",
}


@dataclass
class VehicleState:
    x: float
    y: float
    vx: float
    vy: float
    speed: float
    heading: float
    acceleration: float
    angle_speed: float
    length: float
    width: float
    lane_id: int
    obj_class: int
    bbox: List[Point]
    proj_dist: Optional[float] = None
    proj_speed: Optional[float] = None
    proj_acc: Optional[float] = None


class GeometryHelper:
    @staticmethod
    def order_rect_points(points: Sequence[Point]) -> List[Point]:
        centroid_x = sum(p[0] for p in points) / 4.0
        centroid_y = sum(p[1] for p in points) / 4.0

        def sort_key(point: Point) -> float:
            return math.atan2(point[1] - centroid_y, point[0] - centroid_x)

        sorted_points = sorted(points, key=sort_key)
        a, b, c, d = sorted_points
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if cross > 0:
            return [a, d, c, b]
        return sorted_points

    @staticmethod
    def line_segment_intersection(a1: Point, a2: Point, b1: Point, b2: Point) -> Optional[Point]:
        dx1, dy1 = a2[0] - a1[0], a2[1] - a1[1]
        dx2, dy2 = b2[0] - b1[0], b2[1] - b1[1]
        denominator = dx1 * dy2 - dy1 * dx2
        if denominator == 0:
            return None
        dx = b1[0] - a1[0]
        dy = b1[1] - a1[1]
        s = (dx * dy2 - dy * dx2) / denominator
        t = (dx * dy1 - dy * dx1) / denominator
        if 0 <= s <= 1 and 0 <= t <= 1:
            return (a1[0] + s * dx1, a1[1] + s * dy1)
        return None

    @staticmethod
    def _closest_point_on_segment(p: Point, a: Point, b: Point) -> Point:
        ax, ay = a
        bx, by = b
        px, py = p
        dx = bx - ax
        dy = by - ay
        if dx == 0 and dy == 0:
            return a
        t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        return (ax + t * dx, ay + t * dy)

    @staticmethod
    def _distance(p1: Point, p2: Point) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    @staticmethod
    def _segment_segment_closest_points(
        a1: Point, a2: Point, b1: Point, b2: Point
    ) -> Tuple[Point, Point, float]:
        intersect = GeometryHelper.line_segment_intersection(a1, a2, b1, b2)
        if intersect:
            return (intersect, intersect, 0.0)

        candidates = []
        for p in [a1, a2]:
            closest = GeometryHelper._closest_point_on_segment(p, b1, b2)
            candidates.append((p, closest, GeometryHelper._distance(p, closest)))
        for p in [b1, b2]:
            closest = GeometryHelper._closest_point_on_segment(p, a1, a2)
            candidates.append((closest, p, GeometryHelper._distance(closest, p)))

        a_dir = (a2[0] - a1[0], a2[1] - a1[1])
        b_dir = (b2[0] - b1[0], b2[1] - b1[1])
        A = a_dir[0] ** 2 + a_dir[1] ** 2
        B = a_dir[0] * b_dir[0] + a_dir[1] * b_dir[1]
        C = b_dir[0] ** 2 + b_dir[1] ** 2
        D = a_dir[0] * (a1[0] - b1[0]) + a_dir[1] * (a1[1] - b1[1])
        E = b_dir[0] * (a1[0] - b1[0]) + b_dir[1] * (a1[1] - b1[1])
        denominator = A * C - B * B
        if denominator != 0:
            t = (B * E - C * D) / denominator
            s = (A * E - B * D) / denominator
            if 0 <= t <= 1 and 0 <= s <= 1:
                pa = (a1[0] + t * a_dir[0], a1[1] + t * a_dir[1])
                pb = (b1[0] + s * b_dir[0], b1[1] + s * b_dir[1])
                candidates.append((pa, pb, GeometryHelper._distance(pa, pb)))

        candidates.sort(key=lambda x: x[2])
        return candidates[0] if candidates else ((0.0, 0.0), (0.0, 0.0), float("inf"))

    @staticmethod
    def _is_inside_rect(point: Point, rect: Sequence[Point]) -> bool:
        for i in range(4):
            a, b = rect[i], rect[(i + 1) % 4]
            edge_dx = b[0] - a[0]
            edge_dy = b[1] - a[1]
            normal = (edge_dy, -edge_dx)
            ap_x = point[0] - a[0]
            ap_y = point[1] - a[1]
            if normal[0] * ap_x + normal[1] * ap_y < 0:
                return False
        return True

    @staticmethod
    def _rectangles_intersect(rect1: Sequence[Point], rect2: Sequence[Point]) -> bool:
        axes = []
        for rect in [rect1, rect2]:
            for i in range(4):
                a, b = rect[i], rect[(i + 1) % 4]
                edge = (b[0] - a[0], b[1] - a[1])
                normal = (edge[1], -edge[0])
                length = math.hypot(*normal)
                if length == 0:
                    continue
                axes.append((normal[0] / length, normal[1] / length))

        for axis in axes:
            proj1 = [p[0] * axis[0] + p[1] * axis[1] for p in rect1]
            min1, max1 = min(proj1), max(proj1)
            proj2 = [p[0] * axis[0] + p[1] * axis[1] for p in rect2]
            min2, max2 = min(proj2), max(proj2)
            if max1 < min2 or max2 < min1:
                return False
        return True

    @staticmethod
    def calculate_nearest_points(veh1: Sequence[Point], veh2: Sequence[Point]) -> Tuple[Point, Point, float]:
        rect1 = GeometryHelper.order_rect_points(veh1)
        rect2 = GeometryHelper.order_rect_points(veh2)

        if GeometryHelper._rectangles_intersect(rect1, rect2):
            for p in rect1:
                if GeometryHelper._is_inside_rect(p, rect2):
                    return (p, p, 0.0)
            for p in rect2:
                if GeometryHelper._is_inside_rect(p, rect1):
                    return (p, p, 0.0)
            for i in range(4):
                a1, a2 = rect1[i], rect1[(i + 1) % 4]
                for j in range(4):
                    b1, b2 = rect2[j], rect2[(j + 1) % 4]
                    intersect = GeometryHelper.line_segment_intersection(a1, a2, b1, b2)
                    if intersect:
                        return (intersect, intersect, 0.0)
            center1 = (sum(p[0] for p in rect1) / 4.0, sum(p[1] for p in rect1) / 4.0)
            center2 = (sum(p[0] for p in rect2) / 4.0, sum(p[1] for p in rect2) / 4.0)
            return (center1, center2, 0.0)

        min_dist = float("inf")
        closest_pair: Optional[Tuple[Point, Point]] = None
        for p in rect1:
            for j in range(4):
                b1, b2 = rect2[j], rect2[(j + 1) % 4]
                closest = GeometryHelper._closest_point_on_segment(p, b1, b2)
                d = GeometryHelper._distance(p, closest)
                if d < min_dist:
                    min_dist = d
                    closest_pair = (p, closest)
        for p in rect2:
            for i in range(4):
                a1, a2 = rect1[i], rect1[(i + 1) % 4]
                closest = GeometryHelper._closest_point_on_segment(p, a1, a2)
                d = GeometryHelper._distance(closest, p)
                if d < min_dist:
                    min_dist = d
                    closest_pair = (closest, p)
        for i in range(4):
            a1, a2 = rect1[i], rect1[(i + 1) % 4]
            for j in range(4):
                b1, b2 = rect2[j], rect2[(j + 1) % 4]
                pa, pb, d = GeometryHelper._segment_segment_closest_points(a1, a2, b1, b2)
                if d < min_dist:
                    min_dist = d
                    closest_pair = (pa, pb)
        if closest_pair is None:
            return ((0.0, 0.0), (0.0, 0.0), float("inf"))
        return (*closest_pair, min_dist)


class InstantSSMCalculator:
    def compute_for_pair(
        self,
        veh1: VehicleState,
        veh2: VehicleState,
        same_lane: bool = False,
        lane_dir_rad: Optional[float] = None,
    ) -> Dict[str, float]:
        result: Dict[str, float] = {
            "TTC": np.inf,
            "MTTC": np.inf,
            "DRAC": np.inf,
            "CAI": np.inf,
            "PET": np.inf,
            "2D_TTC": np.inf,
            "conflict_type": None,  # type: ignore[assignment]
        }

        param1 = (veh1.speed, veh1.acceleration, veh1.heading, veh1.angle_speed)
        param2 = (veh2.speed, veh2.acceleration, veh2.heading, veh2.angle_speed)
        result["2D_TTC"] = self._compute_2d_ttc_bbox(veh1.bbox, veh2.bbox, param1, param2)
        result["PET"] = self._compute_pet(veh1.bbox, param1, veh2.bbox, param2)

        p1, p2, _ = GeometryHelper.calculate_nearest_points(veh1.bbox, veh2.bbox)
        th1 = np.radians(veh1.heading)
        th2 = np.radians(veh2.heading)
        code = self._classify_conflict_type(p1, p2, veh1.bbox, veh2.bbox, th1, th2)
        result["conflict_type"] = CONFLICT_TYPE_CODES.get(code) if code is not None else None  # type: ignore[assignment]

        if same_lane and lane_dir_rad is not None and veh1.proj_dist is not None and veh2.proj_dist is not None:
            if veh1.proj_dist < veh2.proj_dist:
                follow = veh1
                lead = veh2
            else:
                follow = veh2
                lead = veh1
            gap = lead.proj_dist - follow.proj_dist - (follow.length / 2.0 + lead.length / 2.0)
            follow_speed = self._safe_proj_speed(follow, lane_dir_rad)
            lead_speed = self._safe_proj_speed(lead, lane_dir_rad)
            follow_acc = self._safe_proj_acc(follow, lane_dir_rad)
            lead_acc = self._safe_proj_acc(lead, lane_dir_rad)
            dv = follow_speed - lead_speed
            da = follow_acc - lead_acc
            if dv > 0 and gap > 0:
                result["TTC"] = self._ttc_from_gap_and_dv(gap, dv)
                result["MTTC"] = self._compute_mttc(gap, dv, da)
                result["DRAC"] = self._drac_from_gap_and_dv(gap, dv)
                if not np.isinf(result["MTTC"]):
                    result["CAI"] = self._compute_cai(follow_speed, follow_acc, lead_speed, lead_acc, result["MTTC"])

        return result

    @staticmethod
    def _safe_proj_speed(veh: VehicleState, lane_dir_rad: float) -> float:
        if veh.proj_speed is not None and not np.isnan(veh.proj_speed):
            return float(veh.proj_speed)
        return veh.speed * float(np.cos(np.radians(veh.heading) - lane_dir_rad))

    @staticmethod
    def _safe_proj_acc(veh: VehicleState, lane_dir_rad: float) -> float:
        if veh.proj_acc is not None and not np.isnan(veh.proj_acc):
            return float(veh.proj_acc)
        return veh.acceleration * float(np.cos(np.radians(veh.heading) - lane_dir_rad))

    @staticmethod
    def _ttc_from_gap_and_dv(gap: float, dv: float) -> float:
        if gap <= 0 or dv <= 0:
            return np.inf
        return gap / dv

    @staticmethod
    def _drac_from_gap_and_dv(gap: float, dv: float) -> float:
        if gap <= 0 or dv <= 0:
            return np.inf
        return dv**2 / (2.0 * gap)

    @staticmethod
    def _compute_mttc(gap: float, dv: float, da: float) -> float:
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

    @staticmethod
    def _compute_cai(v_f: float, a_f: float, v_l: float, a_l: float, mttc: float) -> float:
        if np.isinf(mttc) or mttc <= 0:
            return np.inf
        term_f = v_f + a_f * mttc
        term_l = v_l + a_l * mttc
        return 0.5 * (term_f**2 - term_l**2) / mttc

    @staticmethod
    def _project_to_line(v: float, a: float, theta: float, alpha: float) -> Tuple[float, float]:
        delta = theta - alpha
        return v * np.cos(delta), a * np.cos(delta)

    @staticmethod
    def _front_edge_from_ordered_bbox(ordered_bbox: Sequence[Point], heading_deg: float) -> Tuple[Point, Point]:
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
        if best is None:
            return ordered_bbox[0], ordered_bbox[1]
        return best[0], best[1]

    @staticmethod
    def _rear_edge_from_ordered_bbox(ordered_bbox: Sequence[Point], heading_deg: float) -> Tuple[Point, Point]:
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
        if best is None:
            return ordered_bbox[0], ordered_bbox[1]
        return best[0], best[1]

    @staticmethod
    def _point_in_forward_strip(q: Point, r0: Point, r1: Point, dx: float, dy: float) -> bool:
        wx, wy = r1[0] - r0[0], r1[1] - r0[1]
        det = wx * dy - wy * dx
        if abs(det) < 1e-12:
            return False
        vx, vy = q[0] - r0[0], q[1] - r0[1]
        a = (vx * dy - vy * dx) / det
        t = (wx * vy - wy * vx) / det
        return -1e-9 <= a <= 1.0 + 1e-9 and t >= -1e-9

    @staticmethod
    def _ray_intersect_segment(
        ox: float, oy: float, dx: float, dy: float, ax: float, ay: float, bx: float, by: float
    ) -> bool:
        wx, wy = bx - ax, by - ay
        cross_dw = dx * wy - dy * wx
        if abs(cross_dw) < 1e-12:
            return False
        t = ((ax - ox) * wy - (ay - oy) * wx) / cross_dw
        u = ((ax - ox) * dy - (ay - oy) * dx) / cross_dw
        return t >= -1e-9 and 0.0 <= u <= 1.0

    @staticmethod
    def _two_rays_intersect_forward(
        o1x: float, o1y: float, d1x: float, d1y: float, o2x: float, o2y: float, d2x: float, d2y: float
    ) -> bool:
        cross_d = d1x * d2y - d1y * d2x
        if abs(cross_d) < 1e-12:
            return False
        dx, dy = o2x - o1x, o2y - o1y
        t = (dx * d2y - dy * d2x) / cross_d
        s = (dx * d1y - dy * d1x) / cross_d
        return t >= -1e-9 and s >= -1e-9

    @staticmethod
    def _rear_forward_strips_intersect(veh1: BBox, veh2: BBox, heading1_deg: float, heading2_deg: float) -> bool:
        o1 = GeometryHelper.order_rect_points(list(veh1))
        o2 = GeometryHelper.order_rect_points(list(veh2))
        r0a, r1a = InstantSSMCalculator._rear_edge_from_ordered_bbox(o1, heading1_deg)
        r0b, r1b = InstantSSMCalculator._rear_edge_from_ordered_bbox(o2, heading2_deg)
        t1 = math.radians(heading1_deg)
        t2 = math.radians(heading2_deg)
        dax, day = math.cos(t1), math.sin(t1)
        dbx, dby = math.cos(t2), math.sin(t2)

        if InstantSSMCalculator._point_in_forward_strip(r0a, r0b, r1b, dbx, dby) or InstantSSMCalculator._point_in_forward_strip(
            r1a, r0b, r1b, dbx, dby
        ):
            return True
        if InstantSSMCalculator._point_in_forward_strip(r0b, r0a, r1a, dax, day) or InstantSSMCalculator._point_in_forward_strip(
            r1b, r0a, r1a, dax, day
        ):
            return True
        for ox, oy in (r0a, r1a):
            if InstantSSMCalculator._ray_intersect_segment(ox, oy, dax, day, r0b[0], r0b[1], r1b[0], r1b[1]):
                return True
            if InstantSSMCalculator._two_rays_intersect_forward(ox, oy, dax, day, r0b[0], r0b[1], dbx, dby):
                return True
            if InstantSSMCalculator._two_rays_intersect_forward(ox, oy, dax, day, r1b[0], r1b[1], dbx, dby):
                return True
        for ox, oy in (r0b, r1b):
            if InstantSSMCalculator._ray_intersect_segment(ox, oy, dbx, dby, r0a[0], r0a[1], r1a[0], r1a[1]):
                return True
            if InstantSSMCalculator._two_rays_intersect_forward(ox, oy, dbx, dby, r0a[0], r0a[1], dax, day):
                return True
            if InstantSSMCalculator._two_rays_intersect_forward(ox, oy, dbx, dby, r1a[0], r1a[1], dax, day):
                return True
        return GeometryHelper.line_segment_intersection(r0a, r1a, r0b, r1b) is not None

    def _compute_2d_ttc_bbox(
        self, veh1: BBox, veh2: BBox, param1: Tuple[float, float, float, float], param2: Tuple[float, float, float, float]
    ) -> float:
        point1, point2, distance = GeometryHelper.calculate_nearest_points(veh1, veh2)
        if distance <= 0:
            return 0.0
        if not self._rear_forward_strips_intersect(veh1, veh2, param1[2], param2[2]):
            return np.inf

        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        alpha = math.atan2(dy, dx)

        v1, a1 = self._project_to_line(param1[0], param1[1], np.radians(param1[2]), alpha)
        v2, a2 = self._project_to_line(param2[0], param2[1], np.radians(param2[2]), alpha)
        v_rel = v1 - v2
        a_rel = a1 - a2
        w = np.radians(param1[3] + param2[3])
        closing = v_rel + w * distance
        if closing <= _FIRST_ORDER_CLOSING_EPS and a_rel <= _FIRST_ORDER_ACC_EPS:
            return np.inf

        if a_rel == 0:
            if closing <= 0:
                return np.inf
            t = distance / closing
            return t if t > 0 else np.inf

        disc = closing**2 + 2 * a_rel * distance
        if disc < 0:
            return np.inf
        sqrt_disc = math.sqrt(disc)
        t1 = (-closing - sqrt_disc) / a_rel
        t2 = (-closing + sqrt_disc) / a_rel
        candidates = [t for t in (t1, t2) if t > 1e-6]
        if not candidates:
            return np.inf
        return min(candidates)

    @staticmethod
    def _bbox_head_tail_edges(veh: BBox, theta_rad: float) -> Tuple[List[Point], List[Point]]:
        veh_s = sorted(veh)
        if 0.5 < (theta_rad / np.pi) < 1.5:
            return veh_s[:2], veh_s[2:]
        return veh_s[2:], veh_s[:2]

    @staticmethod
    def _point_on_segment(line: Sequence[Point], point: Point) -> bool:
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

    @classmethod
    def _classify_conflict_type(
        cls, point1: Point, point2: Point, veh1: BBox, veh2: BBox, theta1_rad: float, theta2_rad: float
    ) -> Optional[int]:
        head_line1, tail_line1 = cls._bbox_head_tail_edges(veh1, theta1_rad)
        head_line2, tail_line2 = cls._bbox_head_tail_edges(veh2, theta2_rad)
        p1_on_head = cls._point_on_segment(head_line1, point1)
        p1_on_tail = cls._point_on_segment(tail_line1, point1)
        p2_on_head = cls._point_on_segment(head_line2, point2)
        p2_on_tail = cls._point_on_segment(tail_line2, point2)

        # 将航向差归一到 [0, pi]，大于 90 度视为对向行驶。
        heading_diff = abs(theta1_rad - theta2_rad) % (2.0 * np.pi)
        if heading_diff > np.pi:
            heading_diff = 2.0 * np.pi - heading_diff
        is_opposite_direction = heading_diff > (0.5 * np.pi)

        # 规则1：对向 + 碰撞点都在车头线 -> head_on
        if is_opposite_direction and p1_on_head and p2_on_head:
            return 0

        # 规则2：对向但非车头-车头 -> angled
        if is_opposite_direction:
            return 3

        # 同向：碰到尾部 -> rear_end
        if (p1_on_head and p2_on_tail) or (p2_on_head and p1_on_tail):
            return 1

        # 同向：其余接触归为侧擦（含侧面接触、难以稳定判定的同向接触）
        if p1_on_head or p1_on_tail or p2_on_head or p2_on_tail:
            return 2
        return None

    @staticmethod
    def _ray_segment_intersection_distance(origin: Point, direction: Point, a: Point, b: Point) -> Optional[float]:
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

    @classmethod
    def _head_intrusion_time(
        cls,
        follow_bbox: BBox,
        follow_param: Tuple[float, float, float, float],
        lead_bbox: BBox,
        lead_param: Tuple[float, float, float, float],
    ) -> float:
        follow_ordered = GeometryHelper.order_rect_points(list(follow_bbox))
        lead_ordered = GeometryHelper.order_rect_points(list(lead_bbox))
        fh0, fh1 = cls._front_edge_from_ordered_bbox(follow_ordered, follow_param[2])
        theta_f = math.radians(follow_param[2])
        dir_f = (math.cos(theta_f), math.sin(theta_f))
        v_follow = follow_param[0]
        theta_l = math.radians(lead_param[2])
        v_lead_proj = lead_param[0] * math.cos(theta_l - theta_f)
        rel_speed = v_follow - v_lead_proj
        if rel_speed <= 1e-6:
            return np.inf

        def ray_to_bbox_distance(origin: Point) -> Optional[float]:
            best = None
            for i in range(4):
                a = lead_ordered[i]
                b = lead_ordered[(i + 1) % 4]
                t = cls._ray_segment_intersection_distance(origin, dir_f, a, b)
                if t is None:
                    continue
                if best is None or t < best:
                    best = t
            return best

        sample_num = max(PET_HEAD_SAMPLE_NUM, 2)
        rx, ry = (fh1[0] - fh0[0], fh1[1] - fh0[1])
        best_time = np.inf
        for i in range(sample_num):
            a = i / (sample_num - 1)
            q = (fh0[0] + a * rx, fh0[1] + a * ry)
            dist = ray_to_bbox_distance(q)
            if dist is None:
                continue
            t = dist / rel_speed
            if 1e-6 < t < best_time:
                best_time = t
        return best_time

    def _compute_pet(
        self,
        veh1_bbox: BBox,
        param1: Tuple[float, float, float, float],
        veh2_bbox: BBox,
        param2: Tuple[float, float, float, float],
    ) -> float:
        if not self._rear_forward_strips_intersect(veh1_bbox, veh2_bbox, param1[2], param2[2]):
            return np.inf
        t12 = self._head_intrusion_time(veh1_bbox, param1, veh2_bbox, param2)
        t21 = self._head_intrusion_time(veh2_bbox, param2, veh1_bbox, param1)
        return min(t12, t21)


class PeriodSSMCalculator:
    @staticmethod
    def compute_tet_tit(ttc_history: Iterable[float], dt: float, threshold: float) -> Tuple[float, float]:
        tet = 0.0
        tit = 0.0
        for ttc in ttc_history:
            if ttc < threshold:
                tet += dt
                tit += (1.0 / ttc - 1.0 / threshold) * dt
        return tet, tit

    @staticmethod
    def compute_cpi(
        drac_history: Iterable[Tuple[int, float, int]],
        vehicle_class_madr: Dict[int, float],
        dt: float,
    ) -> float:
        records = sorted(list(drac_history), key=lambda x: x[0])
        if not records:
            return 0.0
        total_time = len(records) * dt
        veh_class = records[0][2]
        madr = vehicle_class_madr.get(veh_class, vehicle_class_madr.get(-1, 3.0))
        risky_time = sum(dt for _, drac, _ in records if drac >= madr)
        return risky_time / total_time if total_time > 0 else 0.0
