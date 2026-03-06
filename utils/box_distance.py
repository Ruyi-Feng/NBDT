import math


def order_rect_points(points):
    centroid_x = sum(p[0] for p in points) / 4
    centroid_y = sum(p[1] for p in points) / 4

    def sort_key(point):
        return math.atan2(point[1] - centroid_y, point[0] - centroid_x)

    sorted_points = sorted(points, key=sort_key)  
    a, b, c, d = sorted_points
    cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    if cross > 0:  
        return [a, d, c, b]
    else:
        return sorted_points


def is_inside_rect(point, rect):
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


def line_segment_intersection(a1, a2, b1, b2):
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


def closest_point_on_segment(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx = bx - ax
    dy = by - ay
    if dx == 0 and dy == 0:
        return a
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0, min(1, t))
    return (ax + t * dx, ay + t * dy)


def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def segment_segment_closest_points(a1, a2, b1, b2):
    intersect = line_segment_intersection(a1, a2, b1, b2)
    if intersect:
        return (intersect, intersect, 0.0)

    candidates = []
    for p in [a1, a2]:
        closest = closest_point_on_segment(p, b1, b2)
        candidates.append((p, closest, distance(p, closest)))
    for p in [b1, b2]:
        closest = closest_point_on_segment(p, a1, a2)
        candidates.append((closest, p, distance(closest, p)))

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
            candidates.append((pa, pb, distance(pa, pb)))

    candidates.sort(key=lambda x: x[2])
    return candidates[0] if candidates else (None, None, float('inf'))


def rectangles_intersect(rect1, rect2):
    axes = []
    for i in range(4):
        a, b = rect1[i], rect1[(i + 1) % 4]
        edge = (b[0] - a[0], b[1] - a[1])
        normal = (edge[1], -edge[0])
        length = math.hypot(*normal)
        if length == 0:
            continue
        axes.append((normal[0] / length, normal[1] / length))
    for i in range(4):
        a, b = rect2[i], rect2[(i + 1) % 4]
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


def calculate_nearest_points(veh1, veh2):
    rect1 = order_rect_points(veh1)
    rect2 = order_rect_points(veh2)

    if rectangles_intersect(rect1, rect2):
        for p in rect1:
            if is_inside_rect(p, rect2):
                return (p, p, 0.0)
        for p in rect2:
            if is_inside_rect(p, rect1):
                return (p, p, 0.0)
        for i in range(4):
            a1, a2 = rect1[i], rect1[(i + 1) % 4]
            for j in range(4):
                b1, b2 = rect2[j], rect2[(j + 1) % 4]
                intersect = line_segment_intersection(a1, a2, b1, b2)
                if intersect:
                    return (intersect, intersect, 0.0)
        center1 = (sum(p[0] for p in rect1) / 4, sum(p[1] for p in rect1) / 4)
        center2 = (sum(p[0] for p in rect2) / 4, sum(p[1] for p in rect2) / 4)
        return (center1, center2, 0.0)
    else:
        min_dist = float('inf')
        closest_pair = None

        for p in rect1:
            for j in range(4):
                b1, b2 = rect2[j], rect2[(j + 1) % 4]
                closest = closest_point_on_segment(p, b1, b2)
                d = distance(p, closest)
                if d < min_dist:
                    min_dist = d
                    closest_pair = (p, closest)

        for p in rect2:
            for i in range(4):
                a1, a2 = rect1[i], rect1[(i + 1) % 4]
                closest = closest_point_on_segment(p, a1, a2)
                d = distance(closest, p)
                if d < min_dist:
                    min_dist = d
                    closest_pair = (closest, p)

        for i in range(4):
            a1, a2 = rect1[i], rect1[(i + 1) % 4]
            for j in range(4):
                b1, b2 = rect2[j], rect2[(j + 1) % 4]
                pa, pb, d = segment_segment_closest_points(a1, a2, b1, b2)
                if d < min_dist:
                    min_dist = d
                    closest_pair = (pa, pb)

        return (*closest_pair, min_dist)