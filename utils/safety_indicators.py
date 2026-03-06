import os
import math
from collections import defaultdict

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from box_distance import calculate_nearest_points, order_rect_points

# ========================= Configuration Parameters =========================
# General
TTC_THRESHOLD = 5               # TTC threshold for TIT/TET
MIN_VEHICLES_FOR_LANE_DIR = 2

# Maximum available deceleration of the vehicle (m/s²) for CPI
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

# Minimum print threshold
PET_PRINT_THRESHOLD = 0.2
TTC2D_PRINT_THRESHOLD = 0.3

# Same-direction judgment threshold (for 2D TTC)
HEADING_DIFF_THRESHOLD = 45      # Heading difference less than this value is considered same-direction

# PET angle range (PET is calculated only within this range, excluding same-direction and opposite-direction)
PET_MIN_ANGLE = 10
PET_MAX_ANGLE = 150

# Visualization toggle
ENABLE_VISUALIZATION = False
VISUALIZATION_OUTPUT_DIR = "./visualization"
SHOW_VISUALIZATION = False
SAVE_VISUALIZATION = True
# ============================================================

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


def estimate_lane_direction(vehicles_in_lane):
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


class ACT:
    def __init__(self):
        self.show_collision = False

    def project_to_line(self, v, a, theta, alpha):
        delta = theta - alpha
        v_proj = v * np.cos(delta)
        a_proj = a * np.cos(delta)
        return v_proj, a_proj

    def get_act(self, v, a, w, distance, scale=1.0):
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
        point1, point2, distance = calculate_nearest_points(veh1, veh2)
        alpha = self.get_direction_vector(point1, point2)
        v1, a1 = self.project_to_line(param1[0], param1[1], np.radians(param1[2]), alpha)
        v2, a2 = self.project_to_line(param2[0], param2[1], np.radians(param2[2]), alpha)
        v = v1 - v2
        a = a1 - a2
        w = np.radians(param1[3] + param2[3])
        act = self.get_act(v, a, w, distance, scale)
        return act


def compute_pet(veh1, param1, veh2, param2, angle_diff):
    if angle_diff < PET_MIN_ANGLE or angle_diff > PET_MAX_ANGLE:
        return np.inf

    cx1 = sum(p[0] for p in veh1) / 4
    cy1 = sum(p[1] for p in veh1) / 4
    cx2 = sum(p[0] for p in veh2) / 4
    cy2 = sum(p[1] for p in veh2) / 4

    angle1 = np.radians(param1[2])
    angle2 = np.radians(param2[2])

    def intersection_point(x1, y1, theta1, x2, y2, theta2):
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

    if np.dot(dir1, vec1) < 0 or np.dot(dir2, vec2) < 0:
        return np.inf

    dist1 = math.hypot(vec1[0], vec1[1])
    dist2 = math.hypot(vec2[0], vec2[1])
    if v1 <= 0 or v2 <= 0:
        return np.inf
    time1 = dist1 / v1
    time2 = dist2 / v2
    return abs(time1 - time2)


def visualize_event(frame, v1, v2, id1, id2, point1, point2, pet, ttc, upper_marks, lower_marks):
    """Draw the scene of a small-value event."""
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
        print(f"Visualization saved: {filepath}")
    if SHOW_VISUALIZATION:
        plt.show()
    plt.close()


def main():
    # Modify the file path according to the actual situation.
    meta_file = r"D:\NBDT\01_recordingMeta.csv"
    tracks_file = r"D:\NBDT\01_tracks.csv"

    frame_rate, upper_marks, lower_marks = load_metadata(meta_file)
    dt = 1.0 / frame_rate
    print(f"Frame rate: {frame_rate} fps, time step: {dt:.4f} s")
    print(f"Lane markings - upper: {upper_marks}, lower: {lower_marks}")

    df_tracks = load_tracks(tracks_file)
    frames = sorted(df_tracks['frameNum'].unique())
    total_frames = len(frames)

    frame_vehicles = defaultdict(dict)
    vehicle_trajectories = defaultdict(list)
    prev_frame_info = defaultdict(dict)

    act_calculator = ACT()

    ttc_1d_series = defaultdict(list)      # (follow_id, lead_id) -> list of (frame, ttc)
    mttc_1d_series = defaultdict(list)     # (follow_id, lead_id) -> list of (frame, mttc)
    cai_series = defaultdict(list)          # (follow_id, lead_id) -> list of (frame, cai)
    drac_series = defaultdict(list)         # follow_id -> list of (frame, drac, class)
    ttc_2d_results = []                     # (frame, id1, id2, act)
    pet_results = []                         # (frame, id1, id2, pet)

    processed_pairs_1d = 0

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

        # Group vehicles by lane
        lane_groups = defaultdict(list)
        for cid in car_ids:
            info = vehicles_this_frame[cid]
            lane = info['laneId']
            lane_groups[lane].append((cid, info))

        for lane, group in lane_groups.items():
            lane_dir_deg = estimate_lane_direction([info for (_, info) in group])
            if lane_dir_deg is None:
                continue
            rad = np.radians(lane_dir_deg)
            cosθ, sinθ = np.cos(rad), np.sin(rad)

            proj_list = []
            for cid, info in group:
                proj_dist = info['x'] * cosθ + info['y'] * sinθ
                proj_speed = info['vx'] * cosθ + info['vy'] * sinθ
                heading_rad = np.radians(info['heading'])
                delta = heading_rad - rad
                proj_acc = info['a'] * np.cos(delta)
                proj_list.append((cid, info, proj_dist, proj_speed, proj_acc))

            proj_list.sort(key=lambda x: x[2])  # Sort by projected distance

            for i in range(len(proj_list) - 1):
                follow_id, follow_info, follow_dist, follow_speed, follow_acc = proj_list[i]
                lead_id, lead_info, lead_dist, lead_speed, lead_acc = proj_list[i + 1]

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

        # ========== 2D TTC and PET for all vehicle pairs (same-direction only) ==========
        for i in range(n):
            for j in range(i + 1, n):
                id1 = car_ids[i]
                id2 = car_ids[j]
                v1 = vehicles_this_frame[id1]
                v2 = vehicles_this_frame[id2]

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

                # Print and visualize small-value events
                need_print = False
                if not np.isinf(pet) and pet < PET_PRINT_THRESHOLD:
                    need_print = True
                if not np.isinf(act) and act < TTC2D_PRINT_THRESHOLD:
                    need_print = True

                if need_print:
                    point1, point2, distance = calculate_nearest_points(v1['bbox'], v2['bbox'])
                    print("\n===== Small value event =====")
                    print(f"Frame: {frame}")
                    print(f"Vehicle 1 ID: {id1}, class: {v1['class']}, speed: {v1['speed']:.2f} m/s, "
                          f"heading: {v1['heading']:.2f}°, acceleration: {v1['a']:.2f} m/s², center: ({v1['x']:.2f}, {v1['y']:.2f})")
                    print(f"Vehicle 2 ID: {id2}, class: {v2['class']}, speed: {v2['speed']:.2f} m/s, "
                          f"heading: {v2['heading']:.2f}°, acceleration: {v2['a']:.2f} m/s², center: ({v2['x']:.2f}, {v2['y']:.2f})")
                    print(f"Nearest point 1: ({point1[0]:.2f}, {point1[1]:.2f}), nearest point 2: ({point2[0]:.2f}, {point2[1]:.2f}), distance: {distance:.2f} m")
                    print(f"PET: {pet:.4f} s")
                    print(f"2D TTC: {act:.4f} s")
                    print("==============================")

                    if ENABLE_VISUALIZATION:
                        visualize_event(frame, v1, v2, id1, id2, point1, point2, pet, act,
                                        upper_marks, lower_marks)

    print(f"Total processed 1D following pairs: {processed_pairs_1d}")

    # ========== Print summary statistics ==========
    # TIT/TET
    print("\n--- TIT/TET (based on 1D TTC) ---")
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
        print(f"Pair ({follow_id}->{lead_id}): TET = {TET:.4f} s, TIT = {TIT:.4f} s")
    print(f"Total: {tet_nonzero} pairs TET > 0, {tit_nonzero} pairs TIT > 0")

    # CPI
    print("\n--- CPI (per following vehicle) ---")
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
        print(f"Vehicle {follow_id} (class {veh_class}, MADR={madr}): CPI = {cpi:.4f}")
    nonzero_cpi = sum(1 for cpi in cpi_per_vehicle.values() if cpi > 0)
    print(f"Total: {nonzero_cpi} vehicles with CPI > 0")

    # 2D TTC
    print("\n--- 2D TTC (ACT) statistics ---")
    valid_ttc2 = [t for _, _, _, t in ttc_2d_results if t < 20]
    if valid_ttc2:
        print(f"Valid 2D TTC count: {len(valid_ttc2)}")
        print(f"Min TTC: {min(valid_ttc2):.4f} s, avg TTC: {np.mean(valid_ttc2):.4f} s")
    else:
        print("No valid 2D TTC")

    # PET
    print("\n--- PET statistics ---")
    valid_pet = [p for _, _, _, p in pet_results if not np.isinf(p)]
    if valid_pet:
        print(f"Valid PET count: {len(valid_pet)}")
        print(f"Min PET: {min(valid_pet):.4f} s, avg PET: {np.mean(valid_pet):.4f} s")
    else:
        print("No valid PET")

    # MTTC
    print("\n--- MTTC statistics (1D) ---")
    mttc_all = []
    for ttc_list in mttc_1d_series.values():
        for _, m in ttc_list:
            if not np.isinf(m):
                mttc_all.append(m)
    if mttc_all:
        print(f"Valid MTTC count: {len(mttc_all)}")
        print(f"Min MTTC: {min(mttc_all):.4f} s, avg MTTC: {np.mean(mttc_all):.4f} s")
    else:
        print("No valid MTTC")

    # CAI
    print("\n--- CAI statistics (1D) ---")
    cai_all = []
    for cai_list in cai_series.values():
        for _, c in cai_list:
            if not np.isinf(c):
                cai_all.append(c)
    if cai_all:
        print(f"Valid CAI count: {len(cai_all)}")
        print(f"Min CAI: {min(cai_all):.4f}, avg CAI: {np.mean(cai_all):.4f}")
        cai_neg = sum(1 for c in cai_all if c < 0)
        print(f"Number of negative values: {cai_neg}")
    else:
        print("No valid CAI")


if __name__ == "__main__":
    main()