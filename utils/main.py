import os
import numpy as np
from ssm_core import SSMCalculator
from data_loader import SafetyAnalyzer
from box_distance import calculate_nearest_points
from output_json import OutputWriter
import visualizer

# ===== 车辆类型对应的最大可用减速度（用于 CPI）=====
MADR_CAR = 3
MADR_TAXI = 3
MADR_TRUCK = 2
MADR_BUS = 2
MADR_MOTO = 4
MADR_PED = 10.0
MADR_UNKNOWN = 3
VEHICLE_CLASS_MADR = {
    0: MADR_CAR,
    1: MADR_TAXI,
    2: MADR_BUS,
    3: MADR_TRUCK,
    4: MADR_MOTO,
    5: MADR_PED,
    -1: MADR_UNKNOWN
}

# ===== 危险阈值（用户可调整）=====
TTC_THRESHOLD = 2.0
MTTC_THRESHOLD = 2.0
PET_THRESHOLD = 2.0
TTC2D_THRESHOLD = 2.0
# ================================

def format_value(val):
    if np.isinf(val):
        return "inf"
    else:
        return f"{val:.4f}"

def main():
    meta_file = r"E:\xizihao\Internship\zhiling\work\NBDT\02_recordingMeta.csv"
    tracks_file = r"E:\xizihao\Internship\zhiling\work\NBDT\KZM6.csv"

    analyzer = SafetyAnalyzer(meta_file, tracks_file)

    # ===== 用户自定义参数 =====
    EGO_ID = 20               # 指定主车ID，None表示遍历所有车
    TARGET_IDS = []             # 指定目标ID列表，为空则使用周围车辆
    START_FRAME = None          # 起始帧，None表示第一帧
    END_FRAME = None            # 结束帧，None表示最后一帧
    ENABLE_VISUAL = True        # 是否保存触发事件的图像
    ENABLE_OUTPUT = True          # 是否保存JSON结果
    OUTPUT_DIR = "./output"       # 输出目录
    # =========================

    frames = analyzer.get_frame_range()
    start = START_FRAME if START_FRAME is not None else frames[0]
    end = END_FRAME if END_FRAME is not None else frames[-1]

    if ENABLE_OUTPUT:
        # 生成文件名前缀
        filename_base = os.path.splitext(os.path.basename(tracks_file))[0]
        writer = OutputWriter(OUTPUT_DIR, filename_base)
        writer.set_meta({
            "recording_file": os.path.basename(meta_file),
            "tracks_file": os.path.basename(tracks_file),
            "frame_rate": analyzer.frame_rate,
            "time_step": analyzer.dt,
            "lane_markings_upper": analyzer.upper_marks,
            "lane_markings_lower": analyzer.lower_marks,
            "thresholds": {
                "TTC": TTC_THRESHOLD,
                "MTTC": MTTC_THRESHOLD,
                "PET": PET_THRESHOLD,
                "2D_TTC": TTC2D_THRESHOLD}})

    if EGO_ID is not None:
        ego_ids = [EGO_ID]
    else:
        all_ids = set()
        for frame in frames:
            all_ids.update(analyzer.frame_vehicles[frame].keys())
        ego_ids = sorted(all_ids)

    calculator = SSMCalculator()

    for ego_id in ego_ids:
        print(f"\n======= Processing ego vehicle {ego_id} =======")
        drac_history_ego = []          # 用于CPI： (frame, drac, class)
        # key: target_id, value: {'frames_data': list, 'has_trigger': bool, 'ttc_history': list}
        target_scenes = {}

        for frame in frames:
            if frame < start or frame > end:
                continue

            ego_info = analyzer.get_vehicle_info(frame, ego_id)
            if ego_info is None:
                # 主车消失：结束所有相关场景，并计算TET/TIT
                for target_id, scene in target_scenes.items():
                    # 可视化（如果该场景曾触发危险）
                    if ENABLE_VISUAL and scene['has_trigger']:
                        for f_data in scene['frames_data']:
                            f_frame, f_v1, f_v2, f_ssm, f_rel = f_data
                            point1, point2, dist = calculate_nearest_points(f_v1['bbox'], f_v2['bbox'])
                            upper, lower = analyzer.get_upper_lower_marks()
                            visualizer.visualize_event(
                                f_frame, f_v1, f_v2, ego_id, target_id,
                                point1, point2, f_ssm,
                                upper, lower,
                                save=True, show=False
                            )

                    if ENABLE_OUTPUT:
                        frames_ssm = []
                        for f_data in scene['frames_data']:
                            f_frame, _, _, f_ssm, f_rel = f_data
                            ssm_clean = {k: (float(v) if not np.isinf(v) else None) for k, v in f_ssm.items()}
                            frames_ssm.append({
                                "frame": int(f_frame),
                                "relation": f_rel,
                                **ssm_clean
                            })
                        if scene['ttc_history']:
                            tet, tit = SSMCalculator.compute_tet_tit_from_history(
                                scene['ttc_history'], analyzer.dt, TTC_THRESHOLD
                            )
                        else:
                            tet, tit = None, None
                        first_rel = scene['frames_data'][0][4] if scene['frames_data'] else 'unknown'
                        writer.add_target_interaction(
                            ego_id, target_id, first_rel,
                            scene['frames_data'][0][0], scene['frames_data'][-1][0],
                            frames_ssm, tit, tet
                        )

                target_scenes.clear()
                continue

            # 获取当前帧的目标车辆列表
            if TARGET_IDS:
                # 先获取当前帧的所有周围车辆关系
                surrounding = analyzer.get_surrounding_vehicles(frame, ego_id)
                rel_dict = {tid: rel for tid, rel in surrounding}
                target_ids = []
                for tid in TARGET_IDS:
                    rel = rel_dict.get(tid, 'unknown')
                    target_ids.append((tid, rel))
            else:
                surrounding = analyzer.get_surrounding_vehicles(frame, ego_id)
                target_ids = [(tid, rel) for tid, rel in surrounding]

            current_targets = set()   # 当前帧存在的目标

            for target_id, relation in target_ids:
                if analyzer.get_vehicle_info(frame, target_id) is None:
                    continue

                ssm = analyzer.compute_ssm_for_pair(frame, ego_id, target_id)
                if ssm is None:
                    continue

                # 打印当前帧的SSM指标
                print(f"Frame {frame}: ego {ego_id} -> target {target_id} (relation: {relation})")
                for key in ['TTC', 'MTTC', 'DRAC', 'CAI', 'PET', '2D_TTC']:
                    print(f"  {key}: {format_value(ssm[key])}")

                # 收集DRAC用于CPI（仅当前方跟驰时）
                if not np.isinf(ssm['DRAC']) and ('front' in relation):
                    drac_history_ego.append((frame, ssm['DRAC'], ego_info['class']))

                # 判断当前帧是否触发危险（用于可视化标记）
                trigger = False
                if not np.isinf(ssm['TTC']) and ssm['TTC'] < TTC_THRESHOLD:
                    trigger = True
                if not np.isinf(ssm['MTTC']) and ssm['MTTC'] < MTTC_THRESHOLD:
                    trigger = True
                if not np.isinf(ssm['PET']) and ssm['PET'] < PET_THRESHOLD:
                    trigger = True
                if not np.isinf(ssm['2D_TTC']) and ssm['2D_TTC'] < TTC2D_THRESHOLD:
                    trigger = True

                current_targets.add(target_id)

                # 初始化场景记录
                if target_id not in target_scenes:
                    target_scenes[target_id] = {
                        'frames_data': [],
                        'has_trigger': False,
                        'ttc_history': []
                    }

                # 记录当前帧数据
                v1 = analyzer.get_vehicle_info(frame, ego_id)
                v2 = analyzer.get_vehicle_info(frame, target_id)
                target_scenes[target_id]['frames_data'].append((frame, v1, v2, ssm, relation))
                if not np.isinf(ssm['TTC']):
                    target_scenes[target_id]['ttc_history'].append(ssm['TTC'])
                if trigger:
                    target_scenes[target_id]['has_trigger'] = True

            # 检查哪些目标车在当前帧消失了，结束它们的场景
            finished_targets = [tid for tid in target_scenes if tid not in current_targets]
            for tid in finished_targets:
                scene = target_scenes.pop(tid)
                # 可视化
                if ENABLE_VISUAL and scene['has_trigger']:
                    for f_data in scene['frames_data']:
                        f_frame, f_v1, f_v2, f_ssm, f_rel = f_data
                        point1, point2, dist = calculate_nearest_points(f_v1['bbox'], f_v2['bbox'])
                        upper, lower = analyzer.get_upper_lower_marks()
                        visualizer.visualize_event(
                            f_frame, f_v1, f_v2, ego_id, tid,
                            point1, point2, f_ssm,
                            upper, lower,
                            save=True, show=False)

                if ENABLE_OUTPUT:
                    frames_ssm = []
                    for f_data in scene['frames_data']:
                        f_frame, _, _, f_ssm, f_rel = f_data
                        ssm_clean = {k: (float(v) if not np.isinf(v) else None) for k, v in f_ssm.items()}
                        frames_ssm.append({
                            "frame": int(f_frame),
                            "relation": f_rel,
                            **ssm_clean})
                    if scene['ttc_history']:
                        tet, tit = SSMCalculator.compute_tet_tit_from_history(
                            scene['ttc_history'], analyzer.dt, TTC_THRESHOLD)
                    else:
                        tet, tit = None, None
                    first_rel = scene['frames_data'][0][4] if scene['frames_data'] else 'unknown'
                    writer.add_target_interaction(
                        ego_id, tid, first_rel,
                        scene['frames_data'][0][0], scene['frames_data'][-1][0],
                        frames_ssm, tit, tet)

        # 主车循环结束：处理所有尚未结束的场景
        for target_id, scene in target_scenes.items():
            if ENABLE_VISUAL and scene['has_trigger']:
                for f_data in scene['frames_data']:
                    f_frame, f_v1, f_v2, f_ssm, f_rel = f_data
                    point1, point2, dist = calculate_nearest_points(f_v1['bbox'], f_v2['bbox'])
                    upper, lower = analyzer.get_upper_lower_marks()
                    visualizer.visualize_event(
                        f_frame, f_v1, f_v2, ego_id, target_id,
                        point1, point2, f_ssm,
                        upper, lower,
                        save=True, show=False)

            if ENABLE_OUTPUT:
                frames_ssm = []
                for f_data in scene['frames_data']:
                    f_frame, _, _, f_ssm, f_rel = f_data
                    ssm_clean = {k: (float(v) if not np.isinf(v) else None) for k, v in f_ssm.items()}
                    frames_ssm.append({
                        "frame": int(f_frame),
                        "relation": f_rel,
                        **ssm_clean})
                if scene['ttc_history']:
                    tet, tit = SSMCalculator.compute_tet_tit_from_history(
                        scene['ttc_history'], analyzer.dt, TTC_THRESHOLD)
                else:
                    tet, tit = None, None
                first_rel = scene['frames_data'][0][4] if scene['frames_data'] else 'unknown'
                writer.add_target_interaction(
                    ego_id, target_id, first_rel,
                    scene['frames_data'][0][0], scene['frames_data'][-1][0],
                    frames_ssm, tit, tet)

        # 计算当前主车的CPI
        cpi = SSMCalculator.compute_cpi_from_drac_history(drac_history_ego, VEHICLE_CLASS_MADR, analyzer.dt)
        print(f"CPI for ego vehicle {ego_id}: {cpi:.4f}")
        if ENABLE_OUTPUT:
            writer.set_cpi(ego_id, cpi)
    if ENABLE_OUTPUT:
        writer.write()

if __name__ == "__main__":
    main()