import os
import json
import numpy as np
from collections import defaultdict


class OutputWriter:
    def __init__(self, output_dir, filename_base):
        """
        output_dir: 输出目录
        filename_base: 文件名前缀（通常取自轨迹文件名）
        """
        self.output_dir = output_dir
        self.filename_base = filename_base
        os.makedirs(output_dir, exist_ok=True)
        self.data = {
            "meta": {},          # 元数据（帧率、车道标记等）
            "vehicles": {}       # 按 ego_id 组织的车辆交互数据
        }

    def set_meta(self, meta_dict):
        """设置元数据"""
        self.data["meta"] = meta_dict

    def add_ego(self, ego_id):
        """初始化一个主车的数据结构（如果不存在）"""
        if str(ego_id) not in self.data["vehicles"]:
            self.data["vehicles"][str(ego_id)] = {
                "targets": {},
                "cpi": None
            }

    def add_target_interaction(self, ego_id, target_id, relation, start_frame, end_frame,
                                frames_ssm, tit=None, tet=None):
        """
        记录一个主车与目标车的完整交互。

        参数：
            ego_id: 主车ID
            target_id: 目标车ID
            relation: 相对位置关系（如 'front', 'left_front' 等，取起始帧的关系）
            start_frame: 起始帧
            end_frame: 终止帧
            frames_ssm: 列表，每个元素为字典，包含 'frame' 和所有SSM指标（值已转换为float或None）
            tit: 该交互的 TIT 值
            tet: 该交互的 TET 值
        """
        self.add_ego(ego_id)

        # 存储交互数据
        self.data["vehicles"][str(ego_id)]["targets"][str(target_id)] = {
            "relation": relation,
            "start_frame": start_frame,
            "end_frame": end_frame,
            "frames": frames_ssm,   # 每帧的SSM值
            "aggregated": {
                "TIT": tit,
                "TET": tet
            }
        }

    def set_cpi(self, ego_id, cpi):
        """设置主车的 CPI 值"""
        self.add_ego(ego_id)
        self.data["vehicles"][str(ego_id)]["cpi"] = cpi

    def write(self):
        """将收集的数据写入 JSON 文件"""
        filepath = os.path.join(self.output_dir, f"{self.filename_base}_results.json")

        # 递归处理数据，将 numpy 类型转换为原生类型，并处理 NaN/Inf
        def clean(obj):
            if isinstance(obj, np.integer):
                return int(obj)
            elif isinstance(obj, np.floating):
                if np.isnan(obj) or np.isinf(obj):
                    return None
                return float(obj)
            elif isinstance(obj, dict):
                return {k: clean(v) for k, v in obj.items()}
            elif isinstance(obj, (list, tuple)):
                return [clean(item) for item in obj]
            else:
                return obj

        data_clean = clean(self.data)
        with open(filepath, 'w') as f:
            json.dump(data_clean, f, indent=2)
        print(f"Results saved to {filepath}")