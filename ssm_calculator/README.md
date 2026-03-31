# SSM Calculator（重构版）

`ssm_calculator` 是独立的 SSM 计算模块，用于从标准化轨迹 CSV 计算车辆安全替代指标。

本模块已从 `dataloader` 解耦，支持两种分析模式：

- 指定两车 + 指定时段（`pair`）
- 全量主车与左前/正前/右前（`batch`）

## 1. 功能概览

### 1.1 瞬时指标（per frame）

每帧计算以下指标：

- `TTC`
- `MTTC`
- `DRAC`
- `CAI`
- `PET`
- `2D_TTC`
- `conflict_type`

其中 `conflict_type` 目前输出：

- `head_on`
- `rear_end`
- `side_swipe`
- `angled`

### 1.2 时段指标（period）

对一段交互轨迹聚合计算：

- `TIT`
- `TET`
- `cpi`

> 说明：`cpi` 与 `TIT/TET` 一样归属 `period_metrics`。

## 2. 目录与代码结构

### 2.1 核心文件

- `ssm_calculator/ssm.py`
  - `VehicleState`：车辆状态统一数据结构
  - `GeometryHelper`：BBox 几何计算工具
  - `InstantSSMCalculator`：瞬时 SSM 计算
  - `PeriodSSMCalculator`：时段 SSM 计算
- `ssm_calculator/analyzer.py`
  - `TrackDataStore`：轨迹 CSV 预处理（速度分解、加速度、车道方向、投影）
  - `PairPeriodAnalyzer`：指定两车分析
  - `BatchFrontAnalyzer`：全量前向三车分析
  - `LaneConfig` / `PeriodMetricConfig`：参数配置对象
- `ssm_calculator/main.py`
  - 配置读取
  - 运行入口
  - 结果格式化与 JSON 输出
- `ssm_calculator/example_config.json`
  - 完整配置样例

## 3. 输入数据要求

本模块默认读取 NBDT 标准轨迹 CSV（固定字段）。

必需字段：

- `frameNum`, `carId`
- `carCenterXm`, `carCenterYm`
- `boundingBox1Xm`, `boundingBox1Ym`
- `boundingBox2Xm`, `boundingBox2Ym`
- `boundingBox3Xm`, `boundingBox3Ym`
- `boundingBox4Xm`, `boundingBox4Ym`
- `heading`, `speed`, `objClass`, `laneId`

额外字段可以存在，会被忽略。

## 4. 快速开始

### 4.1 安装依赖

在项目根目录执行：

```bash
pip install -r requirements.txt
```

### 4.2 准备配置文件

复制 `ssm_calculator/example_config.json`，例如新建为：

- `ssm_calculator/my_config.json`

至少需要修改：

- `tracks_file`: 你的轨迹 CSV 绝对/相对路径
- `mode`: `pair` 或 `batch`

### 4.3 运行命令

在项目根目录执行：

```bash
python -m ssm_calculator.main --config ssm_calculator/my_config.json --output ./output/ssm_result.json
```

如果不传 `--output`，结果会打印到标准输出。

## 5. 配置详解（example_config.json）

```json
{
  "tracks_file": "path/to/your_tracks.csv",
  "mode": "batch",
  "start_frame": null,
  "end_frame": null,
  "lane": {
    "left_lane_offset": -1,
    "right_lane_offset": 1,
    "lane_direction_similarity_deg": 45.0
  },
  "period_metric": {
    "tet_tit_threshold": 10.0,
    "vehicle_class_madr": {
      "0": 3.0,
      "1": 3.0,
      "2": 2.0,
      "3": 2.0,
      "4": 4.0,
      "5": 10.0,
      "-1": 3.0
    }
  },
  "pair": {
    "ego_id": 19,
    "target_id": 23
  },
  "batch": {
    "relations": ["left_front", "front", "right_front"]
  }
}
```

### 5.1 通用字段

- `tracks_file`: 轨迹 CSV 文件路径
- `mode`:
  - `pair`：指定两车分析
  - `batch`：全量主车前向三车
- `start_frame`, `end_frame`:
  - `null` 表示自动使用全帧范围

### 5.2 lane 参数

- `left_lane_offset`: 左车道相对偏移（默认 `-1`）
- `right_lane_offset`: 右车道相对偏移（默认 `+1`）
- `lane_direction_similarity_deg`: 跨车道方向一致性阈值（度）

### 5.3 period_metric 参数

- `tet_tit_threshold`: 计算 `TET/TIT` 的 TTC 阈值
- `vehicle_class_madr`: 各类车最大减速度（用于 `cpi`）

### 5.4 pair / batch 专属参数

- `pair.ego_id`, `pair.target_id`: 在 `pair` 模式下生效
- `batch.relations`: 在 `batch` 模式下生效，可选：
  - `left_front`
  - `front`
  - `right_front`

## 6. 输出 JSON 结构（当前版本）

```json
{
  "meta": {
    "tracks_file": "...",
    "mode": "batch",
    "frame_rate": 25.0,
    "time_step": 0.04,
    "lane": { "...": "..." },
    "period_metric": { "...": "..." }
  },
  "vehicles": {
    "ego_id": {
      "targets": {
        "target_id": {
          "start_frame": 100,
          "end_frame": 150,
          "instant_metrics": [
            {
              "frame": 101,
              "relation": "front",
              "TTC": 2.5,
              "MTTC": 2.3,
              "DRAC": 1.2,
              "CAI": 15.7,
              "PET": null,
              "2D_TTC": null,
              "conflict_type": "rear_end"
            }
          ],
          "period_metrics": {
            "TIT": 0.234,
            "TET": 0.056,
            "cpi": 0.018
          }
        }
      }
    }
  }
}
```

说明：

- `instant_metrics`：每帧指标（瞬时指标）
- `period_metrics`：该 ego-target 交互区间的时段指标
- 所有无穷值（`inf`）会被转成 `null`
- `relation` 在每帧提供，表示该帧瞬时相对关系

## 7. 两种模式使用建议

### 7.1 pair 模式

适合以下场景：

- 已知要分析的主车和目标车
- 需要聚焦某段时窗（例如事故前 5 秒）

建议配置：

- 设置 `mode = "pair"`
- 配置 `pair.ego_id` 与 `pair.target_id`
- 设置 `start_frame/end_frame` 限定分析范围

### 7.2 batch 模式

适合以下场景：

- 需要对整段数据批量扫描
- 统计所有主车与前向邻车的风险暴露

建议配置：

- 设置 `mode = "batch"`
- `batch.relations` 默认保留 `left_front/front/right_front`

## 8. 常见问题（FAQ）

### 8.1 为什么有些指标是 `null`？

当对应指标物理上不可定义或计算结果为无穷时，会输出 `null`。例如：

- 相对不闭合时 TTC 类指标可能为 `inf`
- 输出阶段统一将 `inf` 转为 `null`

### 8.2 为什么某帧 relation 是 `unknown`？

`pair` 模式会逐帧查询该目标是否位于前向三车关系中；若不属于 `left_front/front/right_front`，会标记为 `unknown`。
