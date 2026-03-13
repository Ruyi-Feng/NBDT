import os
import matplotlib.pyplot as plt
from box_distance import order_rect_points


def visualize_event(frame, v1, v2, id1, id2, point1, point2, indicators,
                    upper_marks, lower_marks,
                    save_dir="./visualization", show=False, save=True):
    """
    绘制事件场景。
    indicators: 字典，包含 'TTC','MTTC','DRAC','CAI','PET','2D_TTC' 的值（或 None）
    图片将保存在 save_dir/id1_id2/ 目录下。
    """
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

    rect1 = plt.Polygon(bbox1, closed=True, fill=None, edgecolor='red', linewidth=2,
                        label=f"Vehicle {v1['class']} (ID:{id1})")
    plt.gca().add_patch(rect1)
    rect2 = plt.Polygon(bbox2, closed=True, fill=None, edgecolor='blue', linewidth=2,
                        label=f"Vehicle {v2['class']} (ID:{id2})")
    plt.gca().add_patch(rect2)

    plt.scatter([point1[0], point2[0]], [point1[1], point2[1]], color='green', s=50, zorder=5, label='Nearest points')
    plt.plot([point1[0], point2[0]], [point1[1], point2[1]], color='green', linestyle='-', linewidth=2, label='Nearest line')

    triggered = []
    for name, value in indicators.items():
        if value is not None:
            triggered.append(f"{name}={value:.3f}")
    title = f"Frame {frame} - " + ", ".join(triggered) if triggered else f"Frame {frame}"

    plt.title(title)
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend(loc='upper right')
    plt.axis('equal')
    plt.gca().invert_yaxis()  # 匹配图像坐标原点（左上角）
    plt.grid(True, linestyle=':', alpha=0.6)

    if save:
        # 创建子目录：save_dir/id1_id2/
        sub_dir = os.path.join(save_dir, f"{id1}_{id2}")
        os.makedirs(sub_dir, exist_ok=True)
        if triggered:
            first = triggered[0].split('=')[0]
            filename = f"frame_{frame}_{first}_{id1}_{id2}.png"
        else:
            filename = f"frame_{frame}_v{id1}_{id2}.png"
        filepath = os.path.join(sub_dir, filename)
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"Visualization saved: {filepath}")
    if show:
        plt.show()
    plt.close()