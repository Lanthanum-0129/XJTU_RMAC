import os
from pathlib import Path

src_label_dir = Path("/home/inkspring/RoboMaster/XJTU_RMAC/project_04/runs/detect/pseudo_gen/labels")
dst_label_dir = Path("/home/inkspring/RoboMaster/XJTU_RMAC/project_04/data/pseudo_labels_clean")
dst_label_dir.mkdir(exist_ok=True)

CONF_THRESHOLD = 0.65  # 最终过滤阈值

for txt_file in src_label_dir.glob("*.txt"):
    with open(txt_file, 'r') as f:
        lines = f.readlines()
    
    valid_lines = []
    for line in lines:
        parts = line.strip().split()
        if len(parts) == 6:  # YOLO带置信度的格式: class x y w h conf
            conf = float(parts[5])
            if conf >= CONF_THRESHOLD:
                # 过滤后，将置信度列去掉，恢复为标准YOLO格式 (class x y w h)
                valid_lines.append(" ".join(parts[:5]) + "\n")
                
    # 只有包含有效标签的文件才保留，避免引入纯背景噪声
    if valid_lines:
        with open(dst_label_dir / txt_file.name, 'w') as f:
            f.writelines(valid_lines)

print(f"伪标签清洗完毕！有效标签文件已保存至 {dst_label_dir}")