import os
import shutil
from pathlib import Path

# 定义基础路径
base_dir = Path("/home/inkspring/RoboMaster/XJTU_RMAC/project_04/data")

# 源路径
unlabeled_img_dir = base_dir / "unlabeled" / "images"
pseudo_lbl_dir = base_dir / "pseudo_labels_clean"

# 目标路径（现有的训练集目录）
train_img_dir = base_dir / "train" / "images"
train_lbl_dir = base_dir / "train" / "labels"

# 确保目标目录存在
train_img_dir.mkdir(parents=True, exist_ok=True)
train_lbl_dir.mkdir(parents=True, exist_ok=True)

# 统计计数器
moved_count = 0
error_count = 0

print("=== 开始将伪标签数据合并至训练集 ===")

# 遍历清洗后的伪标签文件
for lbl_file in pseudo_lbl_dir.glob("*.txt"):
    # 获取对应的图片文件名 (将 .txt 替换为 .jpg)
    img_file_name = lbl_file.stem + ".jpg"
    src_img_path = unlabeled_img_dir / img_file_name
    
    # 检查图片是否存在
    if src_img_path.exists():
        # 移动图片到训练集 images 目录
        shutil.move(str(src_img_path), str(train_img_dir / img_file_name))
        # 移动标签到训练集 labels 目录
        shutil.move(str(lbl_file), str(train_lbl_dir / lbl_file.name))
        moved_count += 1
    else:
        print(f"⚠️ 警告: 找不到对应的图片文件 {img_file_name}")
        error_count += 1

print(f"✅ 合并完成！")
print(f"成功将 {moved_count} 对 图片-标签 添加至训练集。")
if error_count > 0:
    print(f"⚠️ 有 {error_count} 个标签未找到对应图片，已跳过。")

# 统计最终训练集和验证集规模
train_imgs = len(list(train_img_dir.glob("*.jpg")))
val_imgs = len(list((base_dir / "val" / "images").glob("*.jpg")))
print(f"\n📊 最终数据集规模:")
print(f"训练集 (Train): {train_imgs} 张图片")
print(f"验证集 (Val):   {val_imgs} 张图片 (保持不变)")