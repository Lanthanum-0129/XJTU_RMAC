import os
import shutil
import random

# 设置随机种子以保证结果可复现
random.seed(42)

# 定义路径
base_dir = '/home/inkspring/RoboMaster/XJTU_RMAC/project_04/data'
src_img_dir = os.path.join(base_dir, 'labeled', 'images')
src_lbl_dir = os.path.join(base_dir, 'labeled', 'labels')

# 定义目标路径
train_img_dir = os.path.join(base_dir, 'train', 'images')
train_lbl_dir = os.path.join(base_dir, 'train', 'labels')
val_img_dir = os.path.join(base_dir, 'val', 'images')
val_lbl_dir = os.path.join(base_dir, 'val', 'labels')

# 创建目标目录
for d in [train_img_dir, train_lbl_dir, val_img_dir, val_lbl_dir]:
    os.makedirs(d, exist_ok=True)

# 获取所有图片文件名（假设图片和标签文件名一致，仅后缀不同）
img_files = [f for f in os.listdir(src_img_dir) if f.endswith('.jpg')]
random.shuffle(img_files)

# 划分比例：80% 训练集，20% 验证集
split_idx = int(len(img_files) * 0.8)
train_files = img_files[:split_idx]
val_files = img_files[split_idx:]

def move_files(file_list, src_img, src_lbl, dst_img, dst_lbl):
    for f in file_list:
        # 移动图片
        shutil.move(os.path.join(src_img, f), os.path.join(dst_img, f))
        # 移动对应的标签 (将 .jpg 替换为 .txt)
        lbl_f = f.replace('.jpg', '.txt')
        lbl_path = os.path.join(src_lbl, lbl_f)
        if os.path.exists(lbl_path):
            shutil.move(lbl_path, os.path.join(dst_lbl, lbl_f))
        else:
            print(f"警告: 未找到标签文件 {lbl_f}")

print(f"开始划分数据... 总计 {len(img_files)} 张图片")
move_files(train_files, src_img_dir, src_lbl_dir, train_img_dir, train_lbl_dir)
move_files(val_files, src_img_dir, src_lbl_dir, val_img_dir, val_lbl_dir)
print("划分完成！")