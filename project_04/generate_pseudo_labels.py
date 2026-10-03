import os
from pathlib import Path
from ultralytics import YOLO

print("=== 1. 开始执行伪标签生成脚本 ===")

# 1. 定义路径
best_model_path = "/home/inkspring/RoboMaster/XJTU_RMAC/project_04/codebase/runs/train/rm_early_close_mosaic/weights/best.pt"
unlabeled_img_dir = "/home/inkspring/RoboMaster/XJTU_RMAC/project_04/data/unlabeled/images"
output_project_dir = "/home/inkspring/RoboMaster/XJTU_RMAC/project_04/runs/detect"

# 2. 检查模型文件是否存在
print(f"正在检查模型路径: {best_model_path}")
if not os.path.exists(best_model_path):
    print(f"❌ 错误：找不到模型文件！请检查路径是否正确。")
    exit()
print("✅ 模型文件存在。")

# 3. 检查未标注图片目录是否存在
print(f"正在检查图片目录: {unlabeled_img_dir}")
if not os.path.exists(unlabeled_img_dir):
    print(f"❌ 错误：找不到图片目录！请检查路径是否正确。")
    exit()

# 统计图片数量
img_files = [f for f in os.listdir(unlabeled_img_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
print(f"✅ 图片目录存在，共找到 {len(img_files)} 张图片。")

if len(img_files) == 0:
    print("⚠️ 警告：目录下没有找到图片文件，脚本将退出。")
    exit()

# 4. 加载模型
print("正在加载模型，请稍候...")
model = YOLO(best_model_path)
print("✅ 模型加载成功。")

# 5. 执行推理并生成伪标签
print("开始推理并生成伪标签 (conf=0.6, save_txt=True)...")
results = model.predict(
    source=unlabeled_img_dir,
    save_txt=True,          # 保存为 YOLO 格式的 .txt 文件
    save_conf=True,         # 在 txt 文件中保留置信度数值
    conf=0.6,               # 仅保留置信度 >= 0.6 的预测框
    project=output_project_dir,
    name="pseudo_gen",
    exist_ok=True,
    verbose=True            # 强制在终端显示进度条
)

print("=== 🎉 伪标签生成完毕！ ===")
print(f"生成的标签文件保存在: {output_project_dir}/pseudo_gen/labels/")