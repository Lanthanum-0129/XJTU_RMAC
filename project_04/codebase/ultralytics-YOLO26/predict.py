import sys
import argparse
import os
from pathlib import Path
from ultralytics import YOLO

PROJECT_ROOT = Path(__file__).resolve().parent

def main(opt):
    # 优先使用指定的权重，否则默认寻找训练生成的最佳权重
    if opt.weights:
        weights_path = Path(opt.weights)
    else:
        weights_path = PROJECT_ROOT.parent / "runs" / "train" / "rm_baseline" / "weights" / "best.pt"
        
    model = YOLO(str(weights_path))
    model.info()
    
    # 执行预测并保存结果
    model.predict(
        source=opt.source, 
        save=True, 
        imgsz=640, 
        conf=0.5, 
        project=str(PROJECT_ROOT.parent / "runs" / "predict"), 
        name="rm_predict"
    )

def parse_opt(known=False):
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default=None, help='weights path (e.g., best.pt)')
    parser.add_argument('--source', type=str, default=str(PROJECT_ROOT.parent / "data" / "val" / "images"), help='source directory or image path')
    opt = parser.parse_known_args()[0] if known else parser.parse_args()
    return opt

if __name__ == "__main__":  # 注意：确保前后有双下划线
    opt = parse_opt()
    main(opt)