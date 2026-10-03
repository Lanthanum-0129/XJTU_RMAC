from __future__ import annotations  # 注意：确保前后有双下划线
import argparse
from pathlib import Path
from ultralytics import YOLO

PROJECT_ROOT = Path(__file__).resolve().parent
# 将默认数据路径指向上一级目录的 data/data.yaml
DEFAULT_DATA = PROJECT_ROOT.parent.parent / "data" / "data.yaml"

def parse_opt() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train YOLO26")
    
    # --- 基础参数 ---
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA, help="dataset YAML path")
    parser.add_argument("--weights", type=Path, default=PROJECT_ROOT / "yolo26n.pt", help="pretrained weights")
    parser.add_argument("--cfg", type=Path, default=PROJECT_ROOT / "ultralytics/cfg/models/26/yolo26.yaml", help="model YAML used with --from-scratch")
    parser.add_argument("--from-scratch", action="store_true", help="initialize the model from --cfg")
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--device", default="0", help="CUDA device, e.g. 0, or cpu")
    parser.add_argument("--fraction", type=float, default=1.0, help="fraction of training images to use")
    parser.add_argument("--project", type=Path, default=PROJECT_ROOT.parent / "runs" / "train")
    parser.add_argument("--name", default="rm_baseline")
    parser.add_argument("--exist-ok", action="store_true")
    
    # --- 训练策略与调优参数 ---
    parser.add_argument("--patience", type=int, default=20, help="epochs to wait for no improvement (Early Stopping)")
    parser.add_argument("--cos_lr", action="store_true", help="use cosine learning rate scheduler")
    parser.add_argument("--amp", action="store_true", default=True, help="use Automatic Mixed Precision (AMP)")
    parser.add_argument("--optimizer", type=str, default="auto", help="optimizer (auto, SGD, Adam, AdamW, etc.)")
    parser.add_argument("--lr0", type=float, default=0.01, help="initial learning rate")
    parser.add_argument("--multi_scale", action="store_true", help="enable multi-scale training")
    
    # --- 数据增强参数 ---
    parser.add_argument("--hsv_h", type=float, default=0.015, help="image HSV-Hue augmentation")
    parser.add_argument("--hsv_s", type=float, default=0.7, help="image HSV-Saturation augmentation")
    parser.add_argument("--hsv_v", type=float, default=0.4, help="image HSV-Value augmentation")
    parser.add_argument("--degrees", type=float, default=0.0, help="image rotation (+/- deg)")
    parser.add_argument("--translate", type=float, default=0.1, help="image translation (+/- fraction)")
    parser.add_argument("--scale", type=float, default=0.5, help="image scale (+/- gain)")
    parser.add_argument("--fliplr", type=float, default=0.5, help="image flip left-right (probability)")
    parser.add_argument("--mosaic", type=float, default=1.0, help="image mosaic (probability)")
    parser.add_argument("--mixup", type=float, default=0.0, help="image mixup (probability)")
    parser.add_argument("--close_mosaic", type=int, default=10, help="disable mosaic augmentation for final N epochs")
    
    return parser.parse_args()

def main(opt: argparse.Namespace) -> None:
    model_source = opt.cfg if opt.from_scratch else opt.weights
    model = YOLO(str(model_source))
    model.info()
    
    # 将命令行解析到的参数全部传入 model.train()
    model.train(
        data=str(opt.data),
        epochs=opt.epochs,
        imgsz=opt.imgsz,
        workers=opt.workers,
        batch=opt.batch,
        device=opt.device,
        fraction=opt.fraction,
        val=True,          # 必须开启验证
        project=str(opt.project),
        name=opt.name,
        exist_ok=opt.exist_ok,
        # 传入调优与数据增强参数
        patience=opt.patience,
        cos_lr=opt.cos_lr,
        amp=opt.amp,
        optimizer=opt.optimizer, # 新增
        lr0=opt.lr0,             # 新增
        multi_scale=opt.multi_scale, # 新增
        hsv_h=opt.hsv_h,
        hsv_s=opt.hsv_s,
        hsv_v=opt.hsv_v,
        degrees=opt.degrees,
        translate=opt.translate,
        scale=opt.scale,
        fliplr=opt.fliplr,
        mosaic=opt.mosaic,
        mixup=opt.mixup,
        close_mosaic=opt.close_mosaic,
        plots=True,        # 生成训练曲线图
    )

if __name__ == "__main__":  # 注意：确保前后有双下划线
    main(parse_opt())