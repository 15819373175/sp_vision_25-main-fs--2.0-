"""
Export new_yolo26/weights/best.pt -> ONNX -> OpenVINO IR
Usage:
    pip install ultralytics
    python export_yolo26.py
"""
import shutil
from pathlib import Path
from ultralytics import YOLO

PT_PATH   = "new_yolo26/weights/best.pt"
ASSETS    = Path("assets")
ONNX_NAME = "yolo26.onnx"

# 1. Export to ONNX (opset 12, static shape, no NMS)
model = YOLO(PT_PATH)
model.export(format="onnx", imgsz=640, opset=12, simplify=True, dynamic=False)

# The exported file lands next to the .pt
onnx_src = Path(PT_PATH).with_suffix(".onnx")
onnx_dst = ASSETS / ONNX_NAME
shutil.move(str(onnx_src), str(onnx_dst))
print(f"ONNX saved -> {onnx_dst}")

# 2. Convert ONNX -> OpenVINO IR via mo (OpenVINO Model Optimizer)
import subprocess, sys

mo_cmd = [
    sys.executable, "-m", "mo",
    "--input_model", str(onnx_dst),
    "--output_dir",  str(ASSETS),
    "--model_name",  "yolo26",
    "--input_shape", "[1,3,640,640]",
]
print("Running:", " ".join(mo_cmd))
result = subprocess.run(mo_cmd, capture_output=False)

if result.returncode != 0:
    # Fallback: try openvino.tools.mo directly
    try:
        from openvino.tools.mo import convert_model
        from openvino import save_model
        import openvino as ov

        ov_model = convert_model(str(onnx_dst), input_shape=[1, 3, 640, 640])
        save_model(ov_model, str(ASSETS / "yolo26.xml"))
        print("OpenVINO IR saved via Python API -> assets/yolo26.xml + assets/yolo26.bin")
    except Exception as e:
        print(f"[WARN] OpenVINO conversion failed: {e}")
        print("You can still use the ONNX file directly (change model path to assets/yolo26.onnx)")
else:
    print("OpenVINO IR saved -> assets/yolo26.xml + assets/yolo26.bin")
