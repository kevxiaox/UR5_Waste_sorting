import os
import glob
import json
import cv2
from ultralytics import YOLO

# 1. 加载专属最优模型
MODEL_PATH = r"D:\Waste_Vision_Dev\src\best.pt"
model = YOLO(MODEL_PATH)

# 2. 垃圾分类映射字典
CATEGORY_MAP = {
    "plastic_bottle": "Recyclable",
    "aluminum_can": "Recyclable",
    "paper_cup": "Recyclable",
    "sponge": "General_Waste"
}

def detect_waste(image_bgr, conf_threshold=0.45):
    """
    核心视觉接口函数（供成员 A 调用）
    输入: 原始单眼 OpenCV BGR 图像帧, 置信度阈值(默认 0.45)
    输出: (标准 JSON 字符串, 绘制了检测框与中心点的原始分辨率 BGR 图像)
    """
    annotated_img = image_bgr.copy()
    orig_h, orig_w = image_bgr.shape[:2]
    
    # 【关键修复】将输入图像临时拉伸为与 Roboflow 训练集一致的 512x512 正方形
    input_resized = cv2.resize(image_bgr, (512, 512))
    scale_x = orig_w / 512.0
    scale_y = orig_h / 512.0
    
    # 强制使用 CPU 推理
    results = model.predict(source=input_resized, device='cpu', verbose=False)[0]
    
    detections = []
    
    for box in results.boxes:
        conf = float(box.conf[0])
        cls_id = int(box.cls[0])
        class_name = model.names[cls_id]
        
        # 将 512x512 下的边界框坐标精准映射回原始单眼高清分辨率
        rx1, ry1, rx2, ry2 = box.xyxy[0]
        x1 = int(rx1 * scale_x)
        y1 = int(ry1 * scale_y)
        x2 = int(rx2 * scale_x)
        y2 = int(ry2 * scale_y)
        
        # 计算原始分辨率下的中心点像素坐标 (u, v)
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        
        # 置信度过滤与 UNKNOWN 拒识机制
        if conf >= conf_threshold and class_name in CATEGORY_MAP:
            waste_category = CATEGORY_MAP[class_name]
            status = "OK"
            color = (0, 255, 0) # 正常识别显示绿框
        else:
            class_name = "UNKNOWN"
            waste_category = "UNKNOWN"
            status = "REJECTED_LOW_CONF"
            color = (0, 165, 255) # 拒识目标显示橙框
            
        detections.append({
            "class_name": class_name,
            "waste_category": waste_category,
            "confidence": round(conf, 3),
            "bbox_xyxy": [x1, y1, x2, y2],
            "center_pixel": [cx, cy],
            "status": status
        })
        
        # 在原始分辨率画面上绘制矩形框、中心红点及坐标文字
        cv2.rectangle(annotated_img, (x1, y1), (x2, y2), color, 2)
        cv2.circle(annotated_img, (cx, cy), 6, (0, 0, 255), -1)
        label = f"{class_name} ({conf:.2f}) [{cx},{cy}]"
        cv2.putText(annotated_img, label, (x1, max(y1 - 10, 25)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2)
        
    output_payload = {
        "image_resolution": [orig_w, orig_h],
        "total_detected": len(detections),
        "items": detections
    }
    
    return json.dumps(output_payload, indent=2, ensure_ascii=False), annotated_img


# ================= 原始单眼分辨率最终验证 =================
if __name__ == "__main__":
    raw_dir = r"D:\Waste_Vision_Dev\dataset_single_eye"
    output_dir = r"D:\Waste_Vision_Dev\src\raw_test_results"
    os.makedirs(output_dir, exist_ok=True)
    
    raw_images = glob.glob(os.path.join(raw_dir, "*.*"))[:5]
    print(f"[INFO] 正在重新测试 {len(raw_images)} 张原始单眼分辨率图片...\n")
    
    for img_path in raw_images:
        img_name = os.path.basename(img_path)
        frame = cv2.imread(img_path)
        h, w, _ = frame.shape
        
        json_result, vis_img = detect_waste(frame, conf_threshold=0.45)
        
        print(f"--- 原始图片: {img_name} (真实分辨率: 宽 {w} x 高 {h}) ---")
        print(json_result)
        
        save_path = os.path.join(output_dir, f"raw_result_{img_name}")
        cv2.imwrite(save_path, vis_img)
        
    print(f"\n[INFO] 修复验证完毕！请查看 {output_dir} 下的结果图。")