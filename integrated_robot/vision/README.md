# Vision

Code from the project's vision team (YOLO training and ZED integration), reduced to what
`run_sorting.py` uses.

| File | Content |
|---|---|
| `detector.py` | `detect_waste(image_bgr, conf_threshold=0.45) -> (json_str, annotated_bgr)`; image stretched to 512 x 512 as in training, boxes mapped back to the original pixels, CPU inference |
| `weights/best.pt` | trained YOLO weights: `plastic_bottle`, `aluminum_can`, `paper_cup`, `sponge` |
| `waste_sorting/camera.py` | `ZedSource`: HD720 left image + XYZ point cloud from the same grab |
| `waste_sorting/zones.py` | workspace polygons (`configs/workspace_zones.json`), overlay, zone lookup |
| `waste_sorting/localization.py` | depth sampling at a pixel with validity checks |

Detector output:

```json
{"image_resolution": [1280, 720], "total_detected": 1,
 "items": [{"class_name": "sponge", "waste_category": "General_Waste", "confidence": 0.82,
            "bbox_xyxy": [810, 165, 890, 235], "center_pixel": [850, 200], "status": "OK"}]}
```

The detector's 0.45 threshold only labels boxes; the sorting decision uses the policy threshold
(0.60, `configs/sorting_policy.json`). A box centre is a visible surface point, not a grasp point.

Changes from the original files: weights path (`weights/best.pt`), Ultralytics settings folder
(`vision/.ultralytics`), and ZED DLL folders found from the SDK installation (`ZED_SDK_ROOT_DIR`,
override with `ZED_DLL_DIRS`) instead of fixed paths.
