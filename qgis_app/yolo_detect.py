import argparse
import json
import math
import os
import tempfile
from pathlib import Path

import numpy as np
import rasterio


def _iter_windows(width, height, tile, overlap):
    step = max(1, tile - overlap)
    for y in range(0, height, step):
        for x in range(0, width, step):
            w = min(tile, width - x)
            h = min(tile, height - y)
            yield x, y, w, h


def _read_tile_rgb(src, x, y, w, h):
    window = rasterio.windows.Window(x, y, w, h)
    if src.count >= 3:
        data = src.read([1, 2, 3], window=window)
    elif src.count == 1:
        single = src.read(1, window=window)
        data = np.stack([single, single, single], axis=0)
    elif src.count == 2:
        data = src.read([1, 2], window=window)
    else:
        data = src.read(window=window)

    data = np.moveaxis(data, 0, -1)  # CHW -> HWC
    if data.shape[2] == 2:
        # Pad to 3 channels for YOLO
        third = data[:, :, 0:1]
        data = np.concatenate([data, third], axis=2)
    elif data.shape[2] > 3:
        data = data[:, :, :3]
    if data.dtype != np.uint8:
        out = np.zeros_like(data, dtype=np.uint8)
        for i in range(min(3, data.shape[2])):
            band = data[:, :, i].astype(np.float32)
            lo, hi = np.percentile(band, (2, 98))
            if hi <= lo:
                hi = lo + 1.0
            band = np.clip((band - lo) * 255.0 / (hi - lo), 0, 255)
            out[:, :, i] = band.astype(np.uint8)
        data = out
    return data


def _init_ultralytics_detector(model_path):
    try:
        from ultralytics import YOLO
    except Exception as exc:
        raise RuntimeError(
            "Ultralytics not available. Install it in the mmyolo env: pip install ultralytics"
        ) from exc

    model_path = str(model_path)
    is_onnx = model_path.lower().endswith(".onnx")
    if is_onnx:
        import onnxruntime as ort

        session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        input_name = session.get_inputs()[0].name
        input_shape = session.get_inputs()[0].shape
        in_h, in_w = int(input_shape[2]), int(input_shape[3])
        return {
            "type": "onnx",
            "session": session,
            "input_name": input_name,
            "in_h": in_h,
            "in_w": in_w,
        }

    model = YOLO(model_path)
    return {"type": "ultra", "model": model}


def _detect_ultralytics(image, detector, conf, iou):
    if detector["type"] == "onnx":
        return _detect_onnx(image, detector, conf, iou)

    model = detector["model"]
    results = model.predict(image, conf=conf, iou=iou, verbose=False)
    dets = []
    for r in results:
        if r.boxes is None:
            continue
        for box in r.boxes:
            xyxy = box.xyxy[0].cpu().numpy().tolist()
            score = float(box.conf[0].cpu().numpy())
            dets.append((xyxy, score))
    return dets


def _detect_onnx(image, detector, conf, iou):
    import cv2

    h, w = image.shape[:2]
    session = detector["session"]
    input_name = detector["input_name"]
    in_h = detector["in_h"]
    in_w = detector["in_w"]

    resized = cv2.resize(image, (in_w, in_h), interpolation=cv2.INTER_LINEAR)
    inp = resized.astype(np.float32) / 255.0
    inp = np.transpose(inp, (2, 0, 1))[None, ...]

    outputs = session.run(None, {input_name: inp})
    out = outputs[0]
    out = out[0].T  # (8400, 5)
    boxes = out[:, :4]
    scores = out[:, 4]

    keep = scores >= conf
    boxes = boxes[keep]
    scores = scores[keep]
    if boxes.size == 0:
        return []

    # xywh -> xyxy
    xyxy = np.zeros_like(boxes)
    xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2
    xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2
    xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2
    xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2

    # scale back to tile size
    sx = w / float(in_w)
    sy = h / float(in_h)
    xyxy[:, [0, 2]] *= sx
    xyxy[:, [1, 3]] *= sy

    keep_idx = _nms(xyxy, scores, iou)
    return [(xyxy[i].tolist(), float(scores[i])) for i in keep_idx]


def _nms(boxes, scores, iou_thr):
    if len(boxes) == 0:
        return []
    idxs = scores.argsort()[::-1]
    keep = []
    while idxs.size > 0:
        i = idxs[0]
        keep.append(i)
        if idxs.size == 1:
            break
        ious = _iou(boxes[i], boxes[idxs[1:]])
        idxs = idxs[1:][ious <= iou_thr]
    return keep


def _iou(box, boxes):
    x1 = np.maximum(box[0], boxes[:, 0])
    y1 = np.maximum(box[1], boxes[:, 1])
    x2 = np.minimum(box[2], boxes[:, 2])
    y2 = np.minimum(box[3], boxes[:, 3])
    inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    area1 = (box[2] - box[0]) * (box[3] - box[1])
    area2 = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    union = area1 + area2 - inter + 1e-6
    return inter / union


def _detect_mmdet(image_path, model_path, config_path, conf):
    try:
        from mmdet.apis import DetInferencer
    except Exception as exc:
        raise RuntimeError(
            "MMDet/DetInferencer not available in env. Ensure mmyolo is installed."
        ) from exc
    inferencer = DetInferencer(model=config_path, weights=model_path)
    res = inferencer(image_path, pred_score_thr=conf)
    dets = []
    if not res or "predictions" not in res:
        return dets
    preds = res["predictions"][0]
    for box, score in zip(preds["bboxes"], preds["scores"]):
        xyxy = [float(box[0]), float(box[1]), float(box[2]), float(box[3])]
        dets.append((xyxy, float(score)))
    return dets


def _dedupe_points(points, radius=6):
    if not points:
        return []
    r2 = radius * radius
    grid = {}
    kept = []
    for x, y, score in sorted(points, key=lambda p: p[2], reverse=True):
        gx = int(x // radius)
        gy = int(y // radius)
        hit = False
        for ix in (gx - 1, gx, gx + 1):
            for iy in (gy - 1, gy, gy + 1):
                for (px, py, _) in grid.get((ix, iy), []):
                    if (x - px) ** 2 + (y - py) ** 2 <= r2:
                        hit = True
                        break
                if hit:
                    break
            if hit:
                break
        if not hit:
            kept.append((x, y, score))
            grid.setdefault((gx, gy), []).append((x, y, score))
    return kept


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--model", required=True)
    ap.add_argument("--config", default="")
    ap.add_argument("--tile", type=int, default=1024)
    ap.add_argument("--overlap", type=int, default=128)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--edge-buffer", type=int, default=10)
    ap.add_argument("--output", required=True)
    args = ap.parse_args()

    image_path = Path(args.image)
    model_path = Path(args.model)
    config_path = Path(args.config) if args.config else None
    print("STATUS Initializing detector...", flush=True)
    detector = None
    if not (config_path and config_path.exists()):
        detector = _init_ultralytics_detector(str(model_path))

    detections_global = []
    with rasterio.open(image_path) as src:
        img_w, img_h = src.width, src.height
        windows = list(_iter_windows(src.width, src.height, args.tile, args.overlap))
        total = len(windows)
        for idx, (x, y, w, h) in enumerate(windows, start=1):
            print(f"PROGRESS {idx}/{total}", flush=True)
            window = rasterio.windows.Window(x, y, w, h)
            # Valid-data mask: 0 = nodata, >0 = valid
            valid_mask = src.dataset_mask(window=window)
            # Avoid duplicate detections from overlapping tiles by keeping only
            # centers inside the inner tile region.
            margin = max(0, args.overlap // 2)
            inner_x1 = x + margin
            inner_y1 = y + margin
            inner_x2 = x + w - margin
            inner_y2 = y + h - margin
            at_left = x == 0
            at_top = y == 0
            at_right = x + w >= img_w
            at_bottom = y + h >= img_h
            if config_path and config_path.exists():
                with tempfile.TemporaryDirectory() as tmpdir:
                    tile_path = Path(tmpdir) / "tile.tif"
                    rgb = _read_tile_rgb(src, x, y, w, h)
                    profile = src.profile.copy()
                    profile.update(
                        {
                            "height": h,
                            "width": w,
                            "transform": src.window_transform(window),
                            "count": 3,
                            "dtype": "uint8",
                        }
                    )
                    with rasterio.open(tile_path, "w", **profile) as dst:
                        dst.write(np.moveaxis(rgb, -1, 0))
                    detections = _detect_mmdet(
                        str(tile_path), str(model_path), str(config_path), args.conf
                    )
            else:
                rgb = _read_tile_rgb(src, x, y, w, h)
                detections = _detect_ultralytics(
                    rgb, detector, args.conf, args.iou
                )

                # Map detections to global coords
                for (x1, y1, x2, y2), score in detections:
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0
                    # Keep only detections whose center lies on valid raster data.
                    cxi = int(round(cx))
                    cyi = int(round(cy))
                    if cxi < 0 or cyi < 0 or cxi >= w or cyi >= h:
                        continue
                    if valid_mask[cyi, cxi] == 0:
                        continue
                    # Reject detections too close to nodata edge.
                    eb = max(0, int(args.edge_buffer))
                    if eb > 0:
                        x0 = max(0, cxi - eb)
                        y0 = max(0, cyi - eb)
                        x1 = min(w, cxi + eb + 1)
                        y1 = min(h, cyi + eb + 1)
                        if valid_mask[y0:y1, x0:x1].min() == 0:
                            continue
                    gx = x + cx
                    gy = y + cy
                    # Keep only centers inside inner region (or on image borders)
                    if not (
                        (inner_x1 <= gx <= inner_x2 and inner_y1 <= gy <= inner_y2)
                        or at_left
                        or at_top
                        or at_right
                        or at_bottom
                    ):
                        continue
                    detections_global.append((gx, gy, score))

        # Dedupe points that are too close (from overlaps)
        deduped = _dedupe_points(detections_global, radius=max(3, args.overlap // 4))
        features = []
        for gx, gy, score in deduped:
            x, y = src.transform * (gx, gy)
            features.append(
                {
                    "type": "Feature",
                    "geometry": {"type": "Point", "coordinates": [float(x), float(y)]},
                    "properties": {"score": float(score)},
                }
            )

    geojson = {"type": "FeatureCollection", "features": features}
    Path(args.output).write_text(json.dumps(geojson), encoding="utf-8")


if __name__ == "__main__":
    main()
