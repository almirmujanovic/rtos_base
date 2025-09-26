from __future__ import annotations
import numpy as np
import threading
from PySide6.QtCore import QObject, Signal
from config import YoloConfig

try:
    from ultralytics import YOLO
except Exception:
    YOLO = None

class YoloWorker(QObject):
    annotatedReady = Signal(object, object)  # (annotated_frame_bgr, dets)
    modelReady = Signal(bool)

    def __init__(self, cfg: YoloConfig):
        super().__init__()
        self.cfg = cfg
        self.model = None
        self._busy = False   
        if YOLO is not None:
            try:
                self.model = YOLO(cfg.model_path)
                self.modelReady.emit(True)
            except Exception:
                self.model = None
                self.modelReady.emit(False)
        else:
            self.modelReady.emit(False)

    def infer_async(self, frame_bgr):
        if self._busy:
            return
        self._busy = True
        threading.Thread(target=self._infer_worker, args=(frame_bgr,), daemon=True).start()

    def infer(self, frame_bgr):
        if self.model is None:
            self.annotatedReady.emit(frame_bgr, [])
            return
        res = self.model.predict(
            source=frame_bgr, imgsz=self.cfg.imgsz, conf=self.cfg.conf, iou=self.cfg.iou,
            device=self.cfg.device, verbose=False
        )[0]
        dets = []
        if res.boxes is not None:
            for b in res.boxes:
                cls_id = int(b.cls[0].item()); conf = float(b.conf[0].item())
                x1,y1,x2,y2 = map(float, b.xyxy[0].tolist())
                dets.append((res.names.get(cls_id, str(cls_id)), conf, x1, y1, x2, y2))
        annotated = res.plot()
        self.annotatedReady.emit(annotated, dets)

    # Worker for the async path
    def _infer_worker(self, frame_bgr):
        try:
            self.infer(frame_bgr)
        finally:
            self._busy = False
