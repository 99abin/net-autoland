"""Blue/purple quadrilateral segmentation. No learned weights required."""

from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class DetectionConfig:
    hsv_lower: tuple = (100, 90, 45)
    hsv_upper: tuple = (160, 255, 255)
    min_area_px: float = 600.0
    min_fill: float = 0.70
    min_aspect: float = 0.8
    max_aspect: float = 6.0
    max_image_fraction: float = 0.90


def order_corners(points):
    """Image top-left, top-right, bottom-right, bottom-left; moderate camera roll."""
    p = np.asarray(points, dtype=np.float32).reshape(4, 2)
    center = p.mean(axis=0)
    p = p[np.argsort(np.arctan2(p[:, 1] - center[1], p[:, 0] - center[0]))]
    p = np.roll(p, -int(np.argmin(p.sum(axis=1))), axis=0)
    a, b = p[1] - p[0], p[2] - p[1]
    if a[0] * b[1] - a[1] * b[0] < 0:
        p = p[[0, 3, 2, 1]]
    return p


def detect_rectangle(bgr, c=DetectionConfig()):
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("Expected BGR image")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.asarray(c.hsv_lower, np.uint8), np.asarray(c.hsv_upper, np.uint8))
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_rank = -1
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < c.min_area_px or area > bgr.shape[0] * bgr.shape[1] * c.max_image_fraction:
            continue
        hull = cv2.convexHull(contour)
        quad = cv2.approxPolyDP(hull, 0.025 * cv2.arcLength(hull, True), True)
        if len(quad) != 4 or not cv2.isContourConvex(quad):
            continue
        quad = order_corners(quad)
        width = (np.linalg.norm(quad[1] - quad[0]) + np.linalg.norm(quad[2] - quad[3])) / 2
        height = (np.linalg.norm(quad[3] - quad[0]) + np.linalg.norm(quad[2] - quad[1])) / 2
        if height < 2 or not c.min_aspect <= width / height <= c.max_aspect:
            continue
        polygon = np.zeros_like(mask)
        cv2.fillConvexPoly(polygon, quad.astype(np.int32), 255)
        fill = np.count_nonzero(cv2.bitwise_and(mask, polygon)) / max(np.count_nonzero(polygon), 1)
        if fill < c.min_fill:
            continue
        # Geometric segmentation score, NOT a calibrated detection probability.
        score = float(min(fill, area / max(cv2.contourArea(quad), 1)))
        rank = area * score
        if rank > best_rank:
            best = (quad, score)
            best_rank = rank
    return best, mask
