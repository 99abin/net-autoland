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
    border_margin_px: int = 2


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


def detect_center(bgr, c=DetectionConfig()):
    """Colour-region centre heuristic; no four-corner fit or pose inference.

    A visible fragment/occlusion can bias this centroid. Image-edge clipping is
    rejected, but interior occlusion and false colour associations remain limits.
    """
    if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
        raise ValueError("Expected BGR image")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.asarray(c.hsv_lower, np.uint8), np.asarray(c.hsv_upper, np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, rank = None, -1.
    height, width = bgr.shape[:2]
    for contour in contours:
        area = cv2.contourArea(contour)
        if not c.min_area_px <= area <= width * height * c.max_image_fraction:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        border = c.border_margin_px
        if x <= border or y <= border or x + w >= width - border or y + h >= height - border:
            continue
        (_, _), (a, b), _ = cv2.minAreaRect(contour)
        if min(a, b) < 2:
            continue
        aspect = max(a, b) / min(a, b)
        fill = area / max(a * b, 1.)
        solidity = area / max(cv2.contourArea(cv2.convexHull(contour)), 1.)
        if not c.min_aspect <= aspect <= c.max_aspect or fill < c.min_fill:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] <= 0:
            continue
        score = float(min(fill, solidity, 1.))
        if area * score > rank:
            center = np.array([moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]])
            best, rank = (center, score, float(area)), area * score
    return best, mask
