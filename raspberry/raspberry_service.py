"""Сервисные команды Raspberry Pi для Wi‑Fi AP, SSH и телеметрии платы."""

from __future__ import annotations

from collections import Counter, deque
import json
import logging
import os
import platform
import re
import signal
import string
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Iterable, Literal

try:
    import cv2
except ImportError:  # pragma: no cover - vision-зависимость опциональна до установки
    cv2 = None  # type: ignore[assignment]

try:
    import numpy as np
except ImportError:  # pragma: no cover - vision-зависимость опциональна до установки
    np = None  # type: ignore[assignment]


LOGGER = logging.getLogger(__name__)


class RaspberryServiceError(RuntimeError):
    """Базовое исключение для ошибок сервисов на стороне Raspberry Pi."""


class RaspberryCommandError(RaspberryServiceError):
    """Выбрасывается, когда системная команда недоступна или завершилась ошибкой."""


@dataclass(slots=True)
class ProcessInfo:
    pid: int
    ppid: int
    command: str
    args: str


@dataclass(slots=True)
class WifiAccessPointInfo:
    channel: int
    signal: int


@dataclass(slots=True)
class VictimDetectionResult:
    found: bool
    victim_type: Literal["none", "letter", "color"]
    letter: Literal["H", "S", "U"] | None = None
    color: Literal["black", "red", "yellow", "green"] | None = None
    confidence: float = 0.0
    bbox: tuple[int, int, int, int] | None = None
    source: str = "uninitialized"

    @classmethod
    def none(cls, *, source: str = "unknown") -> "VictimDetectionResult":
        return cls(found=False, victim_type="none", confidence=0.0, bbox=None, source=source)

    def signature(self) -> tuple[str, str | None, str | None] | None:
        if not self.found:
            return None
        return (self.victim_type, self.letter, self.color)


@dataclass(slots=True)
class _CardCandidate:
    roi: Any
    bbox: tuple[int, int, int, int]
    area: float


class VictimCamera:
    """Детектирует буквенные и цветовые жертвы по кадрам USB-камеры."""

    LETTER_LABELS: tuple[Literal["H", "S", "U"], ...] = ("H", "S", "U")
    COLOR_LABELS: tuple[Literal["red", "yellow", "green"], ...] = ("red", "yellow", "green")
    _ALL_COLOR_LABELS = ("black", "red", "yellow", "green")
    _TEMPLATE_SIZE = 96
    _DEFAULT_MIN_CONFIDENCE = 0.74
    _LETTER_MARGIN_THRESHOLD = 0.04
    _COLOR_ASPECT_MIN = 0.45
    _COLOR_ASPECT_MAX = 2.2

    def __init__(
        self,
        camera_index: int = 0,
        *,
        min_confidence: float = _DEFAULT_MIN_CONFIDENCE,
        stability_frames: int = 3,
        frame_provider: Callable[[], Any] | None = None,
        logger: logging.Logger | None = None,
    ) -> None:
        if not 0.0 <= min_confidence <= 1.0:
            raise ValueError("min_confidence должен быть в диапазоне от 0.0 до 1.0")
        if stability_frames <= 0:
            raise ValueError("stability_frames должен быть больше нуля")

        self._camera_index = camera_index
        self._min_confidence = float(min_confidence)
        self._frame_provider = frame_provider
        self._logger = logger or LOGGER
        self._capture: Any | None = None
        self._history: deque[VictimDetectionResult] = deque(maxlen=stability_frames)
        self._last_result = VictimDetectionResult.none(source="uninitialized")
        self._reference_vectors = self._build_reference_vectors()

    def __enter__(self) -> "VictimCamera":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
            self._capture = None

    def capture_frame(self) -> Any:
        self._ensure_vision_dependencies()
        if self._frame_provider is not None:
            return self._normalize_frame(self._frame_provider())

        capture = self._get_capture()
        ok, frame = capture.read()
        if not ok or frame is None:
            raise RaspberryCommandError(f"Не удалось прочитать кадр с USB-камеры #{self._camera_index}.")
        return self._normalize_frame(frame)

    def analyze_frame(self, frame: Any, *, source: str = "frame") -> VictimDetectionResult:
        normalized_frame = self._normalize_frame(frame)
        result = self._analyze_frame_internal(normalized_frame, source=source)
        self._last_result = result
        return result

    def analyze_file(self, path: str | Path) -> VictimDetectionResult:
        self._ensure_vision_dependencies()
        image_path = Path(path)
        frame = cv2.imread(str(image_path))
        if frame is None:
            raise FileNotFoundError(f"Не удалось открыть изображение: {image_path}")
        result = self._analyze_frame_internal(self._normalize_frame(frame), source=str(image_path))
        self._last_result = result
        return result

    def detect_victim(self) -> VictimDetectionResult:
        frame = self.capture_frame()
        raw_result = self._analyze_frame_internal(frame, source=f"camera:{self._camera_index}")
        stabilized_result = self._stabilize_result(raw_result)
        self._last_result = stabilized_result
        return stabilized_result

    def has_letter(self, letter: Literal["H", "S", "U"] | str | None = None) -> bool:
        result = self._ensure_last_result()
        if letter is None:
            return result.victim_type == "letter" and result.letter is not None
        normalized_letter = self._normalize_letter(letter)
        return result.letter == normalized_letter

    def get_letter(self) -> Literal["H", "S", "U"] | None:
        return self._ensure_last_result().letter

    def has_color(self, color: Literal["black", "red", "yellow", "green"] | str | None = None) -> bool:
        result = self._ensure_last_result()
        if color is None:
            return result.victim_type == "color" and result.color in self.COLOR_LABELS

        normalized_color = self._normalize_color(color)
        return result.color == normalized_color

    def get_color(self) -> Literal["black", "red", "yellow", "green"] | None:
        return self._ensure_last_result().color

    def render_debug_frame(
        self,
        frame: Any,
        result: VictimDetectionResult | None = None,
    ) -> Any:
        self._ensure_vision_dependencies()
        debug_frame = self._normalize_frame(frame).copy()
        result = result or self._last_result

        label = "none"
        color = (80, 80, 80)
        if result.found:
            if result.victim_type == "letter" and result.letter is not None:
                label = f"letter:{result.letter}"
                color = (40, 40, 40)
            elif result.color is not None:
                label = f"color:{result.color}"
                color = {
                    "red": (0, 0, 255),
                    "yellow": (0, 255, 255),
                    "green": (0, 180, 0),
                    "black": (40, 40, 40),
                }.get(result.color, (255, 255, 255))

        if result.bbox is not None:
            x, y, w, h = result.bbox
            cv2.rectangle(debug_frame, (x, y), (x + w, y + h), color, 2)

        cv2.putText(
            debug_frame,
            f"{label} ({result.confidence:.2f})",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )
        return debug_frame

    def _ensure_last_result(self) -> VictimDetectionResult:
        if self._last_result.source == "uninitialized":
            return self.detect_victim()
        return self._last_result

    def _stabilize_result(self, result: VictimDetectionResult) -> VictimDetectionResult:
        self._history.append(result)
        if self._history.maxlen == 1:
            return result

        non_empty_results = [item for item in self._history if item.found]
        if not non_empty_results:
            return VictimDetectionResult.none(source=result.source)

        signatures = [item.signature() for item in non_empty_results if item.signature() is not None]
        if not signatures:
            return VictimDetectionResult.none(source=result.source)

        best_signature, count = Counter(signatures).most_common(1)[0]
        required_votes = max(2, (self._history.maxlen // 2) + 1)
        if count < required_votes:
            return VictimDetectionResult.none(source=result.source)

        matched_results = [item for item in non_empty_results if item.signature() == best_signature]
        best_result = max(matched_results, key=lambda item: item.confidence)
        average_confidence = sum(item.confidence for item in matched_results) / len(matched_results)
        return VictimDetectionResult(
            found=True,
            victim_type=best_result.victim_type,
            letter=best_result.letter,
            color=best_result.color,
            confidence=round(average_confidence, 4),
            bbox=best_result.bbox,
            source=best_result.source,
        )

    def _analyze_frame_internal(self, frame: Any, *, source: str) -> VictimDetectionResult:
        letter_result = self._detect_letter(frame, source=source)
        color_result = self._detect_color(frame, source=source)

        candidates = [candidate for candidate in (letter_result, color_result) if candidate.found]
        if not candidates:
            return VictimDetectionResult.none(source=source)

        best_result = max(candidates, key=lambda item: item.confidence)
        if best_result.confidence < self._min_confidence:
            return VictimDetectionResult.none(source=source)
        return best_result

    def _detect_letter(self, frame: Any, *, source: str) -> VictimDetectionResult:
        card_candidates = self._extract_card_candidates(frame)
        best_card_result = VictimDetectionResult.none(source=source)
        for candidate in card_candidates:
            letter_result = self._detect_letter_from_card(candidate.roi, bbox=candidate.bbox, source=source)
            if letter_result.confidence > best_card_result.confidence:
                best_card_result = letter_result
        if best_card_result.found:
            return best_card_result

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        binary = cv2.adaptiveThreshold(
            blurred,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            31,
            9,
        )
        kernel = np.ones((3, 3), dtype=np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_area = frame.shape[0] * frame.shape[1]

        best_result = VictimDetectionResult.none(source=source)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < max(220.0, frame_area * 0.003) or area > frame_area * 0.8:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = w / float(max(h, 1))
            fill_ratio = area / float(max(w * h, 1))
            contrast = float(gray[y : y + h, x : x + w].std())
            if not 0.25 <= aspect_ratio <= 2.6:
                continue
            if not 0.08 <= fill_ratio <= 0.85:
                continue
            if contrast < 20.0:
                continue

            warped = self._warp_rotated_rect(binary, cv2.minAreaRect(contour))
            if warped is None:
                continue

            prepared = self._prepare_letter_roi(warped)
            if prepared is None:
                continue

            ranked_labels = self._classify_letter_roi(prepared)
            if not ranked_labels:
                continue
            label, similarity, oriented_candidate = ranked_labels[0]
            if label not in self.LETTER_LABELS:
                continue
            if not self._passes_letter_rules(oriented_candidate, label):
                continue
            confidence = self._normalize_letter_confidence(similarity)
            second_target_score = max(
                (
                    candidate_score
                    for candidate_label, candidate_score, _ in ranked_labels
                    if candidate_label in self.LETTER_LABELS and candidate_label != label
                ),
                default=0.0,
            )
            margin = similarity - second_target_score
            if confidence < self._min_confidence or margin < self._LETTER_MARGIN_THRESHOLD:
                continue

            confidence = round(confidence, 4)
            if confidence > best_result.confidence:
                best_result = VictimDetectionResult(
                    found=True,
                    victim_type="letter",
                    letter=label,
                    color="black",
                    confidence=confidence,
                    bbox=(x, y, w, h),
                    source=source,
                )
        return best_result

    def _detect_color(self, frame: Any, *, source: str) -> VictimDetectionResult:
        card_candidates = self._extract_card_candidates(frame)
        best_card_result = VictimDetectionResult.none(source=source)
        for candidate in card_candidates:
            color_result = self._detect_color_from_card(candidate.roi, bbox=candidate.bbox, source=source)
            if color_result.confidence > best_card_result.confidence:
                best_card_result = color_result
        if best_card_result.found:
            return best_card_result
        if card_candidates:
            return VictimDetectionResult.none(source=source)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), dtype=np.uint8)
        frame_area = frame.shape[0] * frame.shape[1]

        best_result = VictimDetectionResult.none(source=source)
        for color_name, ranges in self._color_ranges().items():
            mask = np.zeros(frame.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < max(240.0, frame_area * 0.004) or area > frame_area * 0.75:
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = w / float(max(h, 1))
                fill_ratio = area / float(max(w * h, 1))
                if not self._COLOR_ASPECT_MIN <= aspect_ratio <= self._COLOR_ASPECT_MAX:
                    continue
                if fill_ratio < 0.45:
                    continue

                roi_mask = mask[y : y + h, x : x + w]
                roi_hsv = hsv[y : y + h, x : x + w]
                colored_pixels = roi_mask > 0
                if not np.any(colored_pixels):
                    continue

                purity = float(np.count_nonzero(colored_pixels)) / float(max(w * h, 1))
                saturation = float(roi_hsv[..., 1][colored_pixels].mean())
                value = float(roi_hsv[..., 2][colored_pixels].mean())
                hull = cv2.convexHull(contour)
                hull_area = float(max(cv2.contourArea(hull), 1.0))
                solidity = area / hull_area
                area_score = min(1.0, area / max(frame_area * 0.06, 1.0))
                color_strength = min(1.0, (saturation / 255.0) + (value / 510.0))
                confidence = round(
                    (0.42 * purity) + (0.28 * solidity) + (0.15 * area_score) + (0.15 * color_strength),
                    4,
                )

                if saturation < 70.0 or value < 60.0:
                    continue
                if confidence < self._min_confidence:
                    continue

                if confidence > best_result.confidence:
                    best_result = VictimDetectionResult(
                        found=True,
                        victim_type="color",
                        letter=None,
                        color=color_name,
                        confidence=confidence,
                        bbox=(x, y, w, h),
                        source=source,
                    )
        return best_result

    def _detect_letter_from_card(
        self,
        card_roi: Any,
        *,
        bbox: tuple[int, int, int, int],
        source: str,
    ) -> VictimDetectionResult:
        working = self._extract_inner_card_region(card_roi)
        gray = cv2.cvtColor(working, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
        kernel = np.ones((3, 3), dtype=np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return VictimDetectionResult.none(source=source)

        roi_area = float(binary.shape[0] * binary.shape[1])
        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) >= roi_area * 0.01]
        if not filtered_contours:
            return VictimDetectionResult.none(source=source)

        component_candidates: list[tuple[float, Any, tuple[int, int, int, int], int]] = []
        rows, cols = binary.shape
        for contour in sorted(filtered_contours, key=cv2.contourArea, reverse=True):
            x, y, w, h = cv2.boundingRect(contour)
            touches_border = int(x <= 2) + int(y <= 2) + int(x + w >= cols - 2) + int(y + h >= rows - 2)
            area = float(cv2.contourArea(contour))
            fill_ratio = area / float(max(w * h, 1))
            # В реальных фото рамка карточки и тени часто дают крупный внешний контур;
            # предпочитаем внутренний символ, который не липнет к краям ROI.
            score = area - (touches_border * roi_area * 0.3) + (fill_ratio * roi_area * 0.02)
            component_candidates.append((score, contour, (x, y, w, h), touches_border))

        best_result = VictimDetectionResult.none(source=source)
        for _, contour, (x, y, w, h), touches_border in sorted(
            component_candidates,
            key=lambda item: item[0],
            reverse=True,
        )[:5]:
            if touches_border >= 4:
                continue
            prepared = self._prepare_letter_roi(binary[y : y + h, x : x + w])
            if prepared is None:
                continue

            ranked_labels = self._classify_letter_roi(prepared)
            restricted_ranked_labels = [
                (candidate_label, candidate_score, candidate_roi)
                for candidate_label, candidate_score, candidate_roi in ranked_labels
                if candidate_label in self.LETTER_LABELS
            ]
            if restricted_ranked_labels:
                top_label, top_similarity, _ = restricted_ranked_labels[0]
                second_similarity = restricted_ranked_labels[1][1] if len(restricted_ranked_labels) > 1 else 0.0
                if top_label == "H" and top_similarity >= 0.95 and (top_similarity - second_similarity) >= 0.04:
                    confidence = round(min(1.0, 0.78 + ((top_similarity - 0.95) * 2.0)), 4)
                    if confidence > best_result.confidence:
                        best_result = VictimDetectionResult(
                            found=True,
                            victim_type="letter",
                            letter="H",
                            color="black",
                            confidence=confidence,
                            bbox=bbox,
                            source=source,
                        )
                    continue

            label, similarity, confidence = self._classify_structured_letter(prepared)
            touch_penalty = 0.05 * max(0, touches_border - 1)
            confidence = max(0.0, confidence - touch_penalty)
            if confidence < self._min_confidence:
                continue

            if confidence > best_result.confidence:
                best_result = VictimDetectionResult(
                    found=True,
                    victim_type="letter",
                    letter=label,
                    color="black",
                    confidence=round(confidence, 4),
                    bbox=bbox,
                    source=source,
                )
        return best_result

    def _detect_color_from_card(
        self,
        card_roi: Any,
        *,
        bbox: tuple[int, int, int, int],
        source: str,
    ) -> VictimDetectionResult:
        working = self._extract_inner_card_region(card_roi)
        hsv = cv2.cvtColor(working, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), dtype=np.uint8)
        roi_area = working.shape[0] * working.shape[1]

        best_result = VictimDetectionResult.none(source=source)
        for color_name, ranges in self._color_ranges().items():
            mask = np.zeros(working.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = float(cv2.contourArea(contour))
                if area < max(150.0, roi_area * 0.01) or area > roi_area * 0.4:
                    continue

                x, y, w, h = cv2.boundingRect(contour)
                aspect_ratio = w / float(max(h, 1))
                fill_ratio = area / float(max(w * h, 1))
                if not self._COLOR_ASPECT_MIN <= aspect_ratio <= self._COLOR_ASPECT_MAX:
                    continue
                if fill_ratio < 0.45:
                    continue

                roi_mask = mask[y : y + h, x : x + w]
                roi_hsv = hsv[y : y + h, x : x + w]
                colored_pixels = roi_mask > 0
                if not np.any(colored_pixels):
                    continue

                purity = float(np.count_nonzero(colored_pixels)) / float(max(w * h, 1))
                saturation = float(roi_hsv[..., 1][colored_pixels].mean())
                value = float(roi_hsv[..., 2][colored_pixels].mean())
                hull = cv2.convexHull(contour)
                hull_area = float(max(cv2.contourArea(hull), 1.0))
                solidity = area / hull_area
                area_score = min(1.0, area / max(roi_area * 0.08, 1.0))
                color_strength = min(1.0, (saturation / 255.0) + (value / 510.0))
                confidence = round(
                    (0.42 * purity) + (0.28 * solidity) + (0.15 * area_score) + (0.15 * color_strength),
                    4,
                )
                if saturation < 70.0 or value < 60.0 or confidence < self._min_confidence:
                    continue

                if confidence > best_result.confidence:
                    best_result = VictimDetectionResult(
                        found=True,
                        victim_type="color",
                        letter=None,
                        color=color_name,
                        confidence=confidence,
                        bbox=bbox,
                        source=source,
                    )
        return best_result

    def _extract_card_candidates(self, frame: Any) -> list[_CardCandidate]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edge_mask = cv2.Canny(cv2.GaussianBlur(gray, (5, 5), 0), 50, 150)
        edge_mask = cv2.dilate(edge_mask, None, iterations=2)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        bright_mask = cv2.inRange(
            hsv,
            np.array([0, 0, 120], dtype=np.uint8),
            np.array([180, 120, 255], dtype=np.uint8),
        )
        kernel = np.ones((7, 7), dtype=np.uint8)
        bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_CLOSE, kernel)
        bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_OPEN, kernel)

        contours: list[Any] = []
        for mask in (edge_mask, bright_mask):
            found_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours.extend(found_contours)
        frame_area = float(frame.shape[0] * frame.shape[1])
        candidates: list[_CardCandidate] = []
        seen_boxes: set[tuple[int, int, int, int]] = set()
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = float(cv2.contourArea(contour))
            if area < frame_area * 0.02 or area > frame_area * 0.9:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            box_key = (x // 10, y // 10, w // 10, h // 10)
            if box_key in seen_boxes:
                continue
            aspect_ratio = max(w, h) / float(max(min(w, h), 1))
            fill_ratio = area / float(max(w * h, 1))
            if aspect_ratio > 2.5 or fill_ratio < 0.35:
                continue

            rect = cv2.minAreaRect(contour)
            warped = self._warp_rotated_rect(frame, rect)
            if warped is None:
                continue

            if warped.shape[0] < 80 or warped.shape[1] < 80:
                continue
            roi_mean = float(warped.mean())
            if roi_mean < 110.0:
                continue

            candidates.append(_CardCandidate(roi=warped, bbox=(x, y, w, h), area=area))
            seen_boxes.add(box_key)
            if len(candidates) >= 5:
                break
        return candidates

    @staticmethod
    def _extract_inner_card_region(card_roi: Any) -> Any:
        height, width = card_roi.shape[:2]
        margin_y = max(6, int(height * 0.12))
        margin_x = max(6, int(width * 0.12))
        if (height - (2 * margin_y)) < 20 or (width - (2 * margin_x)) < 20:
            return card_roi
        return card_roi[margin_y : height - margin_y, margin_x : width - margin_x]

    def _classify_letter_roi(
        self,
        roi: Any,
    ) -> list[tuple[str, float, Any]]:
        scores: list[tuple[str, float, Any]] = []
        for label, reference_vectors in self._reference_vectors.items():
            label_score = 0.0
            label_variant = roi
            for rotated_candidate in self._rotate_variants_for_candidate(roi):
                candidate_vector = self._letter_feature_vector(rotated_candidate)
                for reference_vector in reference_vectors:
                    similarity = self._binary_similarity(candidate_vector, reference_vector)
                    if similarity > label_score:
                        label_score = similarity
                        label_variant = rotated_candidate
            scores.append((label, label_score, label_variant))

        scores.sort(key=lambda item: item[1], reverse=True)
        return scores

    def _classify_structured_letter(
        self,
        roi: Any,
    ) -> tuple[Literal["H", "S", "U"], float, float]:
        best_label: Literal["H", "S", "U"] = "H"
        best_similarity = 0.0
        best_confidence = 0.0

        for rotated_candidate in self._rotate_variants_for_candidate(roi):
            grid = cv2.resize((rotated_candidate > 0).astype(np.float32), (7, 7), interpolation=cv2.INTER_AREA)
            structure_scores = {
                "H": self._score_h_structure(grid),
                "S": self._score_s_structure(grid),
                "U": self._score_u_structure(grid),
            }
            label = max(structure_scores, key=structure_scores.get)
            ranked_labels = self._classify_letter_roi(rotated_candidate)
            restricted_scores = {
                candidate_label: candidate_score
                for candidate_label, candidate_score, _ in ranked_labels
                if candidate_label in self.LETTER_LABELS
            }
            similarity = restricted_scores.get(label, 0.0)
            structure_margin = structure_scores[label] - max(
                (score for candidate_label, score in structure_scores.items() if candidate_label != label),
                default=0.0,
            )
            confidence = (
                (0.45 * min(1.0, max(0.0, (structure_scores[label] + 0.25) / 2.5)))
                + (0.35 * min(1.0, max(0.0, (similarity - 0.55) / 0.25)))
                + (0.20 * min(1.0, max(0.0, (structure_margin + 0.1) / 0.9)))
            )
            if label == "H" and structure_scores[label] >= 2.5:
                confidence = min(1.0, confidence + 0.12)
            if confidence > best_confidence:
                best_label = label  # type: ignore[assignment]
                best_similarity = similarity
                best_confidence = confidence

        return best_label, best_similarity, best_confidence

    @staticmethod
    def _score_h_structure(grid: Any) -> float:
        top_center = float(grid[1, 2:5].mean())
        middle_center = float(grid[3, 2:5].mean())
        bottom_center = float(grid[5, 2:5].mean())
        left_mid = float(grid[2:5, 1].mean())
        right_mid = float(grid[2:5, 5].mean())
        return (left_mid + right_mid + (2.0 * middle_center)) - top_center - bottom_center

    @staticmethod
    def _score_u_structure(grid: Any) -> float:
        top_center = float(grid[1, 2:5].mean())
        middle_center = float(grid[3, 2:5].mean())
        bottom_center = float(grid[5, 2:5].mean())
        left_mid = float(grid[2:5, 1].mean())
        right_mid = float(grid[2:5, 5].mean())
        return (left_mid + right_mid + (2.0 * bottom_center)) - (2.0 * top_center) - middle_center

    @staticmethod
    def _score_s_structure(grid: Any) -> float:
        top_center = float(grid[1, 2:5].mean())
        middle_center = float(grid[3, 2:5].mean())
        bottom_center = float(grid[5, 2:5].mean())
        left_mid = float(grid[2:5, 1].mean())
        right_mid = float(grid[2:5, 5].mean())
        return top_center + middle_center + bottom_center + min(left_mid, right_mid)

    def _get_capture(self) -> Any:
        self._ensure_vision_dependencies()
        if self._capture is None:
            self._capture = cv2.VideoCapture(self._camera_index)
            if hasattr(self._capture, "set"):
                self._capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not self._capture.isOpened():
                self._capture.release()
                self._capture = None
                raise RaspberryCommandError(f"Не удалось открыть USB-камеру #{self._camera_index}.")
        return self._capture

    @classmethod
    def _normalize_letter(cls, letter: str) -> Literal["H", "S", "U"]:
        normalized = letter.upper().strip()
        if normalized not in cls.LETTER_LABELS:
            raise ValueError(f"Ожидалась одна из букв {cls.LETTER_LABELS}, получено: {letter!r}")
        return normalized  # type: ignore[return-value]

    @classmethod
    def _normalize_color(cls, color: str) -> Literal["black", "red", "yellow", "green"]:
        normalized = color.lower().strip()
        if normalized not in cls._ALL_COLOR_LABELS:
            raise ValueError(f"Ожидался один из цветов {cls._ALL_COLOR_LABELS}, получено: {color!r}")
        return normalized  # type: ignore[return-value]

    @staticmethod
    def _ensure_vision_dependencies() -> None:
        if cv2 is None or np is None:
            raise RaspberryCommandError(
                "Для VictimCamera нужны зависимости opencv-python-headless и numpy. Установите raspberry/requirements.txt."
            )

    @staticmethod
    def _normalize_frame(frame: Any) -> Any:
        VictimCamera._ensure_vision_dependencies()
        if not isinstance(frame, np.ndarray):
            raise TypeError("frame должен быть numpy.ndarray")
        if frame.ndim == 2:
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        if frame.ndim != 3 or frame.shape[2] not in (3, 4):
            raise ValueError("frame должен иметь форму HxW, HxWx3 или HxWx4")
        if frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame

    @classmethod
    def _build_reference_vectors(cls) -> dict[str, list[Any]]:
        cls._ensure_vision_dependencies()
        reference_vectors: dict[str, list[Any]] = {}
        fonts = (
            cv2.FONT_HERSHEY_SIMPLEX,
            cv2.FONT_HERSHEY_DUPLEX,
            cv2.FONT_HERSHEY_COMPLEX,
            cv2.FONT_HERSHEY_TRIPLEX,
        )
        thicknesses = (10, 12)
        for letter in string.ascii_uppercase:
            vectors: list[Any] = []
            for font in fonts:
                for thickness in thicknesses:
                    prepared = cls._build_reference_template(letter, font=font, thickness=thickness)
                    if prepared is not None:
                        vectors.append(cls._letter_feature_vector(prepared))
            reference_vectors[letter] = vectors
        return reference_vectors

    @classmethod
    def _build_reference_template(
        cls,
        letter: str,
        *,
        font: int,
        thickness: int,
    ) -> Any | None:
        canvas = np.full((240, 320, 3), 245, dtype=np.uint8)
        patch = np.full((160, 160, 3), 255, dtype=np.uint8)
        scale = 4.0 if letter != "S" else 3.8
        (text_width, text_height), baseline = cv2.getTextSize(letter, font, scale, thickness)
        origin_x = max((patch.shape[1] - text_width) // 2, 0)
        origin_y = max((patch.shape[0] + text_height) // 2, text_height + baseline)
        cv2.putText(
            patch,
            letter,
            (origin_x, origin_y),
            font,
            scale,
            (0, 0, 0),
            thickness,
            cv2.LINE_AA,
        )
        canvas[40:200, 80:240] = patch

        gray = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY)
        binary = cv2.adaptiveThreshold(
            cv2.GaussianBlur(gray, (5, 5), 0),
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            31,
            9,
        )
        kernel = np.ones((3, 3), dtype=np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        warped = cls._warp_rotated_rect(binary, cv2.minAreaRect(contour))
        if warped is None:
            return None
        return cls._prepare_letter_roi(warped)

    @staticmethod
    def _rotate_variants_for_candidate(roi: Any) -> list[Any]:
        return [np.rot90(roi, rotation) for rotation in range(4)]

    @staticmethod
    def _binary_similarity(candidate: Any, template: Any) -> float:
        score = float(np.dot(candidate, template))
        if score < 0.0:
            return 0.0
        return score

    @staticmethod
    def _letter_feature_grid(roi: Any) -> Any:
        return cv2.resize((roi > 0).astype(np.float32), (5, 5), interpolation=cv2.INTER_AREA)

    @classmethod
    def _letter_feature_vector(cls, roi: Any) -> Any:
        grid = cls._letter_feature_grid(roi)
        vector = grid.flatten()
        norm = float(np.linalg.norm(vector))
        if norm <= 0.0:
            return vector
        return vector / norm

    @staticmethod
    def _normalize_letter_confidence(similarity: float) -> float:
        return min(1.0, max(0.0, (similarity - 0.5) / 0.25))

    def _passes_letter_rules(self, roi: Any, label: Literal["H", "S", "U"]) -> bool:
        holes = self._count_holes(roi)
        grid = self._letter_feature_grid(roi)
        upper_center = float(grid[1, 2])
        middle_center = float(grid[2, 2])
        lower_center = float(grid[3, 2])
        middle_left = float(grid[2, 1])
        middle_right = float(grid[2, 3])
        if label == "H":
            return 1 <= holes <= 3 and upper_center <= 0.15 and middle_center >= 0.45 and lower_center <= 0.15
        if label == "U":
            return holes == 0 and upper_center <= 0.15 and middle_center <= 0.15 and lower_center >= 0.2
        return (
            holes == 0
            and upper_center >= 0.2
            and middle_center >= 0.45
            and lower_center >= 0.2
            and middle_left >= 0.25
            and middle_right >= 0.25
        )

    @staticmethod
    def _count_holes(roi: Any) -> int:
        contours, hierarchy = cv2.findContours(roi, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if hierarchy is None:
            return 0
        return sum(1 for info in hierarchy[0] if info[3] != -1)

    @classmethod
    def _prepare_letter_roi(cls, roi: Any) -> Any | None:
        if roi is None or roi.size == 0:
            return None
        if roi.ndim == 3:
            roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        non_zero_points = cv2.findNonZero(binary)
        if non_zero_points is None:
            return None

        x, y, w, h = cv2.boundingRect(non_zero_points)
        if w < 4 or h < 4:
            return None

        cropped = binary[y : y + h, x : x + w]
        target_inner = cls._TEMPLATE_SIZE - 16
        scale = min(target_inner / float(w), target_inner / float(h))
        resized_width = max(1, int(round(w * scale)))
        resized_height = max(1, int(round(h * scale)))
        resized = cv2.resize(cropped, (resized_width, resized_height), interpolation=cv2.INTER_AREA)
        canvas = np.zeros((cls._TEMPLATE_SIZE, cls._TEMPLATE_SIZE), dtype=np.uint8)
        offset_x = (cls._TEMPLATE_SIZE - resized_width) // 2
        offset_y = (cls._TEMPLATE_SIZE - resized_height) // 2
        canvas[offset_y : offset_y + resized_height, offset_x : offset_x + resized_width] = resized
        return canvas

    @staticmethod
    def _warp_rotated_rect(
        image: Any,
        rect: tuple[tuple[float, float], tuple[float, float], float],
    ) -> Any | None:
        width = max(int(round(rect[1][0])), 1)
        height = max(int(round(rect[1][1])), 1)
        if width < 4 or height < 4:
            return None

        box = cv2.boxPoints(rect)
        ordered_box = VictimCamera._order_points(box.astype("float32"))
        destination = np.array(
            [
                [0, 0],
                [width - 1, 0],
                [width - 1, height - 1],
                [0, height - 1],
            ],
            dtype="float32",
        )
        matrix = cv2.getPerspectiveTransform(ordered_box, destination)
        return cv2.warpPerspective(image, matrix, (width, height))

    @staticmethod
    def _order_points(points: Any) -> Any:
        sums = points.sum(axis=1)
        diffs = np.diff(points, axis=1)
        ordered = np.zeros((4, 2), dtype="float32")
        ordered[0] = points[np.argmin(sums)]
        ordered[2] = points[np.argmax(sums)]
        ordered[1] = points[np.argmin(diffs)]
        ordered[3] = points[np.argmax(diffs)]
        return ordered

    @staticmethod
    def _color_ranges() -> dict[str, list[tuple[Any, Any]]]:
        return {
            "red": [
                (np.array([0, 90, 70], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8)),
                (np.array([160, 90, 70], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8)),
            ],
            "yellow": [
                (np.array([18, 90, 90], dtype=np.uint8), np.array([40, 255, 255], dtype=np.uint8)),
            ],
            "green": [
                (np.array([40, 70, 70], dtype=np.uint8), np.array([95, 255, 255], dtype=np.uint8)),
            ],
        }


class RaspberryService:
    """Высокоуровневый сервис управления системными функциями Raspberry Pi.

    Класс предназначен для операций, которые не выполняются Arduino:
    - настройка и переключение Raspberry Pi в AP-режим через NetworkManager;
    - восстановление клиентского Wi-Fi-профиля после выхода из AP;
    - завершение активных SSH-сессий без остановки master-процесса ``sshd``;
    - чтение штатной телеметрии платы: температуры и признаков проблем с
      питанием.
    """

    def __init__(
        self,
        *,
        logger: logging.Logger | None = None,
        runner: Callable[..., subprocess.CompletedProcess[str]] | None = None,
        state_file: str | Path | None = None,
        ap_profile_name: str = "rescue-maze-ap",
        raspberry_pi_detector: Callable[[], bool] | None = None,
        euid_getter: Callable[[], int] | None = None,
    ) -> None:
        """Создаёт сервис управления системными командами Raspberry Pi.

        Args:
            logger: Пользовательский logger для служебных сообщений.
            runner: Функция запуска внешних команд. По умолчанию используется
                ``subprocess.run``.
            state_file: Путь к файлу, в котором сохраняется предыдущий
                клиентский Wi‑Fi-профиль для последующего восстановления.
            ap_profile_name: Имя профиля NetworkManager, используемого для
                режима точки доступа.
            raspberry_pi_detector: Необязательная функция определения того,
                что код выполняется именно на Raspberry Pi. Нужна в основном
                для тестов и подмены окружения.
            euid_getter: Необязательная функция получения эффективного UID.
                Нужна в основном для тестов.
        """
        self._logger = logger or LOGGER
        self._runner = runner or subprocess.run
        self._state_file = Path(state_file or "/tmp/rescue_maze_ap_state.json")
        self._ap_profile_name = ap_profile_name
        self._raspberry_pi_detector = raspberry_pi_detector or self._default_raspberry_pi_detector
        self._euid_getter = euid_getter or getattr(os, "geteuid", None)

    def configure_ap(
        self,
        ssid: str,
        password: str,
        *,
        channel: int | Literal["auto"] = 1,
        ipv4_cidr: str = "192.168.4.1/24",
    ) -> None:
        """Создаёт или обновляет профиль точки доступа в NetworkManager.

        Args:
            ssid: Имя Wi‑Fi сети, которую будет раздавать Raspberry Pi.
            password: Пароль WPA-PSK для точки доступа.
            channel: Радиоканал Wi‑Fi. Можно передать число или строку
                ``"auto"``, чтобы выбрать канал автоматически на основе
                загруженности эфира и поддерживаемых адаптером частот.
            ipv4_cidr: Адрес и маска сети для интерфейса AP в формате CIDR.

        Raises:
            ValueError: Если входные параметры заведомо некорректны.
            RaspberryCommandError: Если ``nmcli`` недоступна или вернула
                ошибку.
        """
        self._ensure_elevated_privileges()
        if not ssid:
            raise ValueError("ssid не должен быть пустым")
        if len(password) < 8:
            raise ValueError("password должен содержать минимум 8 символов")
        if channel != "auto" and not 1 <= channel <= 13:
            raise ValueError("channel должен быть в диапазоне от 1 до 13")

        selected_channel = channel
        if channel == "auto":
            selected_channel = self.select_ap_channel()

        if not self._connection_exists(self._ap_profile_name):
            self._nmcli(
                "connection",
                "add",
                "type",
                "wifi",
                "con-name",
                self._ap_profile_name,
                "ssid",
                ssid,
                "autoconnect",
                "no",
            )

        self._nmcli(
            "connection",
            "modify",
            self._ap_profile_name,
            "802-11-wireless.mode",
            "ap",
            "802-11-wireless.ssid",
            ssid,
            "802-11-wireless.band",
            "bg",
            "802-11-wireless.channel",
            str(selected_channel),
            "wifi-sec.key-mgmt",
            "wpa-psk",
            "wifi-sec.psk",
            password,
            "ipv4.method",
            "manual",
            "ipv4.addresses",
            ipv4_cidr,
            "ipv6.method",
            "disabled",
            "connection.autoconnect",
            "no",
        )

    def select_ap_channel(self, wifi_device: str | None = None) -> int:
        """Автоматически выбирает наименее загруженный канал для AP.

        Алгоритм рассчитан на режим точки доступа в диапазоне 2.4 ГГц и
        опирается на две группы данных:
        - какие каналы реально поддерживает Wi‑Fi адаптер Raspberry Pi;
        - какие соседние точки доступа сейчас видны в эфире.

        Приоритетно рассматриваются непересекающиеся каналы ``1``, ``6`` и
        ``11``. Если адаптер или регуляторный домен не позволяют использовать
        эти каналы, сервис аккуратно переходит к поддерживаемым альтернативам.

        Args:
            wifi_device: Имя Wi‑Fi интерфейса. Если не указано, определяется
                автоматически.

        Returns:
            Номер канала, который стоит использовать для точки доступа.

        Raises:
            RaspberryCommandError: Если не удалось определить Wi‑Fi интерфейс,
                получить список поддерживаемых каналов или просканировать эфир.
        """
        self._ensure_elevated_privileges()
        wifi_device = wifi_device or self._detect_wifi_device()
        supported_channels = self._get_supported_24ghz_channels(wifi_device)
        if not supported_channels:
            raise RaspberryCommandError(
                f"Не удалось определить поддерживаемые 2.4 ГГц каналы для интерфейса {wifi_device}."
            )

        access_points = self._scan_wifi_access_points(wifi_device)
        preferred_channels = [channel for channel in (1, 6, 11) if channel in supported_channels]
        candidate_channels = preferred_channels or supported_channels

        best_channel = min(
            candidate_channels,
            key=lambda candidate: (
                self._channel_interference_score(candidate, access_points),
                0 if candidate in preferred_channels else 1,
                candidate,
            ),
        )
        self._logger.info(
            "Автоматически выбран Wi‑Fi канал %s для интерфейса %s",
            best_channel,
            wifi_device,
        )
        return best_channel

    def enable_ap(self) -> None:
        """Активирует AP-профиль на Wi-Fi интерфейсе Raspberry Pi.

        Метод пытается:
        1. определить Wi-Fi-интерфейс;
        2. запомнить активный клиентский профиль;
        3. отключить конфликтующие клиентские подключения;
        4. поднять профиль точки доступа.

        Raises:
            RaspberryCommandError: Если Wi‑Fi устройство не найдено или команды
                ``nmcli`` завершились ошибкой.
        """
        self._ensure_elevated_privileges()
        wifi_device = self._detect_wifi_device()
        previous_client = self._active_wifi_connection_name(wifi_device)
        if previous_client and previous_client != self._ap_profile_name:
            self._save_state({"previous_client": previous_client})
            self._nmcli("connection", "down", previous_client, check=False)

        self._nmcli("device", "disconnect", wifi_device, check=False)
        self._nmcli("connection", "up", self._ap_profile_name, "ifname", wifi_device)

    def disable_ap(self) -> None:
        """Отключает AP-профиль и пытается восстановить прошлый Wi-Fi-клиент.

        Если ранее был сохранён активный клиентский профиль, сервис поднимет
        его на том же Wi-Fi интерфейсе после отключения точки доступа.

        Raises:
            RaspberryCommandError: Если ``nmcli`` недоступна или вернула ошибку.
        """
        self._ensure_elevated_privileges()
        state = self._load_state()
        previous_client = state.get("previous_client")
        wifi_device = self._detect_wifi_device()

        self._nmcli("connection", "down", self._ap_profile_name, check=False)

        if previous_client:
            self._nmcli("connection", "up", previous_client, "ifname", wifi_device)
            self._save_state({})

    def disconnect_all_ssh(self) -> list[int]:
        """Завершает все активные SSH-сессии, не трогая master ``sshd``.

        Метод анализирует дерево процессов, находит дочерние процессы
        интерактивных или рабочих SSH-сессий и отправляет им ``SIGTERM`` в
        обратном порядке, начиная с самых глубоких потомков.

        Returns:
            Список PID, которым был отправлен сигнал завершения.
        """
        self._ensure_elevated_privileges()
        processes = self._read_process_table()
        session_pids = self._select_ssh_session_pids(processes)
        if not session_pids:
            return []

        terminated = []
        for pid in sorted(session_pids, reverse=True):
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                continue
            terminated.append(pid)
        return terminated

    def get_temperature_telemetry(self) -> dict[str, Any]:
        """Возвращает текущую температуру Raspberry Pi и простую оценку нагрева.

        Метод сначала пытается прочитать стандартный Linux sysfs-файл
        ``/sys/class/thermal/thermal_zone0/temp``. Если он недоступен, сервис
        делает резервную попытку через ``vcgencmd measure_temp``.

        Returns:
            Словарь со следующими полями:
            - ``celsius``: температура в градусах Цельсия;
            - ``fahrenheit``: та же температура в градусах Фаренгейта;
            - ``state``: приблизительная словесная оценка нагрева
              ``normal``/``warm``/``hot``/``critical``;
            - ``source``: откуда были получены данные.

        Raises:
            RaspberryCommandError: Если ни один источник температурной
                телеметрии не доступен или вернул повреждённые данные.
        """
        thermal_zone_path = Path("/sys/class/thermal/thermal_zone0/temp")
        temperature_celsius: float | None = None
        source: str | None = None

        if self._path_exists(thermal_zone_path):
            try:
                temperature_celsius = self._parse_thermal_zone_temperature(self._read_text_file(thermal_zone_path))
                source = str(thermal_zone_path)
            except (OSError, ValueError) as exc:
                self._logger.debug(
                    "Не удалось прочитать температуру Raspberry Pi из %s: %s",
                    thermal_zone_path,
                    exc,
                )

        if temperature_celsius is None:
            result = self._run_command(["vcgencmd", "measure_temp"], check=False)
            if result.returncode != 0 or not result.stdout.strip():
                stderr = (result.stderr or "").strip()
                raise RaspberryCommandError(
                    "Не удалось получить температуру Raspberry Pi через sysfs или vcgencmd: "
                    f"{stderr or 'команда не вернула данных'}"
                )
            try:
                temperature_celsius = self._parse_vcgencmd_temperature(result.stdout)
            except ValueError as exc:
                raise RaspberryCommandError(
                    f"Не удалось разобрать температуру из vcgencmd: {result.stdout.strip()}"
                ) from exc
            source = "vcgencmd measure_temp"

        return {
            "celsius": round(temperature_celsius, 2),
            "fahrenheit": round((temperature_celsius * 9.0 / 5.0) + 32.0, 2),
            "state": self._temperature_state(temperature_celsius),
            "source": source,
        }

    def get_power_telemetry(self) -> dict[str, Any]:
        """Возвращает штатную телеметрию питания Raspberry Pi.

        Важно понимать, что это не измерение аккумулятора, тока или внешнего
        блока питания. Сервис читает только встроенные признаки состояния самой
        платы Raspberry Pi: флаги undervoltage, throttling, ограничения частоты
        и, если доступно, внутреннее напряжение ядра через ``vcgencmd``.

        Returns:
            Словарь с полями:
            - ``throttled_raw``: исходная строка из ``vcgencmd get_throttled``;
            - ``throttled_mask``: целочисленная битовая маска;
            - ``core_voltage_volts``: напряжение ядра в вольтах или ``None``;
            - ``voltage_source``: источник измерения напряжения или ``None``;
            - ``undervoltage_now`` и ``undervoltage_occurred``;
            - ``frequency_capped_now`` и ``frequency_capped_occurred``;
            - ``throttled_now`` и ``throttling_occurred``;
            - ``soft_temperature_limit_now`` и
              ``soft_temperature_limit_occurred``;
            - ``power_good_now``: нет ли сейчас признака просадки питания;
            - ``performance_limited_now``: ограничена ли сейчас
              производительность.

        Raises:
            RaspberryCommandError: Если ``vcgencmd get_throttled`` недоступна
                или вернула неожиданный ответ.
        """
        throttled_result = self._run_command(["vcgencmd", "get_throttled"], check=False)
        if throttled_result.returncode != 0 or not throttled_result.stdout.strip():
            stderr = (throttled_result.stderr or "").strip()
            raise RaspberryCommandError(
                "Не удалось получить телеметрию питания Raspberry Pi через vcgencmd get_throttled: "
                f"{stderr or 'команда не вернула данных'}"
            )

        try:
            throttled_raw, throttled_mask = self._parse_vcgencmd_throttled(throttled_result.stdout)
        except ValueError as exc:
            raise RaspberryCommandError(
                f"Не удалось разобрать ответ vcgencmd get_throttled: {throttled_result.stdout.strip()}"
            ) from exc

        core_voltage_volts: float | None = None
        voltage_source: str | None = None
        voltage_result = self._run_command(["vcgencmd", "measure_volts", "core"], check=False)
        if voltage_result.returncode == 0 and voltage_result.stdout.strip():
            try:
                core_voltage_volts = self._parse_vcgencmd_voltage(voltage_result.stdout)
                voltage_source = "vcgencmd measure_volts core"
            except ValueError:
                self._logger.debug(
                    "Не удалось разобрать напряжение ядра из vcgencmd: %s",
                    voltage_result.stdout.strip(),
                )

        undervoltage_now = bool(throttled_mask & 0x1)
        frequency_capped_now = bool(throttled_mask & 0x2)
        throttled_now = bool(throttled_mask & 0x4)
        soft_temperature_limit_now = bool(throttled_mask & 0x8)

        return {
            "throttled_raw": throttled_raw,
            "throttled_mask": throttled_mask,
            "core_voltage_volts": core_voltage_volts,
            "voltage_source": voltage_source,
            "undervoltage_now": undervoltage_now,
            "undervoltage_occurred": bool(throttled_mask & 0x10000),
            "frequency_capped_now": frequency_capped_now,
            "frequency_capped_occurred": bool(throttled_mask & 0x20000),
            "throttled_now": throttled_now,
            "throttling_occurred": bool(throttled_mask & 0x40000),
            "soft_temperature_limit_now": soft_temperature_limit_now,
            "soft_temperature_limit_occurred": bool(throttled_mask & 0x80000),
            "power_good_now": not undervoltage_now,
            "performance_limited_now": frequency_capped_now or throttled_now or soft_temperature_limit_now,
        }

    def get_board_telemetry(self) -> dict[str, Any]:
        """Возвращает сводную телеметрию платы Raspberry Pi.

        Это удобный метод верхнего уровня, который одним вызовом собирает
        температуру и признаки проблем с питанием.

        Returns:
            Словарь с двумя ключами:
            - ``temperature``: результат ``get_temperature_telemetry()``;
            - ``power``: результат ``get_power_telemetry()``.
        """
        return {
            "temperature": self.get_temperature_telemetry(),
            "power": self.get_power_telemetry(),
        }

    def _connection_exists(self, name: str) -> bool:
        result = self._nmcli("connection", "show", name, check=False)
        return result.returncode == 0

    def _ensure_elevated_privileges(self) -> None:
        """Требует root-права только при работе на реальной Raspberry Pi.

        На обычных машинах разработчика, CI и других не-RPi окружениях
        проверка не мешает использованию класса. На Raspberry Pi без
        повышенных прав будет выброшено исключение до запуска системных
        команд.
        """
        if not self._raspberry_pi_detector():
            return

        if self._euid_getter is None:
            raise RaspberryCommandError(
                "На Raspberry Pi не удалось проверить повышенные права: функция geteuid недоступна."
            )

        if self._euid_getter() != 0:
            raise RaspberryCommandError(
                "Для методов RaspberryService на Raspberry Pi нужны повышенные права. "
                "Запустите код от root или через sudo."
            )

    @staticmethod
    def _default_raspberry_pi_detector() -> bool:
        """Определяет, выполняется ли код на реальной Raspberry Pi."""
        if platform.system() != "Linux":
            return False

        model_paths = (
            Path("/proc/device-tree/model"),
            Path("/sys/firmware/devicetree/base/model"),
        )
        for model_path in model_paths:
            try:
                model = model_path.read_text(encoding="utf-8", errors="ignore").strip("\x00\r\n ")
            except OSError:
                continue
            if "raspberry pi" in model.lower():
                return True
        return False

    def _detect_wifi_device(self) -> str:
        result = self._nmcli("device", "status")
        for line in result.stdout.splitlines():
            if not line.strip() or line.startswith("DEVICE"):
                continue
            parts = line.split()
            if len(parts) >= 2 and parts[1] == "wifi":
                return parts[0]
        raise RaspberryCommandError("NetworkManager не обнаружил Wi-Fi-устройство")

    def _active_wifi_connection_name(self, wifi_device: str) -> str | None:
        result = self._nmcli("-t", "-f", "NAME,DEVICE", "connection", "show", "--active")
        for line in result.stdout.splitlines():
            if not line.strip():
                continue
            name, _, device = line.partition(":")
            if device == wifi_device:
                return name
        return None

    def _get_supported_24ghz_channels(self, wifi_device: str) -> list[int]:
        channels = self._parse_iw_phy_channels(self._run_command(["iw", "phy"], check=False).stdout)
        if channels:
            return channels

        iwlist_result = self._run_command(["iwlist", wifi_device, "frequency"], check=False)
        channels = self._parse_iwlist_channels(iwlist_result.stdout)
        if channels:
            return channels

        return []

    @staticmethod
    def _parse_iw_phy_channels(output: str) -> list[int]:
        channels: set[int] = set()
        for raw_line in output.splitlines():
            line = raw_line.strip()
            if "MHz" not in line or "disabled" in line:
                continue
            match = re.search(r"\[(\d+)\]", line)
            if match is None:
                continue
            channel = int(match.group(1))
            if 1 <= channel <= 13:
                channels.add(channel)
        return sorted(channels)

    @staticmethod
    def _parse_iwlist_channels(output: str) -> list[int]:
        channels: set[int] = set()
        for raw_line in output.splitlines():
            match = re.search(r"Channel\s+(\d+)", raw_line, re.IGNORECASE)
            if match is None:
                continue
            channel = int(match.group(1))
            if 1 <= channel <= 13:
                channels.add(channel)
        return sorted(channels)

    def _scan_wifi_access_points(self, wifi_device: str) -> list[WifiAccessPointInfo]:
        result = self._nmcli(
            "-t",
            "-f",
            "CHAN,SIGNAL",
            "device",
            "wifi",
            "list",
            "--rescan",
            "yes",
            "ifname",
            wifi_device,
        )
        return self._parse_wifi_scan_output(result.stdout)

    @staticmethod
    def _parse_wifi_scan_output(output: str) -> list[WifiAccessPointInfo]:
        access_points: list[WifiAccessPointInfo] = []
        for raw_line in output.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split(":")
            if len(parts) < 2:
                continue
            channel_text = parts[0].strip()
            signal_text = parts[-1].strip()
            if not channel_text.isdigit() or not signal_text.isdigit():
                continue
            channel = int(channel_text)
            signal = int(signal_text)
            if 1 <= channel <= 13:
                access_points.append(WifiAccessPointInfo(channel=channel, signal=signal))
        return access_points

    @staticmethod
    def _channel_interference_score(channel: int, access_points: Iterable[WifiAccessPointInfo]) -> float:
        weights = {
            0: 1.0,
            1: 2.5,
            2: 2.0,
            3: 1.5,
            4: 1.2,
        }
        score = 0.0
        for access_point in access_points:
            distance = abs(access_point.channel - channel)
            weight = weights.get(distance)
            if weight is None:
                continue
            score += access_point.signal * weight
        return score

    @staticmethod
    def _parse_thermal_zone_temperature(raw_text: str) -> float:
        match = re.search(r"(-?\d+)", raw_text)
        if match is None:
            raise ValueError("Не найдено целочисленное значение температуры")
        return int(match.group(1)) / 1000.0

    @staticmethod
    def _parse_vcgencmd_temperature(output: str) -> float:
        match = re.search(r"temp=([0-9]+(?:\.[0-9]+)?)'C", output)
        if match is None:
            raise ValueError("Не найдено значение температуры vcgencmd")
        return float(match.group(1))

    @staticmethod
    def _parse_vcgencmd_voltage(output: str) -> float:
        match = re.search(r"volt=([0-9]+(?:\.[0-9]+)?)V", output)
        if match is None:
            raise ValueError("Не найдено значение напряжения vcgencmd")
        return float(match.group(1))

    @staticmethod
    def _parse_vcgencmd_throttled(output: str) -> tuple[str, int]:
        stripped = output.strip()
        match = re.search(r"throttled=(0x[0-9a-fA-F]+)", stripped)
        if match is None:
            raise ValueError("Не найдена throttled-маска")
        raw_mask = match.group(1)
        return raw_mask, int(raw_mask, 16)

    @staticmethod
    def _temperature_state(temperature_celsius: float) -> str:
        if temperature_celsius < 60.0:
            return "normal"
        if temperature_celsius < 75.0:
            return "warm"
        if temperature_celsius < 80.0:
            return "hot"
        return "critical"

    def _read_process_table(self) -> list[ProcessInfo]:
        result = self._run_command(["ps", "-eo", "pid=,ppid=,comm=,args="])
        processes: list[ProcessInfo] = []
        for line in result.stdout.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            fields = stripped.split(None, 3)
            if len(fields) < 4:
                continue
            pid_text, ppid_text, command, args = fields
            processes.append(
                ProcessInfo(
                    pid=int(pid_text),
                    ppid=int(ppid_text),
                    command=command,
                    args=args,
                )
            )
        return processes

    @staticmethod
    def _select_ssh_session_pids(processes: Iterable[ProcessInfo]) -> set[int]:
        processes = list(processes)
        children: dict[int, list[int]] = {}
        process_map = {process.pid: process for process in processes}

        for process in processes:
            children.setdefault(process.ppid, []).append(process.pid)

        master_pids = {
            process.pid
            for process in processes
            if process.command == "sshd"
            and ("-D" in process.args or "[listener]" in process.args or "(sshd)" in process.args)
        }

        session_roots = {
            process.pid
            for process in processes
            if process.command == "sshd" and process.pid not in master_pids
        }

        result: set[int] = set()
        stack = list(session_roots)
        while stack:
            pid = stack.pop()
            process = process_map.get(pid)
            if process is None or pid in master_pids:
                continue
            if pid not in result:
                result.add(pid)
                stack.extend(children.get(pid, []))

        return result

    def _nmcli(self, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
        return self._run_command(["nmcli", *args], check=check)

    def _run_command(
        self,
        command: list[str],
        *,
        check: bool = True,
    ) -> subprocess.CompletedProcess[str]:
        try:
            result = self._runner(command, capture_output=True, text=True, check=False)
        except FileNotFoundError as exc:
            raise RaspberryCommandError(f"Команда недоступна: {command[0]}") from exc

        if check and result.returncode != 0:
            stderr = (result.stderr or "").strip()
            raise RaspberryCommandError(
                f"Команда {' '.join(command)} завершилась с кодом {result.returncode}: {stderr}"
            )
        return result

    def _save_state(self, state: dict[str, str]) -> None:
        self._state_file.parent.mkdir(parents=True, exist_ok=True)
        self._state_file.write_text(json.dumps(state), encoding="utf-8")

    def _load_state(self) -> dict[str, str]:
        if not self._state_file.exists():
            return {}
        try:
            return json.loads(self._state_file.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            self._logger.warning("Файл состояния AP %s повреждён, он будет проигнорирован", self._state_file)
            return {}

    @staticmethod
    def _read_text_file(path: Path) -> str:
        return path.read_text(encoding="utf-8", errors="ignore")

    @staticmethod
    def _path_exists(path: Path) -> bool:
        return path.exists()
