from __future__ import annotations

import unittest
from pathlib import Path
import shutil
import uuid

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - локально тесты пропускаются без vision-зависимостей
    cv2 = None  # type: ignore[assignment]
    np = None  # type: ignore[assignment]

from raspberry.raspberry_service import VictimCamera


def _skip_if_vision_missing() -> None:
    if cv2 is None or np is None:
        raise unittest.SkipTest("Vision dependencies are not installed")


def _make_letter_frame(letter: str, *, rotation: int | None = None) -> np.ndarray:
    _skip_if_vision_missing()
    frame = np.full((240, 320, 3), 245, dtype=np.uint8)
    patch = np.full((160, 160, 3), 255, dtype=np.uint8)
    font = cv2.FONT_HERSHEY_DUPLEX
    scale = 4.0
    thickness = 12
    (text_width, text_height), baseline = cv2.getTextSize(letter, font, scale, thickness)
    origin = ((160 - text_width) // 2, (160 + text_height) // 2)
    cv2.putText(patch, letter, origin, font, scale, (0, 0, 0), thickness, cv2.LINE_AA)
    if rotation is not None:
        patch = cv2.rotate(patch, rotation)
    frame[40:200, 80:240] = patch
    return frame


def _make_color_frame(color_name: str) -> np.ndarray:
    _skip_if_vision_missing()
    colors = {
        "red": (0, 0, 255),
        "yellow": (0, 255, 255),
        "green": (0, 180, 0),
        "blue": (255, 0, 0),
    }
    frame = np.full((240, 320, 3), 245, dtype=np.uint8)
    cv2.rectangle(frame, (90, 50), (220, 180), colors[color_name], -1)
    return frame


def _make_noise_frame() -> np.ndarray:
    _skip_if_vision_missing()
    frame = np.full((240, 320, 3), 245, dtype=np.uint8)
    rng = np.random.default_rng(42)
    for _ in range(30):
        x = int(rng.integers(0, 320))
        y = int(rng.integers(0, 240))
        radius = int(rng.integers(2, 8))
        color = tuple(int(value) for value in rng.integers(0, 255, size=3))
        cv2.circle(frame, (x, y), radius, color, -1)
    return frame


class VictimCameraTests(unittest.TestCase):
    def setUp(self) -> None:
        _skip_if_vision_missing()

    def test_letter_detection_supports_rotations(self) -> None:
        camera = VictimCamera()
        rotations = {
            "0": None,
            "90": cv2.ROTATE_90_CLOCKWISE,
            "180": cv2.ROTATE_180,
            "270": cv2.ROTATE_90_COUNTERCLOCKWISE,
        }

        for letter in ("H", "S", "U"):
            for rotation_name, rotation in rotations.items():
                with self.subTest(letter=letter, rotation=rotation_name):
                    result = camera.analyze_frame(_make_letter_frame(letter, rotation=rotation))
                    self.assertTrue(result.found)
                    self.assertEqual(result.victim_type, "letter")
                    self.assertEqual(result.letter, letter)
                    self.assertEqual(result.color, "black")
                    self.assertGreaterEqual(result.confidence, 0.74)

    def test_color_detection_supports_primary_victims(self) -> None:
        camera = VictimCamera()
        for color_name in ("red", "yellow", "green"):
            with self.subTest(color=color_name):
                result = camera.analyze_frame(_make_color_frame(color_name))
                self.assertTrue(result.found)
                self.assertEqual(result.victim_type, "color")
                self.assertEqual(result.color, color_name)
                self.assertIsNone(result.letter)

    def test_noise_and_non_target_objects_return_none(self) -> None:
        camera = VictimCamera()
        cases = {
            "random-noise": _make_noise_frame(),
            "letter-A": _make_letter_frame("A"),
            "letter-C": _make_letter_frame("C"),
            "blue-color": _make_color_frame("blue"),
        }

        for name, frame in cases.items():
            with self.subTest(case=name):
                result = camera.analyze_frame(frame)
                self.assertFalse(result.found)
                self.assertEqual(result.victim_type, "none")
                self.assertIsNone(result.letter)
                self.assertIsNone(result.color)

    def test_analyze_file_reads_image_from_disk(self) -> None:
        camera = VictimCamera()
        temp_dir = Path("tests") / f".tmp_victim_camera_{uuid.uuid4().hex}"
        temp_dir.mkdir(parents=True, exist_ok=True)
        try:
            image_path = temp_dir / "victim_h.png"
            self.assertTrue(cv2.imwrite(str(image_path), _make_letter_frame("H")))

            result = camera.analyze_file(image_path)
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)

        self.assertTrue(result.found)
        self.assertEqual(result.letter, "H")
        self.assertEqual(result.source, str(image_path))

    def test_debug_render_preserves_frame_shape(self) -> None:
        camera = VictimCamera()
        frame = _make_letter_frame("S")
        result = camera.analyze_frame(frame)

        debug_frame = camera.render_debug_frame(frame, result)

        self.assertEqual(debug_frame.shape, frame.shape)
        self.assertFalse(np.array_equal(debug_frame, frame))

    def test_live_stabilization_ignores_single_noise_frame(self) -> None:
        frames = iter([_make_noise_frame(), _make_letter_frame("H"), _make_noise_frame()])
        camera = VictimCamera(frame_provider=lambda: next(frames), stability_frames=3)

        first = camera.detect_victim()
        second = camera.detect_victim()
        third = camera.detect_victim()

        self.assertFalse(first.found)
        self.assertFalse(second.found)
        self.assertFalse(third.found)

    def test_live_stabilization_confirms_after_two_matching_frames(self) -> None:
        frames = iter([_make_noise_frame(), _make_letter_frame("U"), _make_letter_frame("U")])
        camera = VictimCamera(frame_provider=lambda: next(frames), stability_frames=3)

        first = camera.detect_victim()
        second = camera.detect_victim()
        third = camera.detect_victim()

        self.assertFalse(first.found)
        self.assertFalse(second.found)
        self.assertTrue(third.found)
        self.assertEqual(third.letter, "U")

    def test_accessors_use_last_detection_result(self) -> None:
        camera = VictimCamera()
        camera.analyze_frame(_make_letter_frame("H"))

        self.assertTrue(camera.has_letter())
        self.assertTrue(camera.has_letter("H"))
        self.assertEqual(camera.get_letter(), "H")
        self.assertEqual(camera.get_color(), "black")
        self.assertTrue(camera.has_color("black"))
        self.assertFalse(camera.has_color())

    def test_real_dataset_letter_samples_are_detected(self) -> None:
        camera = VictimCamera()
        dataset_cases = {
            "0.jpg": "U",
            "1.jpg": "S",
            "3.jpg": "H",
            "4.jpg": "U",
            "5.jpg": "S",
            "9.jpg": "S",
            "13.jpg": "S",
            "16.jpg": "U",
            "18.jpg": "H",
            "20.jpg": "U",
            "21.jpg": "S",
            "22.jpg": "H",
            "23.jpg": "H",
            "26.jpg": "H",
            "27.jpg": "U",
            "29.jpg": "H",
            "30.jpg": "U",
            "32.jpg": "H",
        }

        for filename, expected_letter in dataset_cases.items():
            with self.subTest(filename=filename):
                result = camera.analyze_file(Path("tests") / "data" / filename)
                self.assertTrue(result.found)
                self.assertEqual(result.victim_type, "letter")
                self.assertEqual(result.letter, expected_letter)
                self.assertEqual(result.color, "black")

    def test_real_dataset_letter_samples_do_not_fall_back_to_false_color(self) -> None:
        camera = VictimCamera()
        for filename in ("12.jpg", "13.jpg", "21.jpg", "29.jpg", "30.jpg"):
            with self.subTest(filename=filename):
                result = camera.analyze_file(Path("tests") / "data" / filename)
                self.assertNotEqual(result.victim_type, "color")

    def test_real_dataset_distant_card_uses_local_bbox(self) -> None:
        camera = VictimCamera()
        result = camera.analyze_file(Path("tests") / "data" / "23.jpg")

        self.assertTrue(result.found)
        self.assertEqual(result.letter, "H")
        self.assertIsNotNone(result.bbox)
        assert result.bbox is not None
        _, _, width, height = result.bbox
        self.assertLess(width, 800)
        self.assertLess(height, 500)


if __name__ == "__main__":
    unittest.main()
