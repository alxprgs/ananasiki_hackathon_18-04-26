from __future__ import annotations

import io
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

from raspberry import vision_main


def _skip_if_vision_missing() -> None:
    if cv2 is None or np is None:
        raise unittest.SkipTest("Vision dependencies are not installed")


def _make_letter_frame(letter: str) -> np.ndarray:
    _skip_if_vision_missing()
    frame = np.full((240, 320, 3), 245, dtype=np.uint8)
    patch = np.full((160, 160, 3), 255, dtype=np.uint8)
    font = cv2.FONT_HERSHEY_DUPLEX
    scale = 4.0
    thickness = 12
    (text_width, text_height), baseline = cv2.getTextSize(letter, font, scale, thickness)
    origin = ((160 - text_width) // 2, (160 + text_height) // 2)
    cv2.putText(patch, letter, origin, font, scale, (0, 0, 0), thickness, cv2.LINE_AA)
    frame[40:200, 80:240] = patch
    return frame


class VisionMainTests(unittest.TestCase):
    def setUp(self) -> None:
        _skip_if_vision_missing()

    def test_file_mode_prints_structured_result_and_saves_debug_image(self) -> None:
        output = io.StringIO()
        temp_dir = Path("tests") / f".tmp_vision_main_{uuid.uuid4().hex}"
        temp_dir.mkdir(parents=True, exist_ok=True)
        try:
            image_path = temp_dir / "victim_s.png"
            debug_dir = temp_dir / "debug"
            self.assertTrue(cv2.imwrite(str(image_path), _make_letter_frame("S")))

            exit_code = vision_main.main([str(image_path), "--debug-dir", str(debug_dir)], output=output)

            rendered = output.getvalue()
            self.assertEqual(exit_code, 0)
            self.assertIn("found=True", rendered)
            self.assertIn("type=letter", rendered)
            self.assertIn("letter=S", rendered)
            self.assertTrue((debug_dir / "victim_s_debug.png").exists())
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)

    def test_missing_file_returns_error(self) -> None:
        output = io.StringIO()

        exit_code = vision_main.main(["missing-file.png"], output=output)

        self.assertEqual(exit_code, 1)
        self.assertIn("[ERROR]", output.getvalue())


if __name__ == "__main__":
    unittest.main()
