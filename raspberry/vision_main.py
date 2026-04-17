from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from pathlib import Path
from typing import TextIO

try:
    import cv2
except ImportError:  # pragma: no cover - опционально до установки зависимостей
    cv2 = None  # type: ignore[assignment]

try:
    from .raspberry_service import VictimCamera, VictimDetectionResult
except ImportError:  # pragma: no cover - позволяет запускать файл напрямую
    from raspberry_service import VictimCamera, VictimDetectionResult  # type: ignore[no-redef]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Распознавание буквенных и цветовых жертв по USB-камере Raspberry Pi или изображениям."
    )
    parser.add_argument(
        "images",
        nargs="*",
        help="Один или несколько путей к изображениям. Если не переданы, будет проанализирован один кадр с камеры.",
    )
    parser.add_argument(
        "--camera-index",
        type=int,
        default=0,
        help="Индекс USB-камеры для режима захвата одного кадра. По умолчанию 0.",
    )
    parser.add_argument(
        "--min-confidence",
        type=float,
        default=VictimCamera._DEFAULT_MIN_CONFIDENCE,
        help="Минимальная уверенность детекции в диапазоне от 0.0 до 1.0.",
    )
    parser.add_argument(
        "--stability-frames",
        type=int,
        default=3,
        help="Размер окна стабилизации для live-режима API. Для CLI влияет только на создание объекта.",
    )
    parser.add_argument(
        "--debug-dir",
        type=Path,
        default=None,
        help="Папка, в которую будут сохранены debug-изображения с рамками и подписью.",
    )
    return parser


def _format_result(label: str, result: VictimDetectionResult) -> str:
    bbox = result.bbox if result.bbox is not None else "-"
    return (
        f"{label}: found={result.found} "
        f"type={result.victim_type} "
        f"letter={result.letter or '-'} "
        f"color={result.color or '-'} "
        f"confidence={result.confidence:.4f} "
        f"bbox={bbox} "
        f"source={result.source}"
    )


def _save_debug_image(
    camera: VictimCamera,
    frame,
    result: VictimDetectionResult,
    target_path: Path,
) -> None:
    if cv2 is None:
        raise RuntimeError("Для debug-изображений нужен OpenCV.")
    target_path.parent.mkdir(parents=True, exist_ok=True)
    debug_frame = camera.render_debug_frame(frame, result)
    if not cv2.imwrite(str(target_path), debug_frame):
        raise RuntimeError(f"Не удалось сохранить debug-изображение: {target_path}")


def main(argv: Sequence[str] | None = None, *, output: TextIO | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    output = output or sys.stdout

    if not 0.0 <= args.min_confidence <= 1.0:
        parser.error("--min-confidence должен быть в диапазоне от 0.0 до 1.0.")
    if args.stability_frames <= 0:
        parser.error("--stability-frames должен быть больше нуля.")

    try:
        with VictimCamera(
            camera_index=args.camera_index,
            min_confidence=args.min_confidence,
            stability_frames=args.stability_frames,
        ) as camera:
            if args.images:
                for raw_path in args.images:
                    image_path = Path(raw_path)
                    result = camera.analyze_file(image_path)
                    print(_format_result(str(image_path), result), file=output)
                    if args.debug_dir is not None:
                        if cv2 is None:
                            raise RuntimeError("Для сохранения debug-изображений нужен OpenCV.")
                        frame = cv2.imread(str(image_path))
                        if frame is None:
                            raise RuntimeError(f"Не удалось повторно открыть изображение для debug: {image_path}")
                        debug_path = args.debug_dir / f"{image_path.stem}_debug.png"
                        _save_debug_image(camera, frame, result, debug_path)
                        print(f"debug={debug_path}", file=output)
                return 0

            frame = camera.capture_frame()
            result = camera.analyze_frame(frame, source=f"camera:{args.camera_index}")
            print(_format_result(f"camera:{args.camera_index}", result), file=output)
            if args.debug_dir is not None:
                debug_path = args.debug_dir / f"camera_{args.camera_index}_debug.png"
                _save_debug_image(camera, frame, result, debug_path)
                print(f"debug={debug_path}", file=output)
            return 0
    except Exception as exc:
        print(f"[ERROR] Не удалось выполнить vision-анализ: {exc}", file=output)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
