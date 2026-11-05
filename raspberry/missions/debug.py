"""Camera-only mission for viewing processed images and intersection detection."""

import time

import cv2

from perception.image_processing import find_intersection, find_line, traitement


WINDOW_NAME = "Robot debug - camera / traitement"


def run_debug(camera, stop_event, link):
    """Display and analyze every available camera frame; never command motors."""
    try:
        while not stop_event.is_set():
            capture_start = time.perf_counter()
            frame = camera.capture_array("main")
            capture_ms = (time.perf_counter() - capture_start) * 1000

            processing_start = time.perf_counter()
            stage_start = time.perf_counter()
            mask, height = traitement(frame)
            preprocessing_ms = (time.perf_counter() - stage_start) * 1000

            stage_start = time.perf_counter()
            line = find_line(mask, height)
            line_detection_ms = (time.perf_counter() - stage_start) * 1000

            stage_start = time.perf_counter()
            intersection = find_intersection(mask, height)
            intersection_detection_ms = (time.perf_counter() - stage_start) * 1000
            processing_ms = (time.perf_counter() - processing_start) * 1000

            # Match the driver's raw frame input for processing; convert only for display.
            annotated = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if line is None:
                line_status = "Line: not detected"
            else:
                x, y = line
                cv2.circle(annotated, (x, y), 6, (0, 0, 255), -1)
                line_status = f"Line: x={x}, y={y}"

            if intersection is None:
                intersection_status = "Intersection: none"
            else:
                branches = [
                    name for name in ("gauche", "devant", "droite")
                    if intersection[name]
                ]
                intersection_status = "Intersection: " + ", ".join(branches)
                cv2.putText(
                    annotated,
                    intersection_status,
                    (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA,
                )

            cv2.putText(
                annotated,
                line_status,
                (5, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 0, 255),
                1,
                cv2.LINE_AA,
            )
            processed = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            display = cv2.hconcat((annotated, processed))
            cv2.imshow(WINDOW_NAME, display)
            link.send_event(
                "debug_image_timing",
                capture_ms=round(capture_ms, 2),
                preprocessing_ms=round(preprocessing_ms, 2),
                line_detection_ms=round(line_detection_ms, 2),
                intersection_detection_ms=round(intersection_detection_ms, 2),
                processing_ms=round(processing_ms, 2),
            )
            print(
                f"{line_status} | {intersection_status} | "
                f"processing: {processing_ms:.2f} ms",
                flush=True,
            )

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cv2.destroyWindow(WINDOW_NAME)
