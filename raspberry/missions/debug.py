"""Camera-only mission for viewing processed images and intersection detection."""

import time

import cv2

import config as cfg
from perception.image_processing import find_intersection, find_line, traitement


WINDOW_NAME = "Robot debug - camera / traitement"


def run_debug(camera, stop_event):
    """Display one annotated camera frame every configured interval; never command motors."""
    next_capture = 0.0
    try:
        while not stop_event.is_set():
            now = time.monotonic()
            if now >= next_capture:
                # Picamera2 is configured as RGB888; traitement expects a BGR image.
                frame = cv2.cvtColor(camera.capture_array("main"), cv2.COLOR_RGB2BGR)
                mask, height = traitement(frame)
                line = find_line(mask, height)
                intersection = find_intersection(mask, height)

                annotated = frame.copy()
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
                print(line_status, "|", intersection_status, flush=True)
                next_capture = now + cfg.DEBUG_CAPTURE_INTERVAL_S

            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
    finally:
        cv2.destroyWindow(WINDOW_NAME)
