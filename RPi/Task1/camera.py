"""Keep the Pi camera running for the whole Task 1 run.

rpicam-still starts the sensor from cold for every photo: ~2-3 s of start-up
and exposure settling, which the robot spends standing still in front of each
obstacle. Opening Picamera2 once and saving the next frame on each SNAP takes
~0.1-0.3 s instead. Frames are only saved when snap_and_rec asks, so the
running camera never records anything between obstacles.

The still configuration is the sensor's full resolution (3280x2464 on the
IMX219), the same images the 3/3 run used. If Picamera2 cannot open the camera
(missing package, or videostream/main.py still holds it), capture() returns
False and the caller falls back to rpicam-still.
"""
import time

# Exposure/white balance settle once, when the camera opens -- not per photo.
WARMUP_S = 1.0
# The STM reports DONE when the wheels stop; let the chassis stop rocking.
SETTLE_S = 0.1
# rpicam-still's default JPEG quality, so images match the 3/3 run.
JPEG_QUALITY = 93


class WarmCamera:
    def __init__(self, logger):
        self.logger = logger
        self.cam = None
        try:
            from picamera2 import Picamera2
            cam = Picamera2()
            self.cam = cam
            cam.configure(cam.create_still_configuration())
            cam.options["quality"] = JPEG_QUALITY
            cam.start()
            time.sleep(WARMUP_S)
            self.logger.info("Camera kept running for the run (fast capture).")
        except Exception as exc:
            # Release a partially opened sensor before trying rpicam-still.
            self.close()
            self.logger.warning(
                f"Warm camera unavailable ({exc}); using rpicam-still per photo. "
                "Is videostream/main.py still holding the camera?")

    def capture(self, filename) -> bool:
        """Save the next frame to filename. False means use the fallback."""
        if self.cam is None:
            return False
        try:
            time.sleep(SETTLE_S)
            self.cam.capture_file(str(filename))
            return True
        except Exception as exc:
            self.logger.error(f"Warm camera capture failed ({exc}); falling back to rpicam-still.")
            self.close()
            return False

    def close(self):
        cam, self.cam = self.cam, None
        if cam is not None:
            try:
                cam.stop()
            except Exception:
                pass
            try:
                cam.close()
            except Exception:
                pass
