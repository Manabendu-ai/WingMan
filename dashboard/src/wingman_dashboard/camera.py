"""Camera source for the MJPEG stream. Shows the frame the VLM is looking at,
with the detected target circled. Stage 2 adds webcam / RTSP / ROS sources."""
import io
from pathlib import Path

from PIL import Image, ImageDraw


class ReplayCamera:
    def __init__(self, hub, size=(640, 480), overlay_scale: float = 1000.0):
        # overlay_scale: VLM target_x/target_y are assumed to be on a 0..overlay_scale
        # grid. If your VLM returns pixels, set it to the image width (and see README).
        self.hub, self.size, self.scale = hub, size, overlay_scale

    def jpeg(self) -> bytes:
        v = self.hub
        with v._lock:
            path, obs = v.frame_path, v.last_obs
        w, h = self.size
        img = None
        if path is not None and Path(path).is_file():
            try:
                img = Image.open(path).convert("RGB").resize(self.size)
            except Exception:
                img = None
        if img is None:
            img = Image.new("RGB", self.size, (28, 30, 36))
            d = ImageDraw.Draw(img)
            label = "NO SIGNAL" if path is None else f"frame: {Path(path).name}"
            d.text((16, 16), label, fill=(150, 155, 165))
        d = ImageDraw.Draw(img)
        if obs and obs.get("target_visible") and obs.get("x") is not None:
            cx, cy = obs["x"] / self.scale * w, obs["y"] / self.scale * h
            r = 28
            d.ellipse([cx - r, cy - r, cx + r, cy + r], outline=(0, 230, 90), width=4)
            d.text((cx + r + 4, cy - r), f"target {obs.get('confidence', 0):.2f}", fill=(0, 230, 90))
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=80)
        return buf.getvalue()
