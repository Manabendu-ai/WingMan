import argparse
from pathlib import Path

from PIL import Image, ImageDraw

from wingman_vlm.client import observe

HERE = Path(__file__).parent
DATA, OUT = HERE / "data", HERE / "out"
ap = argparse.ArgumentParser()
ap.add_argument("--task", default="find the red box")
args = ap.parse_args()

OUT.mkdir(exist_ok=True)
for p in sorted(DATA.iterdir()):
    if p.suffix.lower() not in {".jpg", ".jpeg", ".png", ".webp"}:
        continue
    o = observe(str(p), args.task).obs
    img = Image.open(p).convert("RGB")
    w, h = img.size
    d = ImageDraw.Draw(img)
    if o.target_visible and o.target_x is not None:
        cx, cy = o.target_x / 1000 * w, o.target_y / 1000 * h
        r = max(w, h) // 40
        d.ellipse([cx - r, cy - r, cx + r, cy + r], outline="lime", width=4)
    d.text((10, 10),
           f"{o.scene} | {o.recommended_direction} | conf {o.confidence:.2f}",
           fill="yellow")
    img.save(OUT / f"{p.stem}.png")
    print("saved", OUT / f"{p.stem}.png")
