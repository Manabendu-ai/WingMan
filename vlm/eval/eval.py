import argparse
import csv
import json
from pathlib import Path

from wingman_vlm.client import observe

HERE = Path(__file__).parent
ap = argparse.ArgumentParser()
ap.add_argument("--task", default="find the red box")
args = ap.parse_args()

rows = list(csv.DictReader(open(HERE / "labels.csv")))
ok = vis = direc = retries = 0
lat, results = [], []

for r in rows:
    res = observe(str(HERE / "data" / r["filename"]), args.task)
    lat.append(res.latency)
    retries += res.attempts == 1 and res.ok
    if not res.ok:
        print(f"{r['filename']:16} INVALID/FALLBACK ({res.latency:.2f}s)")
        results.append({"file": r["filename"], "ok": False, "raw": res.raw})
        continue
    ok += 1
    v = res.obs.target_visible == (r["target_visible"].strip().lower() == "true")
    d = res.obs.recommended_direction == r["recommended_direction"].strip()
    vis += v
    direc += d
    print(f"{r['filename']:16} target:{'OK ' if v else 'BAD'} "
          f"dir:{'OK ' if d else 'BAD'} conf:{res.obs.confidence:.2f} "
          f"({res.latency:.2f}s)")
    results.append({"file": r["filename"], "ok": True, "target_ok": v,
                    "dir_ok": d, "obs": res.obs.model_dump()})

n = len(rows)
warm = lat[1:] or lat
print("\n--- SUMMARY ---")
print(f"Images:             {n}")
print(f"Valid JSON:         {ok}/{n}  (needed retry: {retries})")
print(f"Target visible OK:  {vis}/{n}")
print(f"Direction OK:       {direc}/{n}")
print(f"Avg latency (warm): {sum(warm)/len(warm):.2f}s")
json.dump(results, open(HERE / "results.json", "w"), indent=2)
