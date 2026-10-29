#!/usr/bin/env python3
"""Standalone VLM benchmark (no ROS). Measures latency, memory, JSON validity, run-to-run consistency and,
with labels, accuracy. The VLM output is ADVISORY: obstacle avoidance stays with lidar/depth + planner.

  # Ollama (gemma3:4b)
  python3 vlm_bench.py --images ~/ws_wingman/vlm/test_images --model gemma3:4b --runs 3
  # llama.cpp server (e.g. SmolVLM2), OpenAI-compatible API
  python3 vlm_bench.py --backend openai --host http://localhost:8080 --model smolvlm2 --images ...

Optional ground truth: <images>/labels.json
  {"frame_0001.png": {"scene": "corridor", "person_detected": false, "visible_openings": ["forward"]}}"""
import argparse
import base64
import glob
import io
import json
import os
import re
import statistics
import time
import urllib.request

SCENES = ["corridor", "junction", "dead_end", "room", "unknown"]
OPENINGS = ["left", "forward", "right"]
SCHEMA = {
    "type": "object",
    "properties": {
        "scene": {"type": "string", "enum": SCENES},
        "visible_openings": {"type": "array", "items": {"type": "string", "enum": OPENINGS}},
        "person_detected": {"type": "boolean"},
        "obstacle_ahead": {"type": "boolean"},
        "confidence": {"type": "number", "minimum": 0, "maximum": 1},
        "evidence": {"type": "string"},
    },
    "required": ["scene", "visible_openings", "person_detected", "obstacle_ahead", "confidence", "evidence"],
}
PROMPT = (
    "You are the visual assistant of a small drone flying in a maze. The image is from its forward camera. "
    "Reply with ONE JSON object only, no other text, with keys: "
    "scene (one of corridor, junction, dead_end, room, unknown); "
    "visible_openings (list of left, forward, right where a corridor or doorway clearly continues); "
    "person_detected (true only if a human is clearly visible); "
    "obstacle_ahead (true if something blocks the view straight ahead); "
    "confidence (0 to 1); evidence (one short sentence of what you see). "
    "Do not judge distances or whether it is safe to fly; other sensors do that."
)


# ------------------------------------------------------------------ helpers
def validate(o):
    err = []
    if not isinstance(o, dict):
        return ['not an object']
    for k in SCHEMA['required']:
        if k not in o:
            err.append(f'missing {k}')
    if err:
        return err
    if o['scene'] not in SCENES:
        err.append('bad scene')
    if not (isinstance(o['visible_openings'], list) and all(x in OPENINGS for x in o['visible_openings'])):
        err.append('bad visible_openings')
    for k in ('person_detected', 'obstacle_ahead'):
        if not isinstance(o[k], bool):
            err.append(f'{k} not bool')
    c = o['confidence']
    if isinstance(c, bool) or not isinstance(c, (int, float)) or not 0 <= c <= 1:
        err.append('bad confidence')
    if not isinstance(o['evidence'], str):
        err.append('bad evidence')
    return err


def extract_json(text):
    t = re.sub(r'```(?:json)?', '', text).strip()
    a, b = t.find('{'), t.rfind('}')
    if a < 0 or b <= a:
        return None
    try:
        return json.loads(t[a:b + 1])
    except Exception:
        return None


def prep_image(path, size):
    raw = open(path, 'rb').read()
    try:
        from PIL import Image
        im = Image.open(io.BytesIO(raw)).convert('RGB')
        im.thumbnail((size, size))
        buf = io.BytesIO()
        im.save(buf, format='PNG')
        return buf.getvalue()
    except ImportError:
        return raw


def mem_available_mb():
    for line in open('/proc/meminfo'):
        if line.startswith('MemAvailable'):
            return int(line.split()[1]) / 1024
    return float('nan')


def server_rss_mb(names=('ollama', 'llama-server', 'llama.cpp', 'llama_cpp')):
    total = 0.0
    me = os.getpid()
    for d in glob.glob('/proc/[0-9]*'):
        try:
            if int(os.path.basename(d)) == me:
                continue
            comm = open(d + '/comm').read().strip()
            cmd = open(d + '/cmdline').read().replace('\0', ' ')
            if not any(n in comm or n in cmd.split(' ')[0] for n in names):
                continue
            for line in open(d + '/status'):
                if line.startswith('VmRSS'):
                    total += int(line.split()[1]) / 1024
        except Exception:
            continue
    return total


def post(url, body, timeout):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), headers={'Content-Type': 'application/json'})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


def query(args, png):
    b64 = base64.b64encode(png).decode()
    t0 = time.perf_counter()
    if args.backend == 'ollama':
        body = {'model': args.model, 'stream': False, 'format': SCHEMA, 'keep_alive': '10m',
                'options': {'temperature': 0, 'num_ctx': 2048, 'num_predict': 200},
                'messages': [{'role': 'user', 'content': PROMPT, 'images': [b64]}]}
        r = post(args.host.rstrip('/') + '/api/chat', body, args.timeout)
        wall = time.perf_counter() - t0
        info = {'text': r.get('message', {}).get('content', ''),
                'prompt_tokens': r.get('prompt_eval_count'), 'gen_tokens': r.get('eval_count'),
                'load_s': (r.get('load_duration') or 0) / 1e9, 'eval_s': (r.get('eval_duration') or 0) / 1e9}
    else:
        body = {'model': args.model, 'temperature': 0, 'max_tokens': 200,
                'messages': [{'role': 'user', 'content': [
                    {'type': 'text', 'text': PROMPT},
                    {'type': 'image_url', 'image_url': {'url': 'data:image/png;base64,' + b64}}]}]}
        r = post(args.host.rstrip('/') + '/v1/chat/completions', body, args.timeout)
        wall = time.perf_counter() - t0
        u = r.get('usage', {})
        info = {'text': r['choices'][0]['message']['content'],
                'prompt_tokens': u.get('prompt_tokens'), 'gen_tokens': u.get('completion_tokens'),
                'load_s': 0.0, 'eval_s': 0.0}
    info['wall_s'] = wall
    return info


def jaccard(a, b):
    a, b = set(a), set(b)
    return 1.0 if not a and not b else len(a & b) / len(a | b)


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--images', required=True)
    ap.add_argument('--model', default='gemma3:4b')
    ap.add_argument('--backend', choices=['ollama', 'openai'], default='ollama')
    ap.add_argument('--host', default=None)
    ap.add_argument('--runs', type=int, default=2)
    ap.add_argument('--size', type=int, default=512, help='longest image side sent to the model')
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--out', default=None, help='write per-request results as JSON')
    args = ap.parse_args()
    args.host = args.host or ('http://localhost:11434' if args.backend == 'ollama' else 'http://localhost:8080')
    d = os.path.expanduser(args.images)
    files = sorted(sum([glob.glob(os.path.join(d, e)) for e in ('*.png', '*.jpg', '*.jpeg')], []))
    if not files:
        raise SystemExit(f'no images in {d}')
    labels = {}
    lp = os.path.join(d, 'labels.json')
    if os.path.exists(lp):
        labels = json.load(open(lp))

    print(f'{len(files)} images x {args.runs} runs, model={args.model}, backend={args.backend}, size<={args.size}px')
    mem0 = mem_available_mb()
    print(f'MemAvailable before: {mem0:.0f} MB')
    rows, rss_peak, mem_min = [], 0.0, mem0
    for f in files:
        png = prep_image(f, args.size)
        for run in range(args.runs):
            try:
                info = query(args, png)
            except Exception as e:
                rows.append({'file': os.path.basename(f), 'run': run, 'error': str(e)})
                print(f'  {os.path.basename(f)} run{run}: REQUEST FAILED: {e}')
                continue
            obj = extract_json(info['text'])
            errs = ['no JSON found'] if obj is None else validate(obj)
            rss_peak = max(rss_peak, server_rss_mb())
            mem_min = min(mem_min, mem_available_mb())
            rows.append({'file': os.path.basename(f), 'run': run, 'cold': len(rows) == 0, 'json_ok': obj is not None,
                         'schema_ok': not errs, 'errors': errs, 'obj': obj, **{k: info[k] for k in
                         ('wall_s', 'prompt_tokens', 'gen_tokens', 'load_s', 'eval_s')}})
            print(f'  {os.path.basename(f)} run{run}: {info["wall_s"]:6.1f}s  '
                  f'{"OK " if not errs else "BAD"} {json.dumps(obj) if obj else info["text"][:80]!r}')

    ok = [r for r in rows if 'error' not in r]
    print('\n=== SUMMARY ===')
    print(f'requests: {len(rows)}  failed: {len(rows) - len(ok)}')
    if ok:
        print(f'valid JSON: {100 * sum(r["json_ok"] for r in ok) / len(ok):.0f}%   '
              f'schema-valid: {100 * sum(r["schema_ok"] for r in ok) / len(ok):.0f}%')
        warm = [r['wall_s'] for r in ok if not r['cold']] or [r['wall_s'] for r in ok]
        print(f'latency warm: median {statistics.median(warm):.1f}s  max {max(warm):.1f}s   '
              f'cold (first request): {ok[0]["wall_s"]:.1f}s')
        pt = [r['prompt_tokens'] for r in ok if r['prompt_tokens']]
        if pt:
            print(f'prompt tokens (image + text): ~{int(statistics.median(pt))}')
        ev = [(r['gen_tokens'], r['eval_s']) for r in ok if r['gen_tokens'] and r['eval_s']]
        if ev:
            print(f'generation speed: {sum(g for g, _ in ev) / sum(s for _, s in ev):.1f} tokens/s')
        # run-to-run consistency
        if args.runs > 1:
            agree = []
            for f in files:
                sigs = {json.dumps([o['obj']['scene'], o['obj']['person_detected'], sorted(o['obj']['visible_openings'])])
                        for o in ok if o['file'] == os.path.basename(f) and o['obj'] and o['schema_ok']}
                if sigs:
                    agree.append(len(sigs) == 1)
            if agree:
                print(f'run-to-run agreement (scene+person+openings identical): {100 * sum(agree) / len(agree):.0f}% of images')
        # accuracy
        if labels:
            tp = fp = fn = tn = 0
            sc_ok = sc_n = 0
            jac = []
            for r in ok:
                lab = labels.get(r['file'])
                if not lab or not r['obj'] or not r['schema_ok']:
                    continue
                if 'person_detected' in lab:
                    p, t = r['obj']['person_detected'], lab['person_detected']
                    tp += p and t; fp += p and not t; fn += (not p) and t; tn += (not p) and not t
                if 'scene' in lab:
                    sc_n += 1; sc_ok += r['obj']['scene'] == lab['scene']
                if 'visible_openings' in lab:
                    jac.append(jaccard(r['obj']['visible_openings'], lab['visible_openings']))
            print(f'person: TP={tp} FP={fp} FN={fn} TN={tn}   (a missed person = FN; a false alarm = FP)')
            if sc_n:
                print(f'scene accuracy: {100 * sc_ok / sc_n:.0f}% ({sc_ok}/{sc_n})')
            if jac:
                print(f'openings overlap (Jaccard): {statistics.mean(jac):.2f}')
        else:
            print('(no labels.json: accuracy not computed - run-to-run consistency is NOT correctness)')
    print(f'MemAvailable: {mem0:.0f} MB before -> minimum {mem_min:.0f} MB during  (drop {mem0 - mem_min:.0f} MB)')
    print(f'model-server RSS peak (ollama/llama-server processes): {rss_peak:.0f} MB')
    if args.out:
        json.dump(rows, open(os.path.expanduser(args.out), 'w'), indent=1)


if __name__ == '__main__':
    main()
