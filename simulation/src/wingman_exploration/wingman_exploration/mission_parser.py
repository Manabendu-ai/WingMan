"""Turn an operator's on-the-spot mission text (or JSON) into a structured mission. No ROS imports.

Mission dict keys
  type   : explore | find | goto | return | takeoff | land | hover
  target : text the VLM looks for (find)
  then   : what to do when the mission goal is met:
           return (fly back to the entrance and land) | land (land here) | hover | continue
  xy     : [x, y] metres in the map frame (goto)
  altitude : flight height above the floor in metres (optional; default = what the controller was launched with)
             free text: "at 0.75 m", "maintain a height of 0.75 meters", "altitude 75 cm"
Examples
  "explore the whole maze"                      -> explore, then return
  "find the person and come back"               -> find person, then return
  "find a red box and stay there"               -> find red box, then hover
  "find the person and keep exploring"          -> find, then continue (reports every one found)
  "go to 4 -10" / "return home" / "land" / "stop"
"""
import json
import math
import re

ALT_MIN, ALT_MAX = 0.4, 2.0      # metres; anything outside is clamped (the maze corridors are ~2.5 m high)

THEN_WORDS = [
    ("continue", r"\b(continue|keep (exploring|searching|going)|find (all|every)|all of them)\b"),
    ("land", r"\b(and )?land( there| here| now)?\b"),
    ("hover", r"\b(stay|hover|wait|stop there|stay there|hold there)\b"),
    ("return", r"\b(return|come back|go back|back to (the )?(start|entrance|home)|go home|fly home)\b"),
]
FIND_RE = re.compile(
    r"\b(?:find|locate|search for|look for|spot|detect|identify|seek)\s+(?:for\s+)?(?:an?\s+|the\s+|any\s+|some\s+)?(.+)", re.I)
CUT_RE = re.compile(r"\s*(?:,|;|\band\b|\bthen\b|\bafter\b|\bonce\b|\bwhen\b|\bbefore\b|\bso\b)\s*", re.I)


_NUM = r"(\d+(?:\.\d+)?)"
_UNIT = r"(m|meters?|metres?|cm|centimeters?|centimetres?)\b"
ALT_PATTERNS = [
    # "(while) (maintaining|keeping|holding) (a) (height|altitude) (of|at) 0.75 m"
    re.compile(r"[,;]?\s*(?:\b(?:while|and|but|with)\s+)?(?:\b(?:maintain\w*|keep\w*|hold\w*|fly\w*|stay\w*)\s+)?"
               r"(?:\b(?:a|an|the|at)\s+)?\b(?:height|altitude|alt)\b[^0-9]{0,25}?" + _NUM + r"\s*" + _UNIT, re.I),
    re.compile(r"[,;]?\s*(?:\b(?:while|and|but|with)\s+)?(?:\b(?:maintain\w*|keep\w*|hold\w*|fly\w*|stay\w*)\s+)?"
               r"\bat\s+" + _NUM + r"\s*" + _UNIT + r"(?!\s*(?:away|from))", re.I),
    re.compile(r"[,;]?\s*" + _NUM + r"\s*" + _UNIT + r"\s+(?:height|altitude|high|above the (?:floor|ground))", re.I),
]


def _clamp_alt(v):
    return round(max(ALT_MIN, min(ALT_MAX, float(v))), 2)


def extract_altitude(text):
    """-> (altitude_m | None, text_without_the_altitude_phrase)."""
    for pat in ALT_PATTERNS:
        m = pat.search(text)
        if m:
            v = float(m.group(1))
            if m.group(2).lower().startswith("c"):
                v /= 100.0
            return _clamp_alt(v), (text[:m.start()] + " " + text[m.end():]).strip()
    return None, text


NON_VISUAL_TARGET = re.compile(r"^(?:the\s+)?(?:shortest|fastest|quickest|best|optimal|safest)?\s*"
                               r"(?:path|route|way|exit|way out|end|finish|goal)\b", re.I)


def _then(text, default):
    # "land at the starting point / where you started / back home" means: fly back first, the return lands there
    if re.search(r"\bland(?:ing)?\b", text) and re.search(
            r"\b(start\w*|entrance|home|began|take ?off|origin|where (?:you|it) (?:started|began|took off))\b", text):
        return "return"
    for name, pat in THEN_WORDS:
        if re.search(pat, text):
            return name
    return default


def parse_mission(raw: str):
    """Returns (mission_dict | None, error_message)."""
    raw = (raw or "").strip()
    if not raw:
        return None, "empty mission"
    if raw.startswith("{"):
        try:
            obj = json.loads(raw)
            t = str(obj.get("type", obj.get("command", "explore"))).lower()
            if t not in {"explore", "find", "goto", "return", "takeoff", "land", "hover"}:
                return None, f"unknown mission type {t!r}"
            m = {"type": t, "target": str(obj.get("target", "person")), "then": str(obj.get("then", "return")),
                 "xy": obj.get("xy"), "text": raw, "altitude": None}
            if m["then"] not in {"return", "land", "hover", "continue"}:
                return None, f"unknown 'then' {m['then']!r} (return|land|hover|continue)"
            alt = obj.get("altitude", obj.get("altitude_m", obj.get("height_m")))
            if alt is not None:
                try:
                    alt = float(alt)
                except (TypeError, ValueError):
                    return None, f"altitude must be a number in metres, got {alt!r}"
                if not math.isfinite(alt):
                    return None, "altitude must be finite"
                m["altitude"] = _clamp_alt(alt)
            if t == "goto" and not (isinstance(m["xy"], (list, tuple)) and len(m["xy"]) == 2):
                return None, "goto needs xy:[x,y]"
            return m, ""
        except Exception as exc:
            return None, f"invalid JSON: {exc}"
    altitude, raw_clean = extract_altitude(raw)
    t = raw_clean.lower().strip().rstrip(".!")
    base = {"target": "", "then": "return", "xy": None, "text": raw, "altitude": altitude}
    if altitude is not None and not t:
        t = "explore"
    if re.fullmatch(r"(land|land now|touch ?down)", t):
        return {**base, "type": "land"}, ""
    if re.fullmatch(r"(stop|hover|hold|wait|pause|abort|halt|freeze)( there| here| now)?", t):
        return {**base, "type": "hover"}, ""
    if re.fullmatch(r"(take ?off|launch|lift off|takeoff and hover)", t):
        return {**base, "type": "takeoff"}, ""
    g = re.search(r"\b(?:go to|fly to|goto|navigate to)\s+(?:x\s*=?\s*)?(-?\d+(?:\.\d+)?)[ ,]+(?:y\s*=?\s*)?(-?\d+(?:\.\d+)?)", t)
    if g:
        return {**base, "type": "goto", "xy": [float(g.group(1)), float(g.group(2))],
                "then": _then(t, "hover")}, ""
    f = FIND_RE.search(raw_clean)
    if f and NON_VISUAL_TARGET.match(f.group(1).strip()):
        f = None        # "find the shortest path" is not a visual search: it is exploration
    if f:
        tail = f.group(1)
        target = CUT_RE.split(tail)[0].strip(" .,!?") or "person"
        target = re.sub(r"\b(in|inside|within|around)\b.*$", "", target).strip() or target
        # words after the target decide what happens on success
        after = tail[len(target):].lower() if tail.lower().startswith(target.lower()) else t
        return {**base, "type": "find", "target": target, "then": _then(after if after.strip() else t, "return")}, ""
    if re.search(r"\b(return|come back|go back|go home|fly home|back to (the )?(start|entrance|home))\b", t) \
            and not re.search(r"\b(explore|map|search)\b", t):
        return {**base, "type": "return", "then": "land" if re.search(r"\bland\b", t) else "return"}, ""
    if re.search(r"\b(explore|map|scan|survey|search|cover|sweep|maze|everything|whole|entire)\b", t):
        return {**base, "type": "explore", "then": _then(t, "return")}, ""
    return None, ("could not understand the mission. Try: 'explore the maze', 'find the person and come back', "
                  "'go to 4 -10', 'return home', 'land', 'stop'")


MAX_COMMAND_CHARS = 2000


def normalize_command(raw: str):
    """Gate for commands that arrive over the network (the website / its LLM).
    Accepts a JSON object or plain text. Returns (canonical_json_str | None, mission | None, error).

    JSON forms (all keys optional except one of type/prompt):
      {"id": "42", "type": "explore", "then": "return", "altitude": 0.75}
      {"id": "43", "type": "find", "target": "person", "then": "return", "altitude": 0.75}
      {"id": "44", "type": "goto", "xy": [4, -10], "then": "hover"}
      {"id": "45", "prompt": "explore the maze and return home at 0.75 m"}   # free text, parsed here
    """
    raw = (raw or "").strip()
    if not raw:
        return None, None, "empty command"
    if len(raw) > MAX_COMMAND_CHARS:
        return None, None, f"command longer than {MAX_COMMAND_CHARS} characters"
    text = raw
    if raw.startswith("{"):
        try:
            obj = json.loads(raw)
        except Exception as exc:
            return None, None, f"invalid JSON: {exc}"
        if not isinstance(obj, dict):
            return None, None, "command must be a JSON object"
        prompt = obj.get("prompt", obj.get("text"))
        if "type" not in obj and "command" not in obj and isinstance(prompt, str):
            text = prompt
            if "altitude" in obj or "altitude_m" in obj:       # explicit altitude wins over one found in the text
                text = json.dumps({**parse_mission(prompt)[0], "altitude": obj.get("altitude", obj.get("altitude_m"))}) \
                    if parse_mission(prompt)[0] else prompt
        elif isinstance(obj.get("type", obj.get("command")), str):
            keep = {k: obj[k] for k in ("type", "command", "target", "then", "xy", "altitude", "altitude_m", "height_m")
                    if k in obj}
            text = json.dumps(keep)
    m, err = parse_mission(text)
    if m is None:
        return None, None, err
    canon = {"type": m["type"], "target": m["target"], "then": m["then"], "xy": m["xy"], "altitude": m["altitude"]}
    return json.dumps(canon), m, ""
