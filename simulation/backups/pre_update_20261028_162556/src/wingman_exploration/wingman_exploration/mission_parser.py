"""Turn an operator's on-the-spot mission text (or JSON) into a structured mission. No ROS imports.

Mission dict keys
  type   : explore | find | goto | return | takeoff | land | hover
  target : text the VLM looks for (find)
  then   : what to do when the mission goal is met:
           return (fly back to the entrance and land) | land (land here) | hover | continue
  xy     : [x, y] metres in the map frame (goto)
Examples
  "explore the whole maze"                      -> explore, then return
  "find the person and come back"               -> find person, then return
  "find a red box and stay there"               -> find red box, then hover
  "find the person and keep exploring"          -> find, then continue (reports every one found)
  "go to 4 -10" / "return home" / "land" / "stop"
"""
import json
import re

THEN_WORDS = [
    ("continue", r"\b(continue|keep (exploring|searching|going)|find (all|every)|all of them)\b"),
    ("land", r"\b(and )?land( there| here| now)?\b"),
    ("hover", r"\b(stay|hover|wait|stop there|stay there|hold there)\b"),
    ("return", r"\b(return|come back|go back|back to (the )?(start|entrance|home)|go home|fly home)\b"),
]
FIND_RE = re.compile(
    r"\b(?:find|locate|search for|look for|spot|detect|identify|seek)\s+(?:for\s+)?(?:an?\s+|the\s+|any\s+|some\s+)?(.+)", re.I)
CUT_RE = re.compile(r"\s*(?:,|;|\band\b|\bthen\b|\bafter\b|\bonce\b|\bwhen\b|\bbefore\b|\bso\b)\s*", re.I)


def _then(text, default):
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
                 "xy": obj.get("xy"), "text": raw}
            if t == "goto" and not (isinstance(m["xy"], (list, tuple)) and len(m["xy"]) == 2):
                return None, "goto needs xy:[x,y]"
            return m, ""
        except Exception as exc:
            return None, f"invalid JSON: {exc}"
    t = raw.lower().strip().rstrip(".!")
    base = {"target": "", "then": "return", "xy": None, "text": raw}
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
    f = FIND_RE.search(raw)
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
