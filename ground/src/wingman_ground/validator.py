"""Turns raw LLM output into a validated Mission, or readable errors."""
import json
import re
from dataclasses import dataclass, field
from typing import List, Optional, Union

from pydantic import ValidationError

from .mission import Mission

_THINK_RE = re.compile(r"<think>.*?</think>", re.S | re.I)
_JSON_RE = re.compile(r"\{.*\}", re.S)


@dataclass
class ValidationResult:
    ok: bool
    mission: Optional[Mission] = None
    errors: List[str] = field(default_factory=list)
    clarification: Optional[str] = None   # set when the LLM asked a question


def _clean_error(e: dict) -> str:
    loc = ".".join(str(p) for p in e.get("loc", ()))
    msg = str(e.get("msg", "invalid")).replace("Value error, ", "")
    return f"{loc}: {msg}" if loc else msg


def parse_json(raw: str) -> Optional[dict]:
    text = _THINK_RE.sub("", raw)          # drop reasoning blocks
    m = _JSON_RE.search(text)              # tolerate ``` fences / extra text
    if not m:
        return None
    try:
        data = json.loads(m.group(0))
    except json.JSONDecodeError:
        return None
    return data if isinstance(data, dict) else None


def validate_mission(raw: Union[str, dict]) -> ValidationResult:
    if isinstance(raw, str):
        data = parse_json(raw)
        if data is None:
            return ValidationResult(
                False, errors=["output is not a single valid JSON object"])
    elif isinstance(raw, dict):
        data = raw
    else:
        return ValidationResult(False, errors=["unsupported input type"])

    question = data.get("clarification_needed")
    if isinstance(question, str) and question.strip():
        return ValidationResult(False, clarification=question.strip()[:300])

    try:
        mission = Mission.model_validate(data)
    except ValidationError as exc:
        errs = [_clean_error(e) for e in exc.errors()][:8]
        return ValidationResult(False, errors=errs)
    return ValidationResult(True, mission=mission)
