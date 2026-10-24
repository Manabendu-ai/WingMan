"""The mission contract shared by the ground LLM and the onboard executor.

The LLM may only choose from the fixed action menu below, and every number
has a hard limit it cannot exceed.
"""
import re
from typing import Annotated, List, Literal, Optional, Union

from pydantic import (BaseModel, ConfigDict, Field, field_validator,
                      model_validator)

# ---- hard limits (the LLM can never exceed these) ----
MAX_STEPS = 12
MAX_SPEED_MPS = 1.0
MAX_ALTITUDE_M = 3.0
MAX_MISSION_TIME_S = 900

TARGET_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 _-]{0,39}$")


class _Strict(BaseModel):
    # unknown fields are rejected, so the LLM cannot invent parameters
    model_config = ConfigDict(extra="forbid")


# ---- the action menu ----
class Takeoff(_Strict):
    action: Literal["takeoff"]
    altitude_m: float = Field(default=1.0, ge=0.5, le=MAX_ALTITUDE_M)


class Explore(_Strict):
    action: Literal["explore"]
    strategy: Literal["frontier", "wall_following"] = "frontier"
    until: Literal["target_found", "time_limit"] = "target_found"


class ApproachTarget(_Strict):
    action: Literal["approach_target"]
    standoff_m: float = Field(default=0.5, ge=0.3, le=3.0)


class Hover(_Strict):
    action: Literal["hover"]
    duration_s: float = Field(default=5.0, ge=1.0, le=60.0)


class ReturnToStart(_Strict):
    action: Literal["return_to_start"]


class Land(_Strict):
    action: Literal["land"]


Step = Annotated[
    Union[Takeoff, Explore, ApproachTarget, Hover, ReturnToStart, Land],
    Field(discriminator="action"),
]


class Limits(_Strict):
    max_speed_mps: float = Field(default=0.5, gt=0.0, le=MAX_SPEED_MPS)
    max_altitude_m: float = Field(default=2.0, ge=0.5, le=MAX_ALTITUDE_M)
    max_time_s: int = Field(default=300, ge=30, le=MAX_MISSION_TIME_S)


class Mission(_Strict):
    mission_id: str = Field(pattern=r"^[A-Za-z0-9_-]{1,40}$")
    goal: str = Field(min_length=1, max_length=200)
    # What the VLM should look for, e.g. "maze exit" or "red box"
    target: Optional[str] = None
    steps: List[Step] = Field(min_length=1, max_length=MAX_STEPS)
    limits: Limits = Field(default_factory=Limits)
    success_condition: str = Field(min_length=1, max_length=200)

    @field_validator("target")
    @classmethod
    def _target_ok(cls, v):
        if v is None:
            return v
        v = v.strip()
        # strict charset: this text ends up inside the VLM prompt
        if not TARGET_RE.match(v):
            raise ValueError(
                "target must be 1-40 chars: letters, digits, space, - or _")
        return v

    @model_validator(mode="after")
    def _check_sequence(self):
        actions = [s.action for s in self.steps]
        if actions[0] != "takeoff":
            raise ValueError("first step must be takeoff")
        if actions.count("takeoff") > 1:
            raise ValueError("only one takeoff is allowed")
        if actions[-1] not in ("hover", "land"):
            raise ValueError("last step must be hover or land")
        if "land" in actions[:-1]:
            raise ValueError("no steps are allowed after land")
        for s in self.steps:
            if s.action == "takeoff" and s.altitude_m > self.limits.max_altitude_m:
                raise ValueError(
                    "takeoff altitude exceeds limits.max_altitude_m")
            searching = (s.action == "approach_target"
                         or (s.action == "explore" and s.until == "target_found"))
            if searching and not self.target:
                raise ValueError(
                    "target is required when exploring until target_found "
                    "or approaching a target")
        return self

    def vlm_task(self) -> str:
        """LINK 1: the task string handed to the onboard VLM prompt."""
        if self.target:
            return f"find the {self.target}"
        return "explore safely and avoid obstacles"
