from typing import List, Literal, Optional
from pydantic import BaseModel, Field, model_validator


class Observation(BaseModel):
    scene: str
    target_visible: bool
    # Relative image coordinates 0-1000 (x: left->right, y: top->bottom)
    target_x: Optional[int] = Field(default=None, ge=0, le=1000)
    target_y: Optional[int] = Field(default=None, ge=0, le=1000)
    recommended_direction: Literal["forward", "left", "right", "back", "none"]
    confidence: float = Field(ge=0.0, le=1.0)
    uncertainties: List[str] = []
    suggested_next_step: Literal[
        "continue", "search_again", "approach_target", "uncertain"
    ]

    @model_validator(mode="after")
    def _check_consistency(self):
        if self.target_visible:
            if self.target_x is None or self.target_y is None:
                raise ValueError("target_visible=true but no coordinates")
        else:
            self.target_x = None
            self.target_y = None
        return self


def fallback_observation(reason: str) -> Observation:
    """Safe 'I don't know' answer used when the VLM fails."""
    return Observation(
        scene="unknown",
        target_visible=False,
        recommended_direction="none",
        confidence=0.0,
        uncertainties=[reason],
        suggested_next_step="uncertain",
    )
