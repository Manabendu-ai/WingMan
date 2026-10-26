"""Ask the LLM for a corrected mission after a failure."""
import json
from typing import Any, Dict, Optional

from .mission import Mission
from .planner import PlanResult, plan_mission
from .report import summarize_status


def replan_mission(request: str, failed: Mission, status: Dict[str, Any], llm,
                   max_attempts: int = 3,
                   mission_id: Optional[str] = None) -> PlanResult:
    context = json.dumps({"previous_mission": failed.model_dump(),
                          "outcome": summarize_status(status)}, indent=1)
    return plan_mission(request, llm, max_attempts=max_attempts,
                        mission_id=mission_id, context=context)
