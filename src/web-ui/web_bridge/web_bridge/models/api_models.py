"""
Pydantic models for web bridge API.

Contains request/response models for REST API endpoints.
"""

from typing import List, Dict, Any, Optional
from pydantic import BaseModel


class ActionGoalField(BaseModel):
    """Model for action goal field definition."""

    name: str
    type: str
    description: str = ""


class ActionInfo(BaseModel):
    """Model for action information."""

    name: str
    type: str
    description: str
    goal: List[ActionGoalField]
    available: bool
    namespace: str = ""


class ActionListResponse(BaseModel):
    """Response model for action listing."""

    actions: List[ActionInfo]
    total_count: int


class ActionSendGoalRequest(BaseModel):
    """Request model for sending action goals."""

    parameters: Dict[str, Any]


class ActionSendGoalResponse(BaseModel):
    """Response model for action goal sending."""

    success: bool
    message: str
    goal_id: Optional[str] = None
    accepted: Optional[bool] = None
    rejection_reason: Optional[str] = None


class ActionCancelGoalResponse(BaseModel):
    """Response model for action goal cancellation."""

    success: bool
    message: str
    goal_id: str
