from typing import List, Optional

from pydantic import BaseModel, Field


class RobotVisConfig(BaseModel):
    urdf_path: str
    mesh_path: Optional[str] = None
    model_scale: float = 1.0
    initial_rotation_euler: List[float] = Field(default_factory=lambda: [0.0, 0.0, 0.0])
