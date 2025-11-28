from __future__ import annotations

from dataclasses import dataclass, field
from typing_extensions import List, Dict, Any

from .degree_of_freedom import DegreeOfFreedom
from .world_entity import WorldEntity
from krrood.adapters.json_serializer import (
    SubclassJSONSerializer,
)


@dataclass(eq=False)
class Actuator(WorldEntity, SubclassJSONSerializer):
    """
    Represents an actuator in the world model.
    """

    _dofs: List[DegreeOfFreedom] = field(default_factory=list, init=False, repr=False)

    def to_json(self) -> Dict[str, Any]:
        result = super().to_json()
        result["name"] = self.name.to_json()
        result["dofs"] = [dof.to_json() for dof in self._dofs]
        return result

    @classmethod
    def _from_json(cls, data: Dict[str, Any], **kwargs) -> Actuator:
        actuator = cls(
            name=WorldEntity.name.from_json(data["name"]),
        )
        dofs_data = data.get("dofs", [])
        assert (
            len(dofs_data) > 0
        ), "An actuator must have at least one degree of freedom."
        for dof_data in dofs_data:
            dof = DegreeOfFreedom.from_json(dof_data)
            actuator.add_dof(dof)
        return actuator

    @property
    def dofs(self) -> List[DegreeOfFreedom]:
        """
        Returns the degrees of freedom associated with this actuator.
        """
        return self._dofs

    def add_dof(self, dof: DegreeOfFreedom) -> None:
        """
        Adds a degree of freedom to this actuator.

        :param dof: The degree of freedom to add.
        """
        self._dofs.append(dof)
