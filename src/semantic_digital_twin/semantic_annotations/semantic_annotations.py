from __future__ import annotations

from dataclasses import dataclass, field
from typing import Set, Iterable, Optional

import numpy as np
from krrood.entity_query_language.entity import an, entity, symbolic_mode, let
from probabilistic_model.probabilistic_circuit.rx.helper import uniform_measure_of_event
from typing_extensions import List

from .mixins import (
    HasBody,
    HasSupportingSurface,
    Furniture,
    HasRegion,
    HasDrawers,
    HasDoors,
)
from ..datastructures.variables import SpatialVariables
from ..reasoning.predicates import InsideOf
from ..spatial_types import Point3
from ..world_description.shape_collection import BoundingBoxCollection
from ..world_description.world_entity import (
    SemanticAnnotation,
    Body,
)


@dataclass(eq=False)
class IsPerceivable:
    """
    A mixin class for semantic annotations that can be perceived.
    """

    class_label: Optional[str] = field(default=None, kw_only=True)
    """
    The exact class label of the perceived object.
    """


@dataclass(eq=False)
class Handle(HasBody): ...


@dataclass(eq=False)
class Container(HasBody): ...


@dataclass(eq=False)
class Fridge(SemanticAnnotation):
    """
    A semantic annotation representing a fridge that has a door and a body.
    """

    container: Container
    door: Door


@dataclass(eq=False)
class Table(Furniture, HasBody):
    """
    A semantic annotation that represents a table.
    """

    def points_on_table(self, amount: int = 100) -> List[Point3]:
        """
        Get points that are on the table.

        :amount: The number of points to return.
        :returns: A list of points that are on the table.
        """
        area_of_table = BoundingBoxCollection.from_shapes(self.body.collision)
        event = area_of_table.event
        p = uniform_measure_of_event(event)
        p = p.marginal(SpatialVariables.xy)
        samples = p.sample(amount)
        z_coordinate = np.full(
            (amount, 1), max([b.max_z for b in area_of_table]) + 0.01
        )
        samples = np.concatenate((samples, z_coordinate), axis=1)
        return [Point3(*s, reference_frame=self.body) for s in samples]


@dataclass(eq=False)
class Aperture(HasRegion):
    """
    A semantic annotation that represents an opening in a physical entity.
    An example is like a hole in a wall that can be used to enter a room.
    """


@dataclass(eq=False)
class Door(HasBody):
    """
    A door is a physical entity that has covers an opening, has a movable body and a handle.
    """

    handle: Handle
    """
    The handle of the door.
    """


@dataclass(eq=False)
class DoubleDoor(SemanticAnnotation):
    left_door: Door
    right_door: Door


@dataclass(eq=False)
class Drawer(SemanticAnnotation):
    container: Container
    handle: Handle


############################### subclasses to Furniture
@dataclass(eq=False)
class Cabinet(Furniture, HasDrawers, HasDoors):
    container: Container


@dataclass(eq=False)
class Dresser(Furniture, HasDrawers, HasDoors):
    container: Container = field(kw_only=True)


@dataclass(eq=False)
class Cupboard(Furniture, HasDoors):
    container: Container = field(kw_only=True)


@dataclass(eq=False)
class Wardrobe(Furniture, HasDrawers, HasDoors):
    container: Container = field(kw_only=True)


class Floor(HasSupportingSurface): ...


@dataclass(eq=False)
class Room(SemanticAnnotation):
    """
    A semantic annotation that represents a closed area with a specific purpose
    """

    floor: Floor
    """
    The room's floor.
    """


@dataclass(eq=False)
class Wall(SemanticAnnotation):
    body: Body

    @property
    def doors(self) -> Iterable[Door]:
        with symbolic_mode():
            door = let(Door, self._world.semantic_annotations)
            query = an(entity(door), InsideOf(self.body, door.entry_way.region)() > 0.1)
        return query.evaluate()

# =======================
# === SUTURO Rody
# =======================

@dataclass(eq=False)
class Food(SemanticAnnotation):
    """
    A Semantic annotation representing a food item.
    """
    body: Body

@dataclass(eq=False)
class Drink(SemanticAnnotation):
    """
    A Semantic annotation representing a drink item.
    """
    body: Body

@dataclass(eq=False)
class Coffee(SemanticAnnotation):       #?????????????
    body: Body

@dataclass(eq=False)
class CleaningSupplies(SemanticAnnotation):
    body: Body

@dataclass(eq=False)
class TableWare(SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class KitchenSupplies(SemanticAnnotation):          # kein body
    ...

@dataclass(eq=False)
class DrinkingContainer(SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class DrinkingContainer(SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class OfficeSupplies(SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class CreditCard (SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class Ball(SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class Tool(SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class Toy (SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class Washer (SemanticAnnotation):
    ...

@dataclass(eq=False)
class Magazine (SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class Clothe (SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class Timer (SemanticAnnotation):
    body:Body


@dataclass(eq=False)
class Appliance(SemanticAnnotation):
    ...

@dataclass(eq=False)
class Components (SemanticAnnotation):
    ...

# %% Subclasses of Food
@dataclass(eq=False)
class Fruit(Food):
    ...
@dataclass(eq=False)
class Vegetable(Food):
    ...
@dataclass(eq=False)
class Snack(Food):
    ...

@dataclass(eq=False)
class DominoSugarBox(Food):
    ...

@dataclass(eq=False)
class CanFood(Food):        #
    ...

@dataclass(eq=False)
class Conduments(Food):
    ...

@dataclass(eq=False)
class CerealBox(Food):
    ...


# %% Subclasses of Fruit
@dataclass(eq=False)
class Banana(Fruit):
    ...

@dataclass(eq=False)
class Strawberry(Fruit):
    ...

@dataclass(eq=False)
class Apple(Fruit):
    ...

@dataclass(eq=False)
class Lemon(Fruit):
    ...

@dataclass(eq=False)
class Peach(Fruit):
    ...

@dataclass(eq=False)
class Pear(Fruit):
    ...

@dataclass(eq=False)
class Orange(Fruit):
    ...

@dataclass(eq=False)
class Plum(Fruit):
    ...


# %% Subclasses of Vegetable
@dataclass(eq=False)
class Cucumber(Vegetable):
    ...

@dataclass(eq=False)
class Carrot(Vegetable):
    ...


# %% Subclasses of Snack
@dataclass(eq=False)
class CkrackerBox(Snack):       # has instance
    ...

@dataclass(eq=False)
class JelloBox(Snack):       # has instance
    ...

@dataclass(eq=False)
class Chips(Snack):
    ...


# %% Subclasses of CanFood
@dataclass(eq=False)
class SpamPottedMeat(CanFood):
    ...

@dataclass(eq=False)
class StarkistTunaFish(CanFood):
    ...

@dataclass(eq=False)
class TomatoSoup(CanFood):
    ...


# %% Subclasses of Conduments
@dataclass(eq=False)
class Mustard(Conduments):
    ...

@dataclass(eq=False)
class Mayo(Conduments):
    ...

@dataclass(eq=False)
class Ketchep(Conduments):
    ...


# %% Subclasses of Drink
@dataclass(eq=False)
class GasDrinks(Drink):
    ...
@dataclass(eq=False)
class MilkDrinks(Drink):
    ...


# %% Subclasses of MilkDrinks
@dataclass(eq=False)
class Milk(MilkDrinks):
    ...
# %% Subclasses of GasDrink
@dataclass(eq=False)
class Cola(GasDrinks):
    ...

@dataclass(eq=False)
class Fanta(GasDrinks):
    ...


# %% Subclasses of CleaningSupplies
@dataclass(eq=False)
class WindexSrayBottle(CleaningSupplies):
    ...

@dataclass(eq=False)
class SrubCleanseBottle(CleaningSupplies):
    ...

@dataclass(eq=False)
class ScotchBriteDobieSponge(CleaningSupplies):
    ...


# %% Subclasses of KitchenSupplies
@dataclass(eq=False)
class PitcherBase(KitchenSupplies):     #aus KitchenSupplies ein body erben??
    body:Body

@dataclass(eq=False)
class PitcherLid(KitchenSupplies):     #aus KitchenSupplies ein body erben??
    body:Body

@dataclass(eq=False)
class Pitcher(KitchenSupplies):     #aus KitchenSupplies ein body erben??
    pitcher_base: PitcherBase
    pitcher_lid: PitcherLid

@dataclass(eq=False)
class Spatula(KitchenSupplies):     #aus KitchenSupplies ein body erben??
    body:Body

@dataclass(eq=False)
class Skillet(KitchenSupplies):     #aus KitchenSupplies ein body erben??
    body:Body

@dataclass(eq=False)
class SkilletLid(KitchenSupplies):     #aus KitchenSupplies ein body erben??
    body:Body


# %% Subclasses of TableWare
@dataclass(eq=False)
class Plate(TableWare):
    ...

@dataclass(eq=False)
class Bowl(TableWare):
    ...

@dataclass(eq=False)
class Cutlery(TableWare):
    ...

@dataclass(eq=False)
class TableCloth(TableWare):
    ...

@dataclass(eq=False)
class WineGlass(DrinkingContainer and TableWare):        # darf ich das???
    ...


# %% Subclasses of Cutlery
@dataclass(eq=False)
class Fork(Cutlery):
    ...

@dataclass(eq=False)
class Spoon(Cutlery):
    ...

@dataclass(eq=False)
class Knife(Cutlery):
    ...


# %% Subclasses of DrinkingContainer
@dataclass(eq=False)
class Mug(DrinkingContainer and TableWare):
    ...

@dataclass(eq=False)
class Bottle(DrinkingContainer):
    ...


# %% Subclasses of OfficeSupplies
@dataclass(eq=False)
class Scissors(OfficeSupplies):
    ...

@dataclass(eq=False)
class LargeMarker(OfficeSupplies):
    ...

@dataclass(eq=False)
class SmallMarker(OfficeSupplies):
    ...


# %% Subclasses of Tool
@dataclass(eq=False)
class Key(Tool):
    ...

@dataclass(eq=False)
class Padlock(Tool):
    ...

@dataclass(eq=False)
class Hammer(Tool):
    ...

@dataclass(eq=False)
class Nail(Tool):
    ...

@dataclass(eq=False)
class Bolt(Tool):
    ...

@dataclass(eq=False)
class Nut(Tool):
    ...

@dataclass(eq=False)
class Screwdriver(Tool):
    ...

@dataclass(eq=False)
class AdjustableWrench (Tool):
    ...

@dataclass(eq=False)
class WoodBlock (Tool):         # + colored Wood Blocks??? Nr. 65
    ...

@dataclass(eq=False)
class Clamps (Tool):
    ...

@dataclass(eq=False)
class PowerDrill (Tool):
    ...

@dataclass(eq=False)
class Rope (Tool):
    ...

@dataclass(eq=False)
class Chain (Tool):
    ...


# %% Subclasses of Screwdriver
@dataclass(eq=False)
class PhillipsScrewdriver (Screwdriver):
    ...

@dataclass(eq=False)
class FlatScrewdriver (Screwdriver):
    ...


# %% Subclasses of Ball
@dataclass(eq=False)
class MiniSoccerBall (Ball):
    ...

@dataclass(eq=False)
class SoftBall (Ball):
    ...

@dataclass(eq=False)
class Baseball (Ball):
    ...

@dataclass(eq=False)
class TennisBall (Ball):
    ...

@dataclass(eq=False)
class Racqueball (Ball):
    ...

@dataclass(eq=False)
class GolfBall (Ball):
    ...

@dataclass(eq=False)
class Marble (Ball):
    ...


# %% Subclasses of Toy
@dataclass(eq=False)
class MarblesBag (Toy):             # Tüte hat ein body erbt von toy, und jede marble von der List hat ein body
    marbles: List[Marble] = field(default_factory=list)

@dataclass(eq=False)
class Cup (Toy):
    ...

@dataclass(eq=False)
class FoamBrick (Toy):
    ...

@dataclass(eq=False)
class Dice (Toy):
    ...

@dataclass(eq=False)
class RubicksCube (Toy):
    ...

@dataclass(eq=False)
class PegHoleTest (Toy):
    ...

@dataclass(eq=False)
class ToyAirplane (Toy):
    ...

@dataclass(eq=False)
class Lego (Toy):
    ...


# %% Subclasses of Container
@dataclass(eq=False)
class ClearBox (Container):
    ...


# %% Subclasses of Components
@dataclass(eq=False)
class BoxLid (Components):
    body:Body

@dataclass(eq=False)
class SofaBase(Components):
    body:Body

@dataclass(eq=False)
class SofaBackRest(Components):
    body:Body

@dataclass(eq=False)
class SofaArmRest(Components):
    body:Body

@dataclass(eq=False)
class HotPlate(Components):
    body:Body

# %% Subclasses of Clothe
@dataclass(eq=False)
class TShirt(Clothe):
    ...


# %% Subclasses of furniture
@dataclass(eq=False)
class Sofa(Furniture):
    base: SofaBase
    back_rest: SofaBackRest
    arm_rest: List[SofaArmRest] = field(default_factory=list, hash=False)


# %% Subclasses of Appliance
@dataclass(eq=False)
class Cooktop(Appliance):
    body:Body
    hotplate: List[HotPlate] = field(default_factory=list, hash=False)

@dataclass(unsafe_hash=True)
class Oven(Appliance):
    """
    An oven appliance with a body
    """
    body: Body