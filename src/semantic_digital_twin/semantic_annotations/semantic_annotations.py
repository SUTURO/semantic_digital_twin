from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from probabilistic_model.probabilistic_circuit.rx.helper import uniform_measure_of_event
from typing_extensions import List

from ..world_description.geometry import Sphere
from ..world_description.shape_collection import BoundingBoxCollection
from ..spatial_types import Point3
from ..datastructures.variables import SpatialVariables
from ..world_description.world_entity import SemanticAnnotation, Body, Region


@dataclass(eq=False)
class HasDrawers:
    """
    A mixin class for semantic annotations that have drawers.
    """

    drawers: List[Drawer] = field(default_factory=list, hash=False)


@dataclass(eq=False)
class HasDoors:
    """
    A mixin class for semantic annotations that have doors.
    """

    doors: List[Door] = field(default_factory=list, hash=False)


@dataclass(eq=False)
class Handle(SemanticAnnotation):
    body: Body


@dataclass(eq=False)
class Container(SemanticAnnotation):
    body: Body


@dataclass(eq=False)
class Fridge(SemanticAnnotation):
    """
    A semantic annotation representing a fridge that has a door and a body.
    """

    body: Body
    door: Door


@dataclass(eq=False)
class Table(SemanticAnnotation):
    """
    A semantic annotation that represents a table.
    """

    top: Body
    """
    The body that represents the table's top surface.
    """

    def points_on_table(self, amount: int = 100) -> List[Point3]:
        """
        Get points that are on the table.

        :amount: The number of points to return.
        :returns: A list of points that are on the table.
        """
        area_of_table = BoundingBoxCollection.from_shapes(self.top.collision)
        event = area_of_table.event
        p = uniform_measure_of_event(event)
        p = p.marginal(SpatialVariables.xy)
        samples = p.sample(amount)
        z_coordinate = np.full(
            (amount, 1), max([b.max_z for b in area_of_table]) + 0.01
        )
        samples = np.concatenate((samples, z_coordinate), axis=1)
        return [Point3(*s, reference_frame=self.top) for s in samples]


################################


@dataclass(eq=False)
class Components(SemanticAnnotation): ...


@dataclass(eq=False)
class Furniture(SemanticAnnotation): ...


@dataclass(eq=False)
class SupportingSurface(SemanticAnnotation):
    """
    A semantic annotation that represents a supporting surface.
    """

    region: Region
    """
    The region that represents the supporting surface.
    """


#################### subclasses von Components


@dataclass(eq=False)
class EntryWay(Components):
    body: Body


@dataclass(eq=False)
class Door(EntryWay):
    handle: Handle


@dataclass(eq=False)
class DoubleDoor(EntryWay):
    left_door: Door
    right_door: Door


@dataclass(eq=False)
class Drawer(Components):
    container: Container
    handle: Handle


############################### subclasses to Furniture
@dataclass(eq=False)
class Cabinet(Furniture):
    container: Container
    drawers: List[Drawer] = field(default_factory=list, hash=False)
    doors: List[Door] = field(default_factory=list)


@dataclass(eq=False)
class Dresser(Furniture):
    container: Container
    drawers: List[Drawer] = field(default_factory=list, hash=False)
    doors: List[Door] = field(default_factory=list)


@dataclass(eq=False)
class Cupboard(Furniture):
    container: Container
    doors: List[Door] = field(default_factory=list)


@dataclass(eq=False)
class Wardrobe(Furniture):
    container: Container
    drawers: List[Drawer] = field(default_factory=list, hash=False)
    doors: List[Door] = field(default_factory=list)


class Floor(SupportingSurface): ...


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
    doors: List[Door] = field(default_factory=list)

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
class Fruit(Food):
    ...
@dataclass(eq=False)
class Vegetable(Food):
    ...
@dataclass(eq=False)
class Snack(Food):
    ...

@dataclass(eq=False)
class GasDrinks(Drink):
    ...
@dataclass(eq=False)
class MilkDrinks(Drink):
    ...


@dataclass(eq=False)
class CkrackerBox(Snack):       # has instance
    ...

@dataclass(eq=False)
class DominoSugarBox(Food):
    ...

@dataclass(eq=False)
class JelloBox(Snack):       # has instance
    ...

@dataclass(eq=False)
class CanFood(Food):        #
    ...

@dataclass(eq=False)
class SpamPottedMeat(CanFood):
    ...

@dataclass(eq=False)
class StarkistTunaFish(CanFood):
    ...

@dataclass(eq=False)
class TomatoSoup(CanFood):
    ...

@dataclass(eq=False)
class Chips(Snack):
    ...

@dataclass(eq=False)
class Conduments(Food):
    ...

@dataclass(eq=False)
class Mustard(Conduments):
    ...

@dataclass(eq=False)
class Mayo(Conduments):
    ...

@dataclass(eq=False)
class Ketchep(Conduments):
    ...



@dataclass(eq=False)
class Coffee(SemanticAnnotation):       #?????????????
    body: Body

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

@dataclass(eq=False)
class Cucumber(Vegetable):
    ...

@dataclass(eq=False)
class Carrot(Vegetable):
    ...

@dataclass(eq=False)
class CleaningSupplies(SemanticAnnotation):
    body: Body

@dataclass(eq=False)
class WindexSrayBottle(CleaningSupplies):
    ...

@dataclass(eq=False)
class SrubCleanseBottle(CleaningSupplies):
    ...

@dataclass(eq=False)
class ScotchBriteDobieSponge(CleaningSupplies):
    ...

@dataclass(eq=False)
class KitchenSupplies(SemanticAnnotation):          # kein body
    ...

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
class TableWare(SemanticAnnotation):
    body:Body

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
class Fork(Cutlery):
    ...

@dataclass(eq=False)
class Spoon(Cutlery):
    ...

@dataclass(eq=False)
class Knife(Cutlery):
    ...

@dataclass(eq=False)
class Spatula(KitchenSupplies):     #aus KitchenSupplies ein body erben??
    body:Body

@dataclass(eq=False)
class DrinkingContainer(SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class WineGlass(DrinkingContainer or TableWare):        # darf ich das???
    ...

@dataclass(eq=False)
class Mug(DrinkingContainer):
    ...

@dataclass(eq=False)
class Bottle(DrinkingContainer):
    ...

@dataclass(eq=False)
class Skillet(KitchenSupplies):     #aus KitchenSupplies ein body erben??
    body:Body

@dataclass(eq=False)
class SkilletLid(KitchenSupplies):     #aus KitchenSupplies ein body erben??
    body:Body

@dataclass(eq=False)
class TableCloth(TableWare):
    ...

@dataclass(eq=False)
class OfficeSupplies(SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class Scissors(OfficeSupplies):
    ...

@dataclass(eq=False)
class LargeMarker(OfficeSupplies):
    ...

@dataclass(eq=False)
class SmallMarker(OfficeSupplies):
    ...

@dataclass(eq=False)
class Tool(SemanticAnnotation):
    body:Body

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
class PhillipsScrewdriver (Screwdriver):
    ...

@dataclass(eq=False)
class FlatScrewdriver (Screwdriver):
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
class CreditCard (SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class Ball(SemanticAnnotation):
    body:Body

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

@dataclass(eq=False)
class Toy (SemanticAnnotation):
    body:Body

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
class Washer (SemanticAnnotation):
    ...

@dataclass(eq=False)
class Rope (Tool):
    ...

@dataclass(eq=False)
class Chain (Tool):
    ...

@dataclass(eq=False)
class RubicksCube (Toy):
    ...

@dataclass(eq=False)
class ClearBox (Container):
    ...

@dataclass(eq=False)
class BoxLid (Components):
    body:Body

@dataclass(eq=False)
class PegHoleTest (Toy):
    ...

@dataclass(eq=False)
class ToyAirplane (Toy):
    ...

@dataclass(eq=False)
class Lego (Toy):
    ...

@dataclass(eq=False)
class Magazine (SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class Clothe (SemanticAnnotation):
    body:Body

@dataclass(eq=False)
class TShirt(Clothe):
    ...

@dataclass(eq=False)
class Timer (SemanticAnnotation):
    body:Body


#Nr. 59 .......

@dataclass(eq=False)
class Cola(GasDrinks):
    ...

@dataclass(eq=False)
class Fanta(GasDrinks):
    ...

@dataclass(eq=False)
class Milk(MilkDrinks):
    ...

@dataclass(eq=False)
class CerealBox(Food):
    ...
