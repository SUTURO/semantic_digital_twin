---
jupytext:
  text_representation:
    extension: .md
    format_name: myst
    format_version: 0.13
    jupytext_version: 1.16.4
kernelspec:
  display_name: Python 3
  language: python
  name: python3
---

(semantic_annotations)=
# Semantic Annotations

Semantic annotations can be used to say that a certain body should be interpreted as a handle or that a combination of
bodies should be interpreted as a drawer.
Ontologies inspire semantic annotations. The semantic digital twin overcomes the technical limitations of ontologies by representing
semantic annotations as Python classes and by using Python's typing together with the Entity Query Language (EQL) for reasoning.
This tutorial shows you how to apply semantic annotations to a world and how to create your own semantic_annotations.

Used Concepts:
- [](creating-custom-bodies)
- [](world-structure-manipulation)
- [Entity Query Language](https://abdelrhmanbassiouny.github.io/entity_query_language/intro.html)

First, let's create a simple world that contains a couple of apples.

```{code-cell} ipython3
from dataclasses import dataclass
from typing import List

from entity_query_language import entity, an, let, symbolic_mode

from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix
from semantic_digital_twin.semantic_annotations.semantic_annotations import Container
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.connections import Connection6DoF
from semantic_digital_twin.world_description.geometry import Sphere, Box, Scale
from semantic_digital_twin.world_description.world_entity import SemanticAnnotation, Body
from semantic_digital_twin.spatial_computations.raytracer import RayTracer


@dataclass
class Apple(SemanticAnnotation):
    """A simple custom semantic_annotation declaring that a Body is an Apple."""

    body: Body

```

Semantic annotations are world entities, so it needs to have a unique ´PrefixedName`. You can either provide it directly
when creating the semantic annotation or let semantic_digital_twin generate a unique name for you. 

```{code-cell} ipython3
world = World()
with world.modify_world():
    root = Body(name=PrefixedName("root"))

    # Our first apple
    apple_body = Body(name=PrefixedName("apple_body"))
    sphere = Sphere(radius=0.15, origin=TransformationMatrix(reference_frame=apple_body))
    apple_body.collision = [sphere]
    apple_body.visual = [sphere]

    world.add_connection(Connection6DoF(parent=root, child=apple_body, _world=world))
    world.add_semantic_annotation(Apple(body=apple_body, name=PrefixedName("apple1")))

    # Our second apple
    apple_body_2 = Body(name=PrefixedName("apple_body_2"))
    sphere2 = Sphere(radius=0.15, origin=TransformationMatrix(reference_frame=apple_body_2))
    apple_body_2.collision = [sphere2]
    apple_body_2.visual = [sphere2]
    c2 = Connection6DoF(parent=root, child=apple_body_2, _world=world)
    world.add_connection(c2)
    # Move it a bit so we can see both
    world.state[c2.x.name].position = 0.3
    world.state[c2.y.name].position = 0.2
    world.add_semantic_annotation(Apple(body=apple_body_2, name=PrefixedName("apple2")))

print(world.get_semantic_annotations_by_type(Apple))
rt = RayTracer(world)
rt.update_scene()
rt.scene.show("jupyter")
```

Thanks to the semantic annotations, an agent can query for apples directly using EQL:

```{code-cell} ipython2
with symbolic_mode():
    apples = an(entity(let(Apple, world.semantic_annotations)))
print(*apples.evaluate(), sep="\n")
```

SemanticAnnotations can become arbitrarily expressive. For instance, we can define a FruitBox that groups a container and a list of apples.

```{code-cell} ipython2
from semantic_digital_twin.semantic_annotations.factories import ContainerFactory, Direction

@dataclass
class FruitBox(SemanticAnnotation):
    box: Container
    fruits: List[Apple]

with world.modify_world():
    # To create a hollowed out box in this case we use a "ContainerFactory". 
    # To learn more about how cool SemanticAnnotationFactories are, please visit the appropriate guide!
    fruit_box_container_world = ContainerFactory(
        name=PrefixedName("fruit_box_container"), direction=Direction.Z, scale=Scale(1.0, 1.0, 0.3)
    ).create()
    world.merge_world_at_pose(
        fruit_box_container_world,
        TransformationMatrix.from_xyz_rpy(x=0.3),
    )

fruit_box_container_semantic_annotation = world.get_semantic_annotations_by_type(Container)[0]
fruit_box_with_apples = FruitBox(box=fruit_box_container_semantic_annotation, fruits=world.get_semantic_annotations_by_type(Apple))
world.add_semantic_annotation(fruit_box_with_apples)
print(f"Fruit box with {len(fruit_box_with_apples.fruits)} fruits")
rt = RayTracer(world)
rt.update_scene()
rt.scene.show("jupyter")
```

Because these are plain Python classes, any other agent that imports your semantic_annotation definitions will understand exactly what
you mean. Interoperability comes for free without hidden formats or conversion issues.

---

We can incorporate the attributes of our SemanticAnnotations into our reasoning.
To demonstrate this, let's first create another FruitBox, but which is empty this time.

```{code-cell} ipython2
with world.modify_world():
    empty_fruit_box_container_world = ContainerFactory(
        name=PrefixedName("empty_fruit_box_container"), direction=Direction.Z, scale=Scale(1.0, 1.0, 0.3)
    ).create()
    world.merge_world_at_pose(
        empty_fruit_box_container_world,
        TransformationMatrix.from_xyz_rpy(x=-1),
    )

empty_fruit_box_container_semantic_annotation = world.get_semantic_annotation_by_name("empty_fruit_box_container")
assert isinstance(empty_fruit_box_container_semantic_annotation, Container)
empty_fruit_box = FruitBox(box=empty_fruit_box_container_semantic_annotation, fruits=[])
world.add_semantic_annotation(empty_fruit_box)

rt = RayTracer(world)
rt.update_scene()
rt.scene.show("jupyter")
```

We can now use EQL to get us only the FruitBoxes that actually contain apples!

```{code-cell} ipython2
from semantic_digital_twin.reasoning.predicates import ContainsType
from entity_query_language import a

with symbolic_mode():
    fruit_box_query = a(fb := FruitBox(), ContainsType(fb.fruits, Apple))

query_result = fruit_box_query.evaluate()
print(list(query_result)[0] == fruit_box_with_apples)
```