from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix, RotationMatrix
from semantic_digital_twin.utils import get_semantic_digital_twin_directory_root
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.world_entity import Body
import os
from semantic_digital_twin.spatial_types import Point3, Vector3
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.world_description.geometry import Box, Scale, Sphere, Cylinder, FileMesh, Color

#visulise:
import logging
import os

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.utils import get_semantic_digital_twin_directory_root

from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
import threading
import rclpy

import logging
import math
import os

from semantic_digital_twin.adapters.urdf import URDFParser
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix
from semantic_digital_twin.utils import get_semantic_digital_twin_directory_root
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.connections import Connection6DoF, FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale, Color
from semantic_digital_twin.world_description.shape_collection import ShapeCollection
from semantic_digital_twin.spatial_computations.raytracer import RayTracer


def loading_environment():
    world = World()

    # Collers:
    white = Color(1, 1, 1)
    red = Color(1, 0, 0)
    black = Color(0, 0, 0)
    gray = Color(0.74, 0.74, 0.74)
    wood = Color(1, 0.827, 0.6078)

    # =======================
    # === Bodies & Connections
    # =======================
    root_origin = TransformationMatrix()
    root = Body(name=PrefixedName("root"))

    # All the Walls:
    sWall1 = Box(scale=Scale(0.05, 1.00, 3.00), color=gray)
    visual = ShapeCollection([sWall1])
    collision = ShapeCollection([sWall1])
    sWall1_body = Body(name=PrefixedName("sWall1_body"), collision=collision, visual=visual)

    root_C_sWall1 = FixedConnection(parent=root, child=sWall1_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=0, y=-2.01,
                                                                                                     z=1.50))

    sWall2 = Box(scale=Scale(0.29, 0.05, 3.00), color=gray)
    visual = ShapeCollection([sWall2])
    collision = ShapeCollection([sWall2])
    sWall2_body = Body(name=PrefixedName("sWall2_body"), collision=collision, visual=visual)

    root_C_sWall2 = FixedConnection(parent=root, child=sWall2_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.145, y=-1.45,
                                                                                                     z=1.50))

    sWall3 = Box(scale=Scale(0.05, 1.085, 1.00), color=gray)
    visual = ShapeCollection([sWall3])
    collision = ShapeCollection([sWall3])
    sWall3_body = Body(name=PrefixedName("sWall3_body"), collision=collision, visual=visual)

    root_C_sWall3 = FixedConnection(parent=root, child=sWall3_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.29, y=-0.9925,
                                                                                                     z=0.5))

    sWall4 = Box(scale=Scale(0.29, 0.05, 1.00), color=gray)
    visual = ShapeCollection([sWall4])
    collision = ShapeCollection([sWall4])
    sWall4_body = Body(name=PrefixedName("sWall4_body"), collision=collision, visual=visual)

    root_C_sWall4 = FixedConnection(parent=root, child=sWall4_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.145, y=-0.45,
                                                                                                     z=0.5))

    sWall5 = Box(scale=Scale(0.29, 0.05, 1.00), color=gray)
    visual = ShapeCollection([sWall5])
    collision = ShapeCollection([sWall5])
    sWall5_body = Body(name=PrefixedName("sWall5_body"), collision=collision, visual=visual)

    root_C_sWall5 = FixedConnection(parent=root, child=sWall5_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.145, y=0.45,
                                                                                                     z=0.5))

    sWall6 = Box(scale=Scale(0.05, 2.75, 1.00), color=gray)
    visual = ShapeCollection([sWall6])
    collision = ShapeCollection([sWall6])
    sWall6_body = Body(name=PrefixedName("sWall6_body"), collision=collision, visual=visual)

    root_C_sWall6 = FixedConnection(parent=root, child=sWall6_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.29025, y=1.80,
                                                                                                     z=0.5))


    sWall7 = Box(scale=Scale(0.05, 2.27, 1.00), color=gray)
    visual = ShapeCollection([sWall7])
    collision = ShapeCollection([sWall7])
    sWall7_body = Body(name=PrefixedName("sWall7_body"), collision=collision, visual=visual)

    root_C_sWall7 = FixedConnection(parent=root, child=sWall7_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.29025, y=5.16,
                                                                                                     z=0.5))

    eWall = Box(scale=Scale(4.924, 0.05, 3.00), color=gray)
    visual = ShapeCollection([eWall])
    collision = ShapeCollection([eWall])
    eWall_body = Body(name=PrefixedName("eWall_body"), collision=collision, visual=visual)

    root_C_eWall = FixedConnection(parent=root, child=eWall_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=2.462, y=-2.535,
                                                                                                    z=1.50))

    mWall = Box(scale=Scale(0.05, 2.67, 1.00), color=gray)
    visual = ShapeCollection([mWall])
    collision = ShapeCollection([mWall])
    mWall_body = Body(name=PrefixedName("mWall_body"), collision=collision, visual=visual)

    root_C_mWall = FixedConnection(parent=root, child=mWall_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=2.20975, y=5.00,      # 5.13
                                                                                                    z=0.50))  # 2.13, 3.81, 0.50

    wWall = Box(scale=Scale(4.449, 0.05, 3.00), color=gray)
    visual = ShapeCollection([wWall])
    collision = ShapeCollection([wWall])
    wWall_body = Body(name=PrefixedName("wWall_body"), collision=collision, visual=visual)

    root_C_wWall = FixedConnection(parent=root, child=wWall_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=1.9345, y=6.32,
                                                                                                    z=1.50))


    nWAll = Box(scale=Scale(0.05, 8.04, 3.00), color=gray)
    visual = ShapeCollection([nWAll])
    collision = ShapeCollection([nWAll])
    nWall_body = Body(name=PrefixedName("nWAll_body"), collision=collision, visual=visual)

    root_C_nWall = FixedConnection(parent=root, child=nWall_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=4.949, y=1.51,
                                                                                                    z=1.50))

    nwWall = Cylinder(width=1.53, height=3.00, color=gray)
    visual = ShapeCollection([nwWall])
    collision = ShapeCollection([nwWall])
    nwWall_body = Body(name=PrefixedName("nwWall_body"), collision=collision, visual=visual)

    root_C_nwWall = FixedConnection(parent=root, child=nwWall_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=4.924, y=6.295,
                                                                                                    z=1.50))

    # The Rest:

    refrigerator = Box(scale=Scale(0.60, 0.658, 1.49), color=white)
    visual = ShapeCollection([refrigerator])
    collision = ShapeCollection([refrigerator])
    refrigerator_body = Body(name=PrefixedName("refrigerator_body"), collision=collision, visual=visual)

    root_C_fridge = FixedConnection(parent=root, child=refrigerator_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=0.537, y=-2.181,
                                                                                                     z=0.745))

    counterTop = Box(scale=Scale(2.044, 0.658, 0.545), color=wood)
    visual = ShapeCollection([counterTop])
    collision = ShapeCollection([counterTop])
    counterTop_body = Body(name=PrefixedName("counterTop_body"), collision=collision, visual=visual)

    root_C_counterTop = FixedConnection(parent=root, child=counterTop_body,
                                        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=1.859,
                                                                                                         y=-2.181,
                                                                                                         z=0.2725))

    ovenArea = Box(scale=Scale(1.20, 0.658, 1.49), color=white)
    visual = ShapeCollection([ovenArea])
    collision = ShapeCollection([ovenArea])
    ovenArea_body = Body(name=PrefixedName("ovenArea_body"), collision=collision, visual=visual)

    root_C_ovenArea = FixedConnection(parent=root, child=ovenArea_body,
                                      parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=3.481,
                                                                                                       y=-2.181,
                                                                                                       z=0.745))

    table = Box(scale=Scale(2.45, 0.796, 0.845), color=white)
    visual = ShapeCollection([table])
    collision = ShapeCollection([table])
    table_body = Body(name=PrefixedName("table_body"), collision=collision, visual=visual)

    root_C_table = FixedConnection(parent=root, child=table_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=3.545, y=0.426,
                                                                                                    z=0.4225))

    sofa = Box(scale=Scale(1.68, 0.94, 0.68), color=wood)
    visual = ShapeCollection([sofa])
    collision = ShapeCollection([sofa])
    sofa_body = Body(name=PrefixedName("sofa_body"), collision=collision, visual=visual)

    root_C_sofa = FixedConnection(parent=root, child=sofa_body,
                                  parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=3.60, y=1.20,
                                                                                                   z=0.34))

    lowerTable = Box(scale=Scale(0.37, 0.91, 0.44), color=white)
    visual = ShapeCollection([lowerTable])
    collision = ShapeCollection([lowerTable])
    lowerTable_body = Body(name=PrefixedName("lowerTable_body"), collision=collision, visual=visual)

    root_C_lowerTable = FixedConnection(parent=root, child=lowerTable_body,
                                        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=4.22, y=2.22,
                                                                                                         z=0.22))

    cabinet = Box(scale=Scale(0.43, 0.80, 2.02), color=white)
    visual = ShapeCollection([cabinet])
    collision = ShapeCollection([cabinet])
    cabinet_body = Body(name=PrefixedName("cabinet_body"), collision=collision, visual=visual)

    root_C_cabinet = FixedConnection(parent=root, child=cabinet_body,
                                     parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=4.65, y=4.72,
                                                                                                      z=1.01))

    desk = Box(scale=Scale(0.60, 1.20, 0.75), color=white)
    visual = ShapeCollection([desk])
    collision = ShapeCollection([desk])
    desk_body = Body(name=PrefixedName("desk_body"), collision=collision, visual=visual)

    root_C_desk = FixedConnection(parent=root, child=desk_body,
                                  parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=0.05, y=1.48,
                                                                                                   z=0.375))

    cookingTable = Box(scale=Scale(1.75, 0.64, 0.71),color=wood)
    visual = ShapeCollection([cookingTable])
    collision = ShapeCollection([cookingTable])
    cookingTable_body = Body(name=PrefixedName("cookingTable_body"), collision=collision, visual=visual)

    root_C_cookingTable = FixedConnection(parent=root,child=cookingTable_body,
                                  parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=1.325, y=5.675, z=0.355))


    diningTable = Box(scale=Scale(0.73, 1.18, 0.73),color=wood)
    visual = ShapeCollection([diningTable])
    collision = ShapeCollection([diningTable])
    diningTable_body = Body(name=PrefixedName("diningTable_body"), collision=collision, visual=visual)

    root_C_diningTable = FixedConnection(parent=root,child=diningTable_body,
                                         parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=2.59975, y=5.705, z=0.365))

    with world.modify_world():
        world.add_body(root)

        world.add_connection(root_C_sWall1)
        world.add_body(sWall1_body)
        world.add_connection(root_C_sWall2)
        world.add_body(sWall2_body)
        world.add_connection(root_C_sWall3)
        world.add_body(sWall3_body)
        world.add_connection(root_C_sWall4)
        world.add_body(sWall4_body)
        world.add_connection(root_C_sWall5)
        world.add_body(sWall5_body)
        world.add_body(sWall6_body)
        world.add_connection(root_C_sWall6)
        world.add_connection(root_C_sWall7)
        world.add_body(sWall7_body)

        world.add_connection(root_C_eWall)
        world.add_body(eWall_body)

        world.add_connection(root_C_nWall)
        world.add_body(nWall_body)

        world.add_connection(root_C_nwWall)
        world.add_body(nwWall_body)

        world.add_connection(root_C_wWall)
        world.add_body(wWall_body)

        world.add_connection(root_C_mWall)
        world.add_body(mWall_body)

        world.add_connection(root_C_fridge)
        world.add_body(refrigerator_body)
        world.add_body(counterTop_body)
        world.add_connection(root_C_counterTop)
        world.add_body(ovenArea_body)
        world.add_connection(root_C_ovenArea)
        world.add_connection(root_C_table)
        world.add_body(table_body)
        world.add_connection(root_C_sofa)
        world.add_body(sofa_body)
        world.add_connection(root_C_lowerTable)
        world.add_body(lowerTable_body)
        world.add_connection(root_C_cabinet)
        world.add_body(cabinet_body)
        world.add_connection(root_C_desk)
        world.add_body(desk_body)
        world.add_connection(root_C_cookingTable)
        world.add_body(cookingTable_body)
        world.add_connection(root_C_diningTable)
        world.add_body(diningTable_body)

        return world



def published(world: World):
    rclpy.init()
    node = rclpy.create_node("semantic_digital_twin")
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    viz = VizMarkerPublisher(world=world, node=node)

world = loading_environment()
published(world)
