from decorator import append
from semantic_digital_twin.world import World
from semantic_digital_twin.world_description.geometry import Box, Scale, Sphere, Cylinder, FileMesh, Color
from semantic_digital_twin.adapters.viz_marker import VizMarkerPublisher
import threading
import rclpy
from semantic_digital_twin.datastructures.prefixed_name import PrefixedName
from semantic_digital_twin.spatial_types.spatial_types import TransformationMatrix
from semantic_digital_twin.world_description.world_entity import Body
from semantic_digital_twin.world_description.connections import Connection6DoF, FixedConnection
from semantic_digital_twin.world_description.geometry import Box, Scale, Color
from semantic_digital_twin.world_description.shape_collection import ShapeCollection


white = Color(1, 1, 1)
red = Color(1, 0, 0)
black = Color(0, 0, 0)
gray = Color(0.74, 0.74, 0.74)
wood = Color(1, 0.827, 0.6078)

def loading_environment_walls():
    wall_world = World()
    all_wall_bodies = []
    all_wall_connections = []

    root = Body(name=PrefixedName("root"))
    all_wall_bodies.append(root)

    south_wall1 = Box(scale=Scale(0.05, 1.00, 3.00), color=gray)
    shape_geometry = ShapeCollection([south_wall1])
    south_wall1_body = Body(name=PrefixedName("south_wall1_body"), collision=shape_geometry, visual=shape_geometry)
    all_wall_bodies.append(south_wall1_body)

    root_C_south_wall1 = FixedConnection(parent=root, child=south_wall1_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(y=-2.01, z=1.50))
    all_wall_connections.append(root_C_south_wall1)

    south_wall2 = Box(scale=Scale(0.29, 0.05, 3.00), color=gray)
    shape_geometry = ShapeCollection([south_wall2])
    south_wall2_body = Body(name=PrefixedName("south_wall2_body"), collision=shape_geometry, visual=shape_geometry)
    all_wall_bodies.append(south_wall2_body)

    root_C_south_wall2 = FixedConnection(parent=root, child=south_wall2_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.145, y=-1.45, z=1.50))
    all_wall_connections.append(root_C_south_wall2)

    south_wall3 = Box(scale=Scale(0.05, 1.085, 1.00), color=gray)
    shape_geometry = ShapeCollection([south_wall3])
    south_wall3_body = Body(name=PrefixedName("south_wall3_body"), collision=shape_geometry, visual=shape_geometry)
    all_wall_bodies.append(south_wall3_body)

    root_C_south_wall3 = FixedConnection(parent=root, child=south_wall3_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.29, y=-0.9925, z=0.5))
    all_wall_connections.append(root_C_south_wall3)

    south_wall4 = Box(scale=Scale(0.29, 0.05, 1.00), color=gray)
    shape_geometry = ShapeCollection([south_wall4])
    south_wall4_body = Body(name=PrefixedName("south_wall4_body"), collision=shape_geometry, visual=shape_geometry)
    all_wall_bodies.append(south_wall4_body)

    root_C_south_wall4 = FixedConnection(parent=root, child=south_wall4_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.145, y=-0.45, z=0.5))
    all_wall_connections.append(root_C_south_wall4)

    south_wall5 = Box(scale=Scale(0.29, 0.05, 1.00), color=gray)
    shape_geometry = ShapeCollection([south_wall5])
    south_wall5_body = Body(name=PrefixedName("south_wall5_body"), collision=shape_geometry, visual=shape_geometry)
    all_wall_bodies.append(south_wall5_body)

    root_C_south_wall5 = FixedConnection(parent=root, child=south_wall5_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.145, y=0.45, z=0.5))
    all_wall_connections.append(root_C_south_wall5)

    south_wall6 = Box(scale=Scale(0.05, 2.75, 1.00), color=gray)
    shape_geometry = ShapeCollection([south_wall6])
    south_wall6_body = Body(name=PrefixedName("south_wall6_body"), collision=shape_geometry, visual=shape_geometry)
    all_wall_bodies.append(south_wall6_body)

    root_C_south_wall6 = FixedConnection(parent=root, child=south_wall6_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.29025, y=1.80, z=0.5))
    all_wall_connections.append(root_C_south_wall6)

    south_wall7 = Box(scale=Scale(0.05, 2.27, 1.00), color=gray)
    shape_geometry = ShapeCollection([south_wall7])
    south_wall7_body = Body(name=PrefixedName("south_wall7_body"), collision=shape_geometry, visual=shape_geometry)
    all_wall_bodies.append(south_wall7_body)

    root_C_south_wall7 = FixedConnection(parent=root, child=south_wall7_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.29025, y=5.16, z=0.5))
    all_wall_connections.append(root_C_south_wall7)

    east_wall = Box(scale=Scale(4.924, 0.05, 3.00), color=gray)
    shape_geometry = ShapeCollection([east_wall])
    east_wall_body = Body(name=PrefixedName("east_wall_body"), collision=shape_geometry, visual=shape_geometry)
    all_wall_bodies.append(east_wall_body)

    root_C_east_wall = FixedConnection(parent=root, child=east_wall_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=2.462, y=-2.535, z=1.50))
    all_wall_connections.append(root_C_east_wall)

    middle_wall = Box(scale=Scale(0.05, 2.67, 1.00), color=gray)
    shape_geometry = ShapeCollection([middle_wall])
    middle_wall_body = Body(name=PrefixedName("middle_wall_body"), collision=shape_geometry, visual=shape_geometry)
    all_wall_bodies.append(middle_wall_body)

    root_C_middle_wall = FixedConnection(parent=root, child=middle_wall_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=2.20975, y=5.00, z=0.50))
    all_wall_connections.append(root_C_middle_wall)

    west_wall = Box(scale=Scale(4.449, 0.05, 3.00), color=gray)
    shape_geometry = ShapeCollection([west_wall])
    west_wall_body = Body(name=PrefixedName("west_wall_body"), collision=shape_geometry, visual=shape_geometry)
    all_wall_bodies.append(west_wall_body)

    root_C_west_wall = FixedConnection(parent=root, child=west_wall_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=1.9345, y=6.32, z=1.50))
    all_wall_connections.append(root_C_west_wall)

    north_wall = Box(scale=Scale(0.05, 8.04, 3.00), color=gray)
    shape_geometry = ShapeCollection([north_wall])
    north_wall_body = Body(name=PrefixedName("north_wall_body"), collision=shape_geometry, visual=shape_geometry)
    all_wall_bodies.append(north_wall_body)

    root_C_north_wall = FixedConnection(parent=root, child=north_wall_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=4.949, y=1.51, z=1.50))
    all_wall_connections.append(root_C_north_wall)

    north_west_wall = Cylinder(width=1.53, height=3.00, color=gray)
    shape_geometry = ShapeCollection([north_west_wall])
    north_west_wall_body = Body(name=PrefixedName("north_west_wall_body"), collision=shape_geometry, visual=shape_geometry)
    all_wall_bodies.append(north_west_wall_body)

    root_C_north_west_wall = FixedConnection(parent=root, child=north_west_wall_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=4.924, y=6.295, z=1.50))
    all_wall_connections.append(root_C_north_west_wall)

    with wall_world.modify_world():
        for body in all_wall_bodies:
            wall_world.add_body(body)

        for conn in all_wall_connections:
            wall_world.add_connection(conn)
        return wall_world


def loading_environment_elements():
    elements_world = World()
    all_elements_bodies = []
    all_elements_connections = []

    root = Body(name=PrefixedName("root"))
    all_elements_bodies.append(root)

    refrigerator = Box(scale=Scale(0.60, 0.658, 1.49), color=white)
    shape_geometry = ShapeCollection([refrigerator])
    refrigerator_body = Body(name=PrefixedName("refrigerator_body"), collision=shape_geometry, visual=shape_geometry)
    all_elements_bodies.append(refrigerator_body)

    root_C_fridge = FixedConnection(parent=root, child=refrigerator_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=0.537, y=-2.181, z=0.745))
    all_elements_connections.append(root_C_fridge)

    counterTop = Box(scale=Scale(2.044, 0.658, 0.545), color=wood)
    shape_geometry = ShapeCollection([counterTop])
    counterTop_body = Body(name=PrefixedName("counterTop_body"), collision=shape_geometry, visual=shape_geometry)
    all_elements_bodies.append(counterTop_body)

    root_C_counterTop = FixedConnection(parent=root, child=counterTop_body,
                                        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=1.859,y=-2.181, z=0.2725))
    all_elements_connections.append(root_C_counterTop)

    ovenArea = Box(scale=Scale(1.20, 0.658, 1.49), color=white)
    shape_geometry = ShapeCollection([ovenArea])
    ovenArea_body = Body(name=PrefixedName("ovenArea_body"), collision=shape_geometry, visual=shape_geometry)
    all_elements_bodies.append(ovenArea_body)

    root_C_ovenArea = FixedConnection(parent=root, child=ovenArea_body,
                                      parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=3.481,y=-2.181, z=0.745))
    all_elements_connections.append(root_C_ovenArea)

    table = Box(scale=Scale(2.45, 0.796, 0.845), color=white)
    shape_geometry = ShapeCollection([table])
    table_body = Body(name=PrefixedName("table_body"), collision=shape_geometry, visual=shape_geometry)
    all_elements_bodies.append(table_body)

    root_C_table = FixedConnection(parent=root, child=table_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=3.545, y=0.426, z=0.4225))
    all_elements_connections.append(root_C_table)

    sofa = Box(scale=Scale(1.68, 0.94, 0.68), color=wood)
    shape_geometry = ShapeCollection([sofa])
    sofa_body = Body(name=PrefixedName("sofa_body"), collision=shape_geometry, visual=shape_geometry)
    all_elements_bodies.append(sofa_body)

    root_C_sofa = FixedConnection(parent=root, child=sofa_body,
                                  parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=3.60, y=1.20, z=0.34))
    all_elements_connections.append(root_C_sofa)

    lowerTable = Box(scale=Scale(0.37, 0.91, 0.44), color=white)
    shape_geometry = ShapeCollection([lowerTable])
    lowerTable_body = Body(name=PrefixedName("lowerTable_body"), collision=shape_geometry, visual=shape_geometry)
    all_elements_bodies.append(lowerTable_body)

    root_C_lowerTable = FixedConnection(parent=root, child=lowerTable_body,
                                        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=4.22, y=2.22, z=0.22))
    all_elements_connections.append(root_C_lowerTable)

    cabinet = Box(scale=Scale(0.43, 0.80, 2.02), color=white)
    shape_geometry = ShapeCollection([cabinet])
    cabinet_body = Body(name=PrefixedName("cabinet_body"), collision=shape_geometry, visual=shape_geometry)
    all_elements_bodies.append(cabinet_body)

    root_C_cabinet = FixedConnection(parent=root, child=cabinet_body,
                                     parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=4.65, y=4.72, z=1.01))
    all_elements_connections.append(root_C_cabinet)

    desk = Box(scale=Scale(0.60, 1.20, 0.75), color=white)
    shape_geometry = ShapeCollection([desk])
    desk_body = Body(name=PrefixedName("desk_body"), collision=shape_geometry, visual=shape_geometry)
    all_elements_bodies.append(desk_body)

    root_C_desk = FixedConnection(parent=root, child=desk_body,
                                  parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=0.05, y=1.48, z=0.375))
    all_elements_connections.append(root_C_desk)

    cookingTable = Box(scale=Scale(1.75, 0.64, 0.71),color=wood)
    shape_geometry = ShapeCollection([cookingTable])
    cookingTable_body = Body(name=PrefixedName("cookingTable_body"), collision=shape_geometry, visual=shape_geometry)
    all_elements_bodies.append(cookingTable_body)

    root_C_cookingTable = FixedConnection(parent=root,child=cookingTable_body,
                                  parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=1.325, y=5.675, z=0.355))
    all_elements_connections.append(root_C_cookingTable)


    diningTable = Box(scale=Scale(0.73, 1.18, 0.73),color=wood)
    shape_geometry = ShapeCollection([diningTable])
    diningTable_body = Body(name=PrefixedName("diningTable_body"), collision=shape_geometry, visual=shape_geometry)
    all_elements_bodies.append(diningTable_body)

    root_C_diningTable = FixedConnection(parent=root,child=diningTable_body,
                                         parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=2.59975, y=5.705, z=0.365))
    all_elements_connections.append(root_C_diningTable)

    with elements_world.modify_world():
        for body in all_elements_bodies:
            elements_world.add_body(body)

        for conn in all_elements_connections:
            elements_world.add_connection(conn)
        return elements_world

class Publisher:
    def __init__(self, name):
        self.context = rclpy.init()
        self.node = rclpy.create_node(name)
        self.thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.thread.start()

    def publish(self, world):
        viz = VizMarkerPublisher(world=world, node=self.node)


def published(world: World):
    rclpy.init()
    node = rclpy.create_node("semantic_digital_twin")
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

world_elements = loading_environment_elements()
world_walls = loading_environment_walls()
publisher = Publisher("semantic_digital_twin")
publisher.publish(world_elements)
publisher.publish(world_walls)
