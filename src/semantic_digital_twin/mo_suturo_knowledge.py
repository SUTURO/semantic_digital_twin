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


def loading_environment():
    world = World()
    all_bodies=[]
    all_connections=[]


    # Collers:
    white = Color(1, 1, 1)
    red = Color(1, 0, 0)
    black = Color(0, 0, 0)
    gray = Color(0.74, 0.74, 0.74)
    wood = Color(1, 0.827, 0.6078)

    # =======================
    # === Bodies & Connections
    # =======================
    root = Body(name=PrefixedName("root"))
    all_bodies.append(root)

    # All the Walls:
    sWall1 = Box(scale=Scale(0.05, 1.00, 3.00), color=gray)
    visual = ShapeCollection([sWall1])
    collision = ShapeCollection([sWall1])
    sWall1_body = Body(name=PrefixedName("sWall1_body"), collision=collision, visual=visual)
    all_bodies.append(sWall1_body)

    root_C_sWall1 = FixedConnection(parent=root, child=sWall1_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(y=-2.01, z=1.50))
    all_connections.append(root_C_sWall1)

    sWall2 = Box(scale=Scale(0.29, 0.05, 3.00), color=gray)
    visual = ShapeCollection([sWall2])
    collision = ShapeCollection([sWall2])
    sWall2_body = Body(name=PrefixedName("sWall2_body"), collision=collision, visual=visual)
    all_bodies.append(sWall2_body)

    root_C_sWall2 = FixedConnection(parent=root, child=sWall2_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.145, y=-1.45, z=1.50))
    all_connections.append(root_C_sWall2)

    sWall3 = Box(scale=Scale(0.05, 1.085, 1.00), color=gray)
    visual = ShapeCollection([sWall3])
    collision = ShapeCollection([sWall3])
    sWall3_body = Body(name=PrefixedName("sWall3_body"), collision=collision, visual=visual)
    all_bodies.append(sWall3_body)

    root_C_sWall3 = FixedConnection(parent=root, child=sWall3_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.29, y=-0.9925, z=0.5))
    all_connections.append(root_C_sWall3)

    sWall4 = Box(scale=Scale(0.29, 0.05, 1.00), color=gray)
    visual = ShapeCollection([sWall4])
    collision = ShapeCollection([sWall4])
    sWall4_body = Body(name=PrefixedName("sWall4_body"), collision=collision, visual=visual)
    all_bodies.append(sWall4_body)

    root_C_sWall4 = FixedConnection(parent=root, child=sWall4_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.145, y=-0.45, z=0.5))
    all_connections.append(root_C_sWall4)

    sWall5 = Box(scale=Scale(0.29, 0.05, 1.00), color=gray)
    visual = ShapeCollection([sWall5])
    collision = ShapeCollection([sWall5])
    sWall5_body = Body(name=PrefixedName("sWall5_body"), collision=collision, visual=visual)
    all_bodies.append(sWall5_body)

    root_C_sWall5 = FixedConnection(parent=root, child=sWall5_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.145, y=0.45, z=0.5))
    all_connections.append(root_C_sWall5)

    sWall6 = Box(scale=Scale(0.05, 2.75, 1.00), color=gray)
    visual = ShapeCollection([sWall6])
    collision = ShapeCollection([sWall6])
    sWall6_body = Body(name=PrefixedName("sWall6_body"), collision=collision, visual=visual)
    all_bodies.append(sWall6_body)

    root_C_sWall6 = FixedConnection(parent=root, child=sWall6_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.29025, y=1.80, z=0.5))
    all_connections.append(root_C_sWall6)


    sWall7 = Box(scale=Scale(0.05, 2.27, 1.00), color=gray)
    visual = ShapeCollection([sWall7])
    collision = ShapeCollection([sWall7])
    sWall7_body = Body(name=PrefixedName("sWall7_body"), collision=collision, visual=visual)
    all_bodies.append(sWall7_body)

    root_C_sWall7 = FixedConnection(parent=root, child=sWall7_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=-0.29025, y=5.16, z=0.5))
    all_connections.append(root_C_sWall7)

    eWall = Box(scale=Scale(4.924, 0.05, 3.00), color=gray)
    visual = ShapeCollection([eWall])
    collision = ShapeCollection([eWall])
    eWall_body = Body(name=PrefixedName("eWall_body"), collision=collision, visual=visual)
    all_bodies.append(eWall_body)

    root_C_eWall = FixedConnection(parent=root, child=eWall_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=2.462, y=-2.535, z=1.50))
    all_connections.append(root_C_eWall)

    mWall = Box(scale=Scale(0.05, 2.67, 1.00), color=gray)
    visual = ShapeCollection([mWall])
    collision = ShapeCollection([mWall])
    mWall_body = Body(name=PrefixedName("mWall_body"), collision=collision, visual=visual)
    all_bodies.append(mWall_body)

    root_C_mWall = FixedConnection(parent=root, child=mWall_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=2.20975, y=5.00, z=0.50))
    all_connections.append(root_C_mWall)

    wWall = Box(scale=Scale(4.449, 0.05, 3.00), color=gray)
    visual = ShapeCollection([wWall])
    collision = ShapeCollection([wWall])
    wWall_body = Body(name=PrefixedName("wWall_body"), collision=collision, visual=visual)
    all_bodies.append(wWall_body)

    root_C_wWall = FixedConnection(parent=root, child=wWall_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=1.9345, y=6.32, z=1.50))
    all_connections.append(root_C_wWall)


    nWAll = Box(scale=Scale(0.05, 8.04, 3.00), color=gray)
    visual = ShapeCollection([nWAll])
    collision = ShapeCollection([nWAll])
    nWall_body = Body(name=PrefixedName("nWAll_body"), collision=collision, visual=visual)
    all_bodies.append(nWall_body)

    root_C_nWall = FixedConnection(parent=root, child=nWall_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=4.949, y=1.51, z=1.50))
    all_connections.append(root_C_nWall)

    nwWall = Cylinder(width=1.53, height=3.00, color=gray)
    visual = ShapeCollection([nwWall])
    collision = ShapeCollection([nwWall])
    nwWall_body = Body(name=PrefixedName("nwWall_body"), collision=collision, visual=visual)
    all_bodies.append(nwWall_body)

    root_C_nwWall = FixedConnection(parent=root, child=nwWall_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=4.924, y=6.295, z=1.50))
    all_connections.append(root_C_nwWall)

    # The Rest:

    refrigerator = Box(scale=Scale(0.60, 0.658, 1.49), color=white)
    visual = ShapeCollection([refrigerator])
    collision = ShapeCollection([refrigerator])
    refrigerator_body = Body(name=PrefixedName("refrigerator_body"), collision=collision, visual=visual)
    all_bodies.append(refrigerator_body)

    root_C_fridge = FixedConnection(parent=root, child=refrigerator_body,
                                    parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=0.537, y=-2.181, z=0.745))
    all_connections.append(root_C_fridge)

    counterTop = Box(scale=Scale(2.044, 0.658, 0.545), color=wood)
    visual = ShapeCollection([counterTop])
    collision = ShapeCollection([counterTop])
    counterTop_body = Body(name=PrefixedName("counterTop_body"), collision=collision, visual=visual)
    all_bodies.append(counterTop_body)

    root_C_counterTop = FixedConnection(parent=root, child=counterTop_body,
                                        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=1.859,y=-2.181, z=0.2725))
    all_connections.append(root_C_counterTop)

    ovenArea = Box(scale=Scale(1.20, 0.658, 1.49), color=white)
    visual = ShapeCollection([ovenArea])
    collision = ShapeCollection([ovenArea])
    ovenArea_body = Body(name=PrefixedName("ovenArea_body"), collision=collision, visual=visual)
    all_bodies.append(ovenArea_body)

    root_C_ovenArea = FixedConnection(parent=root, child=ovenArea_body,
                                      parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=3.481,y=-2.181, z=0.745))
    all_connections.append(root_C_ovenArea)

    table = Box(scale=Scale(2.45, 0.796, 0.845), color=white)
    visual = ShapeCollection([table])
    collision = ShapeCollection([table])
    table_body = Body(name=PrefixedName("table_body"), collision=collision, visual=visual)
    all_bodies.append(table_body)

    root_C_table = FixedConnection(parent=root, child=table_body,
                                   parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=3.545, y=0.426, z=0.4225))
    all_connections.append(root_C_table)

    sofa = Box(scale=Scale(1.68, 0.94, 0.68), color=wood)
    visual = ShapeCollection([sofa])
    collision = ShapeCollection([sofa])
    sofa_body = Body(name=PrefixedName("sofa_body"), collision=collision, visual=visual)
    all_bodies.append(sofa_body)

    root_C_sofa = FixedConnection(parent=root, child=sofa_body,
                                  parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=3.60, y=1.20, z=0.34))
    all_connections.append(root_C_sofa)

    lowerTable = Box(scale=Scale(0.37, 0.91, 0.44), color=white)
    visual = ShapeCollection([lowerTable])
    collision = ShapeCollection([lowerTable])
    lowerTable_body = Body(name=PrefixedName("lowerTable_body"), collision=collision, visual=visual)
    all_bodies.append(lowerTable_body)

    root_C_lowerTable = FixedConnection(parent=root, child=lowerTable_body,
                                        parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=4.22, y=2.22, z=0.22))
    all_connections.append(root_C_lowerTable)

    cabinet = Box(scale=Scale(0.43, 0.80, 2.02), color=white)
    visual = ShapeCollection([cabinet])
    collision = ShapeCollection([cabinet])
    cabinet_body = Body(name=PrefixedName("cabinet_body"), collision=collision, visual=visual)
    all_bodies.append(cabinet_body)

    root_C_cabinet = FixedConnection(parent=root, child=cabinet_body,
                                     parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=4.65, y=4.72, z=1.01))
    all_connections.append(root_C_cabinet)

    desk = Box(scale=Scale(0.60, 1.20, 0.75), color=white)
    visual = ShapeCollection([desk])
    collision = ShapeCollection([desk])
    desk_body = Body(name=PrefixedName("desk_body"), collision=collision, visual=visual)
    all_bodies.append(desk_body)

    root_C_desk = FixedConnection(parent=root, child=desk_body,
                                  parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=0.05, y=1.48, z=0.375))
    all_connections.append(root_C_desk)

    cookingTable = Box(scale=Scale(1.75, 0.64, 0.71),color=wood)
    visual = ShapeCollection([cookingTable])
    collision = ShapeCollection([cookingTable])
    cookingTable_body = Body(name=PrefixedName("cookingTable_body"), collision=collision, visual=visual)
    all_bodies.append(cookingTable_body)

    root_C_cookingTable = FixedConnection(parent=root,child=cookingTable_body,
                                  parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=1.325, y=5.675, z=0.355))
    all_connections.append(root_C_cookingTable)


    diningTable = Box(scale=Scale(0.73, 1.18, 0.73),color=wood)
    visual = ShapeCollection([diningTable])
    collision = ShapeCollection([diningTable])
    diningTable_body = Body(name=PrefixedName("diningTable_body"), collision=collision, visual=visual)
    all_bodies.append(diningTable_body)

    root_C_diningTable = FixedConnection(parent=root,child=diningTable_body,
                                         parent_T_connection_expression=TransformationMatrix.from_xyz_rpy(x=2.59975, y=5.705, z=0.365))
    all_connections.append(root_C_diningTable)

    with world.modify_world():
        for body in all_bodies:
            world.add_body(body)

        for connection in all_connections:
            world.add_connection(connection)
        return world



def published(world: World):
    rclpy.init()
    node = rclpy.create_node("semantic_digital_twin")
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    viz = VizMarkerPublisher(world=world, node=node)

world = loading_environment()
published(world)
