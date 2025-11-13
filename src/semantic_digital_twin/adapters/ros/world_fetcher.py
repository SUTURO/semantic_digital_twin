import json
from dataclasses import dataclass, field
from time import time, sleep
from typing import Optional

from rclpy.node import Node
from rclpy.service import Service
from std_srvs.srv import Trigger

from ..world_entity_kwargs_tracker import KinematicStructureEntityKwargsTracker
from ...world import World
from ...world_description.world_modification import (
    WorldModelModification,
    WorldModelModificationBlock,
)


@dataclass
class FetchWorldServer:
    """
    A ros service that allows other processes to fetch the entire world modification list from this world.
    The modification list is sent via a JSON string message.
    """

    node: Node
    """
    The rclpy node used to create the service.
    """

    world: World
    """
    The world to fetch modifications from.
    """

    service_name: str = "/semantic_digital_twin/fetch_world"
    """
    The name of the service.
    """

    service: Optional[Service] = field(default=None, init=False)
    """
    The ROS service object.
    """

    def __post_init__(self):
        """
        Initialize the ROS service.
        """
        self.service = self.node.create_service(
            Trigger, self.service_name, self.service_callback
        )

    def service_callback(self, request: Trigger.Request, response: Trigger.Response):
        """
        Handle service requests to fetch the world modification list.

        :param request: The service request (empty for Trigger service).
        :param response: The service response containing success status and message.
        :return: The populated response.
        """
        modifications_json = self.get_modifications_as_json()
        response.success = True
        response.message = modifications_json
        return response

    def get_modifications_as_json(self) -> str:
        """
        Serialize all world modification blocks to JSON.

        :return: JSON string containing all modification blocks.
        """
        modifications_list = [
            block.to_json()
            for block in self.world.get_world_model_manager().model_modification_blocks
        ]
        return json.dumps(modifications_list)

    def close(self):
        """
        Clean up the service.
        """
        if self.service is not None:
            self.node.destroy_service(self.service)
            self.service = None


@dataclass
class NoServiceFoundError(Exception):
    service_suffix: str


def fetch_world_from_service(
    node: Node,
    service_suffix: str = "fetch_world",
    timeout_seconds: float = 5.0,
) -> World:
    """
    Fetch a world from any WorldFetcher Service.
    This method discovers all available WorldFetcher Services in the ROS2 network and picks the first one to get all
    world modification blocks from it.

    :param node: The ROS2 node to use for communication.
    :param service_suffix: The suffix (last part behind '/') of the WorldFetcher services to look for.
    :param timeout_seconds: Maximum time to wait for service availability and response.
    :return: The fetched modification blocks.
    """
    deadline = time() + timeout_seconds
    while time() < deadline:
        available_services = node.get_service_names_and_types()
        matching_services = [
            name
            for name, srv_type in available_services
            if name.endswith(service_suffix) and srv_type == ["std_srvs/srv/Trigger"]
        ]

        if matching_services:
            break

        sleep(0.1)
    else:
        raise NoServiceFoundError(service_suffix)
    remaining = deadline - time()

    chosen_service = matching_services[0]
    client = node.create_client(Trigger, chosen_service)

    service_available = client.wait_for_service(timeout_sec=remaining)
    if not service_available:
        raise TimeoutError(
            f"WorldFetcher service '{chosen_service}' not available after {timeout_seconds} seconds"
        )

    # fetch world
    response = client.call(Trigger.Request())

    tracker = KinematicStructureEntityKwargsTracker()
    kwargs = tracker.create_kwargs()
    modifications = [
        WorldModelModificationBlock.from_json(block_json, **kwargs)
        for block_json in json.loads(response.message)
    ]

    world = World()
    for modification_block in modifications:
        modification_block.apply(world)

    return world
