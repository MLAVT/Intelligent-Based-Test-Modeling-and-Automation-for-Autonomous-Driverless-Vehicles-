#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Follow leading vehicle scenario:

The scenario realizes a common driving behavior, in which the
user-controlled ego vehicle follows a leading car driving down
a given road. At some point the leading car has to slow down and
finally stop. The ego vehicle has to react accordingly to avoid
a collision. The scenario ends either via a timeout, or if the ego
vehicle stopped close enough to the leading vehicle
"""

import random

import py_trees

import carla
from enum import Enum
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance
class DirectionalPerspective(Enum):
    ADD_X = "ADD_X"
    SUBTRACT_X = "SUBTRACT_X"
    ADD_Y = "ADD_Y"
    SUBTRACT_Y = "SUBTRACT_Y"

waypoint_location = {
    "Town10HD": {
        'Three-way intersection': {
            "location": carla.Location(x=85.5, y=70.5, z=0.3),
            "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0),
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        'Four-way intersection': {
            "location": carla.Location(x=-49, y=-10, z=0.3),
            "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0),
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        'Multi-way intersection': {
            "location": carla.Location(x=-60, y=140, z=0.3),
            "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0),
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        "yield-controlled intersection": {
            "location": carla.Location(x=85.5, y=70.5, z=0.3),
            "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0),
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        "stop-controlled intersection": {
            "location": carla.Location(x=-48, y=-115, z=0.3),
            "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0),
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        "signal-controlled intersection": {
            "location": carla.Location(x=-82, y=16, z=0.3),
            "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0),
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        "traffic circle": {
            "location": carla.Location(x=-60, y=140, z=0.3),
            "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0),
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        'Two-way intersection': {
            "location": carla.Location(x=102, y=70, z=0.3),
            "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0),
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        "one way road": {
            "location": carla.Location(x=-7, y=24, z=0.3),
            "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0),
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        },
        "parking lot": {
            "location": carla.Location(x=-58, y=186, z=0.3),
            "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0),
            "perspective": {
                "left": DirectionalPerspective.ADD_X.value,
                "right": DirectionalPerspective.SUBTRACT_X.value,
                "front": DirectionalPerspective.ADD_Y.value,
                "back": DirectionalPerspective.SUBTRACT_Y.value,
            },
        }
    }
}

class FollowLeadingVehicle(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 25
        self._first_vehicle_speed = 10
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowLeadingVehicle, self).__init__("FollowVehicle",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

            # Example code how to randomize start location
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self._other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
            first_vehicle_waypoint.transform.rotation)
        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
            self._other_actor_transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self._other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """

        # to avoid the other actor blocking traffic, it was spawed elsewhere
        # reset its pose to the required one
        start_transform = ActorTransformSetter(self._other_actors[0], self._other_actor_transform)

        # let the other actor drive until next intersection
        # @todo: We should add some feedback mechanism to respond to ego_vehicle behavior
        driving_to_next_intersection = py_trees.composites.Parallel(
            "DrivingTowardsIntersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        driving_to_next_intersection.add_child(WaypointFollower(self._other_actors[0], self._first_vehicle_speed))
        driving_to_next_intersection.add_child(InTriggerDistanceToNextIntersection(
            self._other_actors[0], self._other_actor_stop_in_front_intersection))

        # stop vehicle
        stop = StopVehicle(self._other_actors[0], self._other_actor_max_brake)

        # end condition
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self._other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform)
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(stop)
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self._other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class FollowLeadingVehicleWithObstacle(BasicScenario):

    """
    This class holds a scenario similar to FollowLeadingVehicle
    but there is an obstacle in front of the leading vehicle

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
        """
        Setup all relevant parameters and create scenario
        """
        self._map = CarlaDataProvider.get_map()
        self._first_actor_location = 25
        self._second_actor_location = self._first_actor_location + 41
        self._first_actor_speed = 10
        self._second_actor_speed = 1.5
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._first_actor_transform = None
        self._second_actor_transform = None
        self._actor_transforms = {}
        self._other_actors = {}

        #####OVERRIDE_SPECTATOR_DEFINITION#####
        ego_vehicle_location = ego_vehicles[0].get_location()
        ego_vehicle_rotation = ego_vehicles[0].get_transform().rotation
        world.get_spectator().set_transform(carla.Transform(carla.Location(ego_vehicle_location.x, ego_vehicle_location.y, 50), carla.Rotation(-90, ego_vehicle_rotation.yaw, 0)))

        super(FollowLeadingVehicleWithObstacle, self).__init__("FollowLeadingVehicleWithObstacle",
                                                               ego_vehicles,
                                                               config,
                                                               world,
                                                               debug_mode,
                                                               criteria_enable=criteria_enable)
        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        ego_vehicle_location = self.ego_vehicles[0].get_location()
        ego_vehicle_rotation = self.ego_vehicles[0].get_transform().rotation

        #####OVERRIDE_WAYPOINT_DEFINITION#####

        #####END_ADD_OBJECTS#####

        # From new_cara_script keep for demo
        # 
        # transform = carla.Transform(
        #     carla.Location(
        #         x=ego_vehicle_location.x,
        #         y=ego_vehicle_location.y + 20.14076,
        #         z=ego_vehicle_location.z,
        #     ),
        #     carla.Rotation(0, ego_vehicle_rotation.yaw, 0),
        # )
        # first_actor = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', transform)
        # self._first_actor_transform = transform

        # transform = carla.Transform(
        #     carla.Location(
        #         x=ego_vehicle_location.x,
        #         y=ego_vehicle_location.y + 15.45264,
        #         z=ego_vehicle_location.z,
        #     ),
        #     carla.Rotation(0, ego_vehicle_rotation.yaw, 0),
        # )
        # second_actor = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', transform)
        # self._second_actor_transform = transform

        # first_actor.set_simulate_physics(enabled=False)
        # second_actor.set_simulate_physics(enabled=False)

        # self._other_actors.append(first_actor)
        # self._other_actors.append(second_actor)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive towards obstacle.
        Once obstacle clears the road, make the other actor to drive towards the
        next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """
        ego_vehicle_location = self.ego_vehicles[0].get_location()
        # end condition
        endcondition = py_trees.composites.Parallel("Waiting for end position", policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        sequence = py_trees.composites.Sequence("Sequence Behavior")

        #####END_ADD_BEHAVIORS#####

        # let the other actor drive until next intersection
        driving_to_next_intersection = py_trees.composites.Parallel(
            "Driving towards Intersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        # obstacle_clear_road = py_trees.composites.Parallel("Obstalce clearing road",
        #                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        # obstacle_clear_road.add_child(DriveDistance(self._other_actors[1], 4))
        # obstacle_clear_road.add_child(KeepVelocity(self._other_actors[1], self._second_actor_speed))

        # stop_near_intersection = py_trees.composites.Parallel(
        #     "Waiting for end position near Intersection",
        #     policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        # stop_near_intersection.add_child(WaypointFollower(self._other_actors[0], 10))
        # stop_near_intersection.add_child(InTriggerDistanceToNextIntersection(self._other_actors[0], 20))

        # driving_to_next_intersection.add_child(WaypointFollower(self._other_actors[0], self._first_actor_speed))
        # driving_to_next_intersection.add_child(WaypointFollower(self._other_actors[1], self._first_actor_speed))
        # driving_to_next_intersection.add_child(InTriggerDistanceToVehicle(self._other_actors[1],
        #                                                                   self._other_actors[0], 15))

        # endcondition_part1 = InTriggerDistanceToVehicle(self._other_actors[0],
        #                                                 self.ego_vehicles[0],
        #                                                 distance=20,
        #                                                 name="FinalDistance")
        # endcondition.add_child(endcondition_part1)
        # endcondition.add_child(StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1))
        # endcondition.add_child(DriveDistance(self.ego_vehicles[0], 1))

        # Build behavior tree
        # sequence.add_child(ActorTransformSetter(self._other_actors[0], self._first_actor_transform))
        # sequence.add_child(ActorTransformSetter(self._other_actors[1], self._second_actor_transform))
        sequence.add_child(driving_to_next_intersection)
        # sequence.add_child(TimeOut(10))
        # sequence.add_child(StopVehicle(self._other_actors[0], self._other_actor_max_brake))
        # sequence.add_child(StopVehicle(self._other_actors[1], self._other_actor_max_brake))
        # sequence.add_child(TimeOut(3))
        # sequence.add_child(obstacle_clear_road)
        # sequence.add_child(stop_near_intersection)
        # sequence.add_child(StopVehicle(self._other_actors[0], self._other_actor_max_brake))
        sequence.add_child(endcondition)
        for ego_vehicle in self.ego_vehicles:
            sequence.add_child(ActorDestroy(ego_vehicle))
        for val in self._other_actors:
            sequence.add_child(ActorDestroy(val))
        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
