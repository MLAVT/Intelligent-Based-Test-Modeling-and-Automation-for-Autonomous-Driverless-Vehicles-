#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
from enum import Enum
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

import random
import time
import subprocess

RUNTIME_IN_SEC = 30

# class syntax
class DirectionalPerspective(Enum):
    ADD_X = "ADD_X"
    SUBTRACT_X = "SUBTRACT_X"
    ADD_Y = "ADD_Y"
    SUBTRACT_Y = "SUBTRACT_Y"

def collision_sensor_callback(event):
    print("collision")
    print("============== Actor ===========")
    print(f"ID: {event.actor.id}")
    print(f"Type ID: {event.actor.type_id}")
    print("============== Other Actor ===========")
    print(f"ID: {event.other_actor.id}")
    print(f"Type ID: {event.other_actor.type_id}")

def obstacle_sensor_callback(event):
    print("============== Other Actor ===========")
    print(f"ID: {event.other_actor.id}")
    print(f"Type ID: {event.other_actor.type_id}")
    print(f"Distance: {event.distance}")

def lane_invasion_sensor_callback(event):
    print("lane invasion")
    print("============== Actor ===========")
    print(event)
    print(f"ID: {event.actor.id}")
    print(f"Type ID: {event.actor.type_id}")
    print(f"Actor State: {event.actor.state}")
    print(f"Crossed Lane Markings: {event.actor.crossed_lane_markings}")

def main():
    actor_list = []
    waypoint_location = {
        "three-way intersection": {"location": carla.Location(x=85.5, y=-70.5, z=0.3), "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0), "perspective": {"left": DirectionalPerspective.ADD_X, "right": DirectionalPerspective.SUBTRACT_X, "front": DirectionalPerspective.ADD_Y, "back": DirectionalPerspective.SUBTRACT_Y}},
        "four-way intersection": {"location": carla.Location(x=-45.5, y=-40, z=0.3), "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0), "perspective": {"left": DirectionalPerspective.ADD_X, "right": DirectionalPerspective.SUBTRACT_X, "front": DirectionalPerspective.ADD_Y, "back": DirectionalPerspective.SUBTRACT_Y}},
        "uncontrolled intersection": {"location": carla.Location(x=-60, y=140, z=0.3), "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0), "perspective": {"left": DirectionalPerspective.ADD_X, "right": DirectionalPerspective.SUBTRACT_X, "front": DirectionalPerspective.ADD_Y, "back": DirectionalPerspective.SUBTRACT_Y}},
        "yield-controlled intersection": {"location": carla.Location(x=-48, y=-115, z=0.3), "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0), "perspective": {"left": DirectionalPerspective.ADD_X, "right": DirectionalPerspective.SUBTRACT_X, "front": DirectionalPerspective.ADD_Y, "back": DirectionalPerspective.SUBTRACT_Y}},
        "stop-controlled intersection": {"location": carla.Location(x=-48, y=-115, z=0.3), "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0), "perspective": {"left": DirectionalPerspective.ADD_X, "right": DirectionalPerspective.SUBTRACT_X, "front": DirectionalPerspective.ADD_Y, "back": DirectionalPerspective.SUBTRACT_Y}},
        "signal-controlled intersection": {"location": carla.Location(x=-48, y=-115, z=0.3), "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0), "perspective": {"left": DirectionalPerspective.ADD_X, "right": DirectionalPerspective.SUBTRACT_X, "front": DirectionalPerspective.ADD_Y, "back": DirectionalPerspective.SUBTRACT_Y}},
        "traffic circle": {"location": carla.Location(x=-60, y=140, z=0.3), "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0), "perspective": {"left": DirectionalPerspective.ADD_X, "right": DirectionalPerspective.SUBTRACT_X, "front": DirectionalPerspective.ADD_Y, "back": DirectionalPerspective.SUBTRACT_Y}},
        "two way road": {"location": carla.Location(x=-60, y=140, z=0.3), "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0), "perspective": {"left": DirectionalPerspective.ADD_X, "right": DirectionalPerspective.SUBTRACT_X, "front": DirectionalPerspective.ADD_Y, "back": DirectionalPerspective.SUBTRACT_Y}},
        "one way road": {"location": carla.Location(x=80, y=-17, z=0.3), "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0), "perspective": {"left": DirectionalPerspective.ADD_X, "right": DirectionalPerspective.SUBTRACT_X, "front": DirectionalPerspective.ADD_Y, "back": DirectionalPerspective.SUBTRACT_Y}},
        "parking lot": {"location": carla.Location(x=-58, y=186, z=0.3), "rotation": carla.Rotation(pitch=0.0, roll=0.0, yaw=90.0), "perspective": {"left": DirectionalPerspective.ADD_X, "right": DirectionalPerspective.SUBTRACT_X, "front": DirectionalPerspective.ADD_Y, "back": DirectionalPerspective.SUBTRACT_Y}},
    }

    # In this tutorial script, we are going to add a vehicle to the simulation
    # and let it drive in autopilot. We will also create a camera attached to
    # that vehicle, and save all the images generated by the camera to disk.

    try:
        # First of all, we need to create the client that will send the requests
        # to the simulator. Here we'll assume the simulator is accepting
        # requests in the localhost at port 2000.
        client = carla.Client('localhost', 2000)
        client.set_timeout(200.0)

        # Once we have a client we can retrieve the world that is currently
        # running.
        world = client.get_world()

        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.
        blueprint_library = world.get_blueprint_library()

        # Now let's filter all the blueprints of type 'vehicle' and choose one
        # at random.
        bp = random.choice(blueprint_library.filter('vehicle'))

        # A blueprint contains the list of attributes that define a vehicle's
        # instance, we can read them and modify some of them. For instance,
        # let's randomize its color.
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)

        # Now we need to give an initial transform to the vehicle. We choose a
        # random transform from the list of recommended spawn points of the map.
        map = world.get_map()
        waypoint = map.get_waypoint(location=waypoint_location["one way road"]["location"], project_to_road=True, lane_type=carla.LaneType.Driving) # Overriden with location and lane_type different
        waypoint_rotation = waypoint_location["one way road"]["rotation"] # Overriden with yaw different (i.e., the way the car is facing)
        #####OVERRIDE_WAYPOINT_DEFINITION#####
        waypoint_loc = waypoint.transform.location
        transform = carla.Transform(carla.Location(x=waypoint_loc.x, y=waypoint_loc.y, z=0.3), waypoint_rotation)

        # So let's tell the world to spawn the vehicle.
        ego_vehicle = world.spawn_actor(bp, transform)

        ego_vehicle_forward_vector = ego_vehicle.get_transform().rotation.get_forward_vector() # To be referenced in generated script to initialize objects in relation to the directed vector

        # Send spectator view to vehicle's location
        world.get_spectator().set_transform(carla.Transform(carla.Location(ego_vehicle.get_location().x, ego_vehicle.get_location().y, 50),carla.Rotation(-90, ego_vehicle_rotation.yaw, ego_vehicle_rotation.roll)))

        # It is important to note that the actors we create won't be destroyed
        # unless we call their "destroy" function. If we fail to call "destroy"
        # they will stay in the simulation even after we quit the Python script.
        # For that reason, we are storing all the actors we create so we can
        # destroy them afterwards.
        actor_list.append(ego_vehicle)
        print('created %s' % ego_vehicle.type_id)

        # Let's put the vehicle to drive around.
        ego_vehicle.set_autopilot(True)

        # Let's add now a "depth" camera attached to the vehicle. Note that the
        # transform we give here is now relative to the vehicle.
        camera_bp = blueprint_library.find('sensor.camera.depth')
        ego_vehicle_location = ego_vehicle.get_location()
        ego_vehicle_rotation = ego_vehicle.get_transform().rotation
        camera_transform = carla.Transform(carla.Location(x=ego_vehicle_location.x, z=ego_vehicle_location.z+1))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
        actor_list.append(camera)
        print('created %s' % camera.type_id)

        # Now we register the function that will be called each time the sensor
        # receives an image. In this example we are saving the image to disk
        # converting the pixels to gray-scale.
        cc = carla.ColorConverter.LogarithmicDepth
        camera.listen(lambda image: image.save_to_disk('_out/%06d.png' % image.frame, cc))

        # Find the blueprint of the sensor.
        collision_blueprint = blueprint_library.find('sensor.other.collision')
        lane_invasion_blueprint = blueprint_library.find('sensor.other.lane_invasion')
        obstacle_blueprint = blueprint_library.find('sensor.other.obstacle')
        # Provide the position of the sensor relative to the vehicle.
        transform = carla.Transform(carla.Location(x=ego_vehicle_location.x, z=ego_vehicle_location.z))
        # Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
        collision_sensor = world.spawn_actor(collision_blueprint, transform, attach_to=ego_vehicle)
        lane_invasion_sensor = world.spawn_actor(lane_invasion_blueprint, transform, attach_to=ego_vehicle)
        obstacle_sensor = world.spawn_actor(obstacle_blueprint, transform, attach_to=ego_vehicle)
        # Subscribe to the sensor stream by providing a callback function, this function is
        # called each time a new image is generated by the sensor.
        collision_sensor.listen(lambda data: collision_sensor_callback(data))
        obstacle_sensor.listen(lambda data: obstacle_sensor_callback(data))
        lane_invasion_sensor.listen(lambda data: lane_invasion_sensor_callback(data))
        client.show_recorder_file_info("recording.log", True)
        client.show_recorder_collisions("collosions.log", "v", "a")
        client.show_recorder_actors_blocked("blocked.log", 1.0, 1.0)

        # # This time we are using try_spawn_actor. If the spot is already
        # # occupied by another object, the function will return None.

        #####END_ADD_OBJECTS#####
        try:
            t_end = time.time() + RUNTIME_IN_SEC
            # TODO: Run scenario runner in telecarla plugin instead
            subprocess.run(['py', 'manual_control.py'])
            while time.time() < t_end:
                world.get_spectator().set_transform(carla.Transform(carla.Location(ego_vehicle.get_location().x, ego_vehicle.get_location().y, 50),carla.Rotation(-90, ego_vehicle_rotation.yaw, ego_vehicle_rotation.roll)))
        except KeyboardInterrupt:
            time.sleep(RUNTIME_IN_SEC)
            pass

    finally:

        print('destroying actors')
        camera.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print('done.')


if __name__ == '__main__':

    main()