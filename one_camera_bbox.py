#!/usr/bin/env python

# Copyright (c) 2018 The simple robotics
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a keyboard. For a simpler and more
# documented example, please take a look at tutorial.py.

"""
Welcome to TSR manual control.

Use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    C            : change weather (Shift+C reverse)
    ESC          : quit
"""

from __future__ import print_function
import glob
import os
import sys


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_0
    from pygame.locals import K_9
    from pygame.locals import K_BACKQUOTE
    from pygame.locals import K_BACKSPACE
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_TAB
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_c
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_r
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_MINUS
    from pygame.locals import K_EQUALS
except ImportError:
    raise RuntimeError(
        'cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')


VIEW_WIDTH = '800'
VIEW_HEIGHT = '600'
VIEW_FOV = '90'

BB_COLOR = (0, 191, 255)

depth_cam = True
semseg_cam = False
# ==============================================================================
# -- TSR functions ----------------------------------------------------------
# ==============================================================================

# Dummy function for bbox draw
def bbox_visualize():
    for vehicle in world.world.get_actors().filter('vehicle.*'):
        print(vehicle.bounding_box)
        # draw Box
        transform = vehicle.get_transform()
        bounding_box = vehicle.bounding_box
        bounding_box.location += transform.location
        world.world.debug.draw_box(bounding_box, transform.rotation)

class CameraTsr(object):

    def __init__(self, vehicle, width, height, fov='90', tick='0.0', pygame_disp=False):
        self.vehicle = vehicle
        self.width = width
        self.height = height
        self.fov = fov
        self.tick = tick
        self._pygame_disp = pygame_disp  # display for pygame
        self._surface = None  # image to render fom array for pygame display
        self.camera = None

    def add_camera(self, x=-6.5, y=0.0, z=2.7,
                   roll=0, pitch=0, yaw=0,
                   camera_type='sensor.camera.rgb',
                   ):
        ''' The camera type can be also:
              'sensor.camera.semantic_segmentation'
               'sensor.camera.depth'
        '''
        if self.camera is None:
            # Find the blueprint of the sensor.
            blueprint = self.vehicle.get_world().get_blueprint_library().find(camera_type)
            # Modify the attributes of the blueprint to set image resolution and field of view.
            blueprint.set_attribute('image_size_x', self.width)
            blueprint.set_attribute('image_size_y', self.height)
            blueprint.set_attribute('fov', self.fov)
            # Set the time in seconds between sensor captures
            blueprint.set_attribute('sensor_tick', self.tick)
            # Provide the position of the sensor relative to the vehicle.
            transform = carla.Transform(carla.Location(
                x=x, y=y, z=z), carla.Rotation(roll=roll, pitch=pitch, yaw=yaw))
            # Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
            self.camera = self.vehicle.get_world().spawn_actor(
                blueprint, transform, attach_to=self.vehicle)
            # Subscribe to the sensor stream by providing a callback function, this function is
            # called each time a new image is generated by the sensor.
            self.camera.listen(lambda data: self._process_image(data))

            calibration = np.identity(3)
            calibration[0, 2] = float(self.width) / 2.0
            calibration[1, 2] = float(self.height) / 2.0
            calibration[0, 0] = calibration[1, 1] = float(self.width) / (2.0 * np.tan(float(self.fov) * np.pi / 360.0))
            self.camera.calibration = calibration
        else:
            Print("The camera sensor is already initialized")

    def _process_image(self, image):
        ''' The callback function which gets raw image
             and convert it to array '''

        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        if self._pygame_disp:
            self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, display):
        ''' The function gets display class from pygame window
            and render the image from the current camera. '''

        if self._surface is not None:
            display.blit(self._surface, (0, 0))

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters)
               if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate-1] + u'\u2026') if len(name) > truncate else name

# ==============================================================================
# -- ClientSideBoundingBoxes ---------------------------------------------------
# ==============================================================================


class ClientSideBoundingBoxes(object):
    """
    This is a module responsible for creating 3D bounding boxes and drawing them
    client-side on pygame surface.
    """

    @staticmethod
    def get_bounding_boxes(vehicles, camera):
        """
        Creates 3D bounding boxes based on carla vehicle list and camera.
        """

        bounding_boxes = [ClientSideBoundingBoxes.get_bounding_box(vehicle, camera) for vehicle in vehicles]
        # filter objects behind camera
        bounding_boxes = [bb for bb in bounding_boxes if all(bb[:, 2] > 0)]
        return bounding_boxes

    @staticmethod
    def draw_bounding_boxes(display, bounding_boxes):
        """
        Draws bounding boxes on pygame display.
        """

        bb_surface = pygame.Surface((int(VIEW_WIDTH), int(VIEW_HEIGHT)))
        bb_surface.set_colorkey((0, 0, 0))
        for bbox in bounding_boxes:
            points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
            # draw lines
            # base
            pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
            pygame.draw.line(bb_surface, BB_COLOR, points[0], points[1])
            pygame.draw.line(bb_surface, BB_COLOR, points[1], points[2])
            pygame.draw.line(bb_surface, BB_COLOR, points[2], points[3])
            pygame.draw.line(bb_surface, BB_COLOR, points[3], points[0])
            # top
            pygame.draw.line(bb_surface, BB_COLOR, points[4], points[5])
            pygame.draw.line(bb_surface, BB_COLOR, points[5], points[6])
            pygame.draw.line(bb_surface, BB_COLOR, points[6], points[7])
            pygame.draw.line(bb_surface, BB_COLOR, points[7], points[4])
            # base-top
            pygame.draw.line(bb_surface, BB_COLOR, points[0], points[4])
            pygame.draw.line(bb_surface, BB_COLOR, points[1], points[5])
            pygame.draw.line(bb_surface, BB_COLOR, points[2], points[6])
            pygame.draw.line(bb_surface, BB_COLOR, points[3], points[7])
        display.blit(bb_surface, (0, 0))

    @staticmethod
    def get_bounding_box(vehicle, camera):
        """
        Returns 3D bounding box for a vehicle based on camera view.
        """

        bb_cords = ClientSideBoundingBoxes._create_bb_points(vehicle)
        cords_x_y_z = ClientSideBoundingBoxes._vehicle_to_sensor(bb_cords, vehicle, camera)[:3, :]
        cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
        bbox = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
        camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
        return camera_bbox

    @staticmethod
    def _create_bb_points(vehicle):
        """
        Returns 3D bounding box for a vehicle.
        """

        cords = np.zeros((8, 4))
        extent = vehicle.bounding_box.extent
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
        return cords

    @staticmethod
    def _vehicle_to_sensor(cords, vehicle, sensor):
        """
        Transforms coordinates of a vehicle bounding box to sensor.
        """

        world_cord = ClientSideBoundingBoxes._vehicle_to_world(cords, vehicle)
        sensor_cord = ClientSideBoundingBoxes._world_to_sensor(world_cord, sensor)
        return sensor_cord

    @staticmethod
    def _vehicle_to_world(cords, vehicle):
        """
        Transforms coordinates of a vehicle bounding box to world.
        """

        bb_transform = carla.Transform(vehicle.bounding_box.location)
        bb_vehicle_matrix = ClientSideBoundingBoxes.get_matrix(bb_transform)
        vehicle_world_matrix = ClientSideBoundingBoxes.get_matrix(vehicle.get_transform())
        bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
        world_cords = np.dot(bb_world_matrix, np.transpose(cords))
        return world_cords

    @staticmethod
    def _world_to_sensor(cords, sensor):
        """
        Transforms world coordinates to sensor.
        """

        sensor_world_matrix = ClientSideBoundingBoxes.get_matrix(sensor.get_transform())
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        sensor_cords = np.dot(world_sensor_matrix, cords)
        return sensor_cords

    @staticmethod
    def get_matrix(transform):
        """
        Creates matrix from carla transform.
        """

        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================
class World(object):
    def __init__(self):
        self.world = None # carla_world
        self.map = None #self.world.get_map()
        self.player = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0

        self.image = None
        self.capture = True
        # Camera view
        self.camera_view = None

        # SVS cameras
        self.cam1_svs_front = None

        # SVS depth camera
        self.depth_cam1_svs_front = None

        # SVS semseg cameras
        self.sem_cam1_svs_front = None


    def restart(self):

        # Get a vehicle mercedes.
        blueprint_library = self.world.get_blueprint_library()

        vehicle_bp = blueprint_library.find('vehicle.mercedes-benz.coupe')
        vehicle_bp.set_attribute('role_name', 'hero')
        vehicle_bp.set_attribute('color', '0,0,0')

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(vehicle_bp, spawn_point)
        while self.player is None:
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(
                spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(vehicle_bp, spawn_point)

        # Svs front camera
        self.cam1_svs_front = CameraTsr(
            vehicle=self.player, width= VIEW_WIDTH, height= VIEW_HEIGHT, fov=VIEW_FOV, pygame_disp=True)
        self.cam1_svs_front.add_camera(
            x=0.8, y=0.0, z=1.7, roll=0, pitch=-15, yaw=0)
        self.camera_view = self.cam1_svs_front

        # Svs depth camera
        if depth_cam:
            self.depth_cam1_svs_front = CameraTsr(
                vehicle=self.player, width=VIEW_WIDTH, height=VIEW_HEIGHT, fov=VIEW_FOV, pygame_disp=True)
            self.depth_cam1_svs_front.add_camera(
                x=0.8, y=0.0, z=1.7, roll=0, pitch=-15, yaw=0, camera_type='sensor.camera.depth')

        # Svs semseg gt camera
        if semseg_cam:
            self.sem_cam1_svs_front = CameraTsr(
                vehicle=self.player, width= VIEW_WIDTH, height= VIEW_HEIGHT, fov= VIEW_FOV)
            self.sem_cam1_svs_front.add_camera(
                x=0.8, y=0.0, z=1.7, roll=0, pitch=-15, yaw=0, camera_type='sensor.camera.semantic_segmentation')

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.player.get_world().set_weather(preset[0])

    def render(self, display):
        self.camera_view.render(display)

    def destroy(self):
        actors = [
            self.player,
            self.camera_view.camera,
            self.cam1_svs_front.camera,
            self.depth_cam1_svs_front.camera,
            self.sem_cam1_svs_front.camera,
        ]
        for actor in actors:
            if actor is not None:
                actor.destroy()

    def set_synchronous_mode(self, synchronous_mode):
        """
        Sets synchronous mode.
        """

        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        self.world.apply_settings(settings)

    @staticmethod
    def set_image(weak_self, img):
        """
        Sets image coming from camera sensor.
        The self.capture flag is a mean of synchronization - once the flag is
        set, next coming image will be stored.
        """

        self = weak_self()
        if self.capture:
            self.image = img
            self.capture = False

    def control(self, car):
        """
        Applies control to main car based on pygame pressed keys.
        Will return True If ESCAPE is hit, otherwise False to end main loop.
        """

        keys = pygame.key.get_pressed()
        if keys[K_ESCAPE]:
            return True

        control = car.get_control()
        control.throttle = 0
        if keys[K_w]:
            control.throttle = 1
            control.reverse = False
        elif keys[K_s]:
            control.throttle = 1
            control.reverse = True
        if keys[K_a]:
            control.steer = max(-1., min(control.steer - 0.05, 0))
        elif keys[K_d]:
            control.steer = min(1., max(control.steer + 0.05, 0))
        elif keys[K_p]:
            self._autopilot_enabled = not self._autopilot_enabled
            self.world.player.set_autopilot(self._autopilot_enabled)
        elif keys[K_c]:
            self.world.next_weather()
        else:
            control.steer = 0
        control.hand_brake = keys[K_SPACE]

        car.apply_control(control)
        return False

    def game_loop(self):
        """
        Main program loop.
        """

        try:
            pygame.init()

            self.client = carla.Client('127.0.0.1', 2000)
            self.client.set_timeout(2.0)
            self.world = self.client.get_world()
            self.map = self.world.get_map()
            self.restart()
            #self.setup_car()
            #self.setup_camera()

            self.display = pygame.display.set_mode((int(VIEW_WIDTH), int(VIEW_HEIGHT)), pygame.HWSURFACE | pygame.DOUBLEBUF)
            pygame_clock = pygame.time.Clock()

            self.set_synchronous_mode(True)
            vehicles = self.world.get_actors().filter('vehicle.*')

            while True:
                self.world.tick()

                self.capture = True
                pygame_clock.tick_busy_loop(15)

                self.render(self.display)
                bounding_boxes = ClientSideBoundingBoxes.get_bounding_boxes(vehicles, self.camera_view.camera)
                print(bounding_boxes)
                ClientSideBoundingBoxes.draw_bounding_boxes(self.display, bounding_boxes)

                pygame.display.flip()

                pygame.event.pump()
                if self.control(self.player):
                    return

        finally:
            self.set_synchronous_mode(False)
            if self.world is not None:
               self.destroy()
            pygame.quit()





# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    Initializes the client-side bounding box demo.
    """

    try:
        client = World()
        client.game_loop()
    finally:
        print('EXIT')


if __name__ == '__main__':
    main()
