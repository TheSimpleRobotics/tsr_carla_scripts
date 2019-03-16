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
    M            : toggle manual transmission
    ,/.          : gear up/down
    C            : change weather (Shift+C reverse)
    Backspace    : change vehicle

    R            : toggle recording images to disk
    ESC          : quit
"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


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
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- TSR functions ----------------------------------------------------------
# ==============================================================================
class CameraTsr(object):

    def __init__(self, vehicle, width, height, fov='90', tick='0.0', pygame_disp=False):
        self.vehicle=vehicle
        self.width=width
        self.height=height
        self.fov=fov
        self.tick=tick
        self._pygame_disp=pygame_disp #display for pygame
        self._surface = None # image to render fom array for pygame display
        self.camera = None

    def add_camera(self, x=-6.5, y=0.0, z=2.7, 
                   roll=0, pitch=0, yaw=0,
                   camera_type='sensor.camera.rgb', 
                   ):

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
            transform = carla.Transform(carla.Location(x=x,y=y,z=z), carla.Rotation(roll=roll,pitch=pitch,yaw=yaw))
            # Tell the world to spawn the sensor, don't forget to attach it to your vehicle actor.
            self.camera = self.vehicle.get_world().spawn_actor(blueprint, transform, attach_to=self.vehicle)
            # Subscribe to the sensor stream by providing a callback function, this function is
            # called each time a new image is generated by the sensor.
            self.camera.listen(lambda data: self._process_image(data))
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
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate-1] + u'\u2026') if len(name) > truncate else name


# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, actor_filter):
        self.world = carla_world
        self.map = self.world.get_map()
        self.player = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = actor_filter
        # Camera view
        self.camera_view = None
        self.restart()

        # SVS cameras
        self.cam1_svs_front = None
        self.cam2_svs_rigtht = None
        self.cam3_svs_back = None
        self.cam4_svs_rear = None    

    def restart(self):

        # Get a vehicle mercedes.
        blueprint_library = self.world.get_blueprint_library()

        vehicle_bp = blueprint_library.find('vehicle.mercedes-benz.coupe')
        vehicle_bp.set_attribute('role_name', 'hero')
        vehicle_bp.set_attribute('color', '255,0,0')

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
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(vehicle_bp, spawn_point)

        # Add default view camera for pygame window
        self.camera_view=CameraTsr(vehicle=self.player, width=str(800), height=str(600), fov="90",pygame_disp=True)
        self.camera_view.add_camera()

        # Svs front camera
        self.cam1_svs_front=CameraTsr(vehicle=self.player, width=str(320), height=str(200), fov="90")
        self.cam1_svs_front.add_camera(x=0.8, y=0.0, z=1.7, roll=0, pitch=-15, yaw=0)

        # Svs right camera
        self.cam2_svs_right=CameraTsr(vehicle=self.player, width=str(320), height=str(200), fov="90")
        self.cam2_svs_right.add_camera( x=0.0, y=0.5, z=1.7, roll=0, pitch=-15, yaw=90)

        # Svs back camera
        self.cam3_svs_back=CameraTsr(vehicle=self.player, width=str(320), height=str(200), fov="90")
        self.cam3_svs_back.add_camera(x=-0.8, y=0.0, z=1.7, roll=0, pitch=-15, yaw=180)

        # Svs left camera
        self.cam4_svs_left=CameraTsr(vehicle=self.player, width=str(320), height=str(200), fov="90")
        self.cam4_svs_left.add_camera(x=0.0, y=-0.5, z=1.7, roll=0, pitch=-15, yaw=270)


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
            self.cam2_svs_rigtht.camera,
            self.cam3_svs_back.camera,
            self.cam4_svs_rear.camera]
        for actor in actors:
            if actor is not None:
                actor.destroy()

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    def __init__(self, world, start_in_autopilot):
        self._autopilot_enabled = start_in_autopilot
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            world.player.set_autopilot(self._autopilot_enabled)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self._steer_cache = 0.0

    def parse_events(self, client, world, clock):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_BACKSPACE:
                    world.restart()
                elif event.key == K_c and pygame.key.get_mods() & KMOD_SHIFT:
                    world.next_weather(reverse=True)
                elif event.key == K_c:
                    world.next_weather()
                if isinstance(self._control, carla.VehicleControl):
                    if event.key == K_q:
                        self._control.gear = 1 if self._control.reverse else -1
                    elif event.key == K_m:
                        self._control.manual_gear_shift = not self._control.manual_gear_shift
                        self._control.gear = world.player.get_control().gear
                    elif self._control.manual_gear_shift and event.key == K_COMMA:
                        self._control.gear = max(-1, self._control.gear - 1)
                    elif self._control.manual_gear_shift and event.key == K_PERIOD:
                        self._control.gear = self._control.gear + 1
                    elif event.key == K_p and not (pygame.key.get_mods() & KMOD_CTRL):
                        self._autopilot_enabled = not self._autopilot_enabled
                        world.player.set_autopilot(self._autopilot_enabled)
        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
                self._control.reverse = self._control.gear < 0
            elif isinstance(self._control, carla.WalkerControl):
                self._parse_walker_keys(pygame.key.get_pressed(), clock.get_time())
            world.player.apply_control(self._control)

    def _parse_vehicle_keys(self, keys, milliseconds):
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = keys[K_SPACE]

    def _parse_walker_keys(self, keys, milliseconds):
        self._control.speed = 0.0
        if keys[K_DOWN] or keys[K_s]:
            self._control.speed = 0.0
        if keys[K_LEFT] or keys[K_a]:
            self._control.speed = .01
            self._rotation.yaw -= 0.08 * milliseconds
        if keys[K_RIGHT] or keys[K_d]:
            self._control.speed = .01
            self._rotation.yaw += 0.08 * milliseconds
        if keys[K_UP] or keys[K_w]:
            self._control.speed = 5.556 if pygame.key.get_mods() & KMOD_SHIFT else 2.778
        self._control.jump = keys[K_SPACE]
        self._rotation.yaw = round(self._rotation.yaw, 1)
        self._control.direction = self._rotation.get_forward_vector()

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        display = pygame.display.set_mode(
            (args.width, args.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF)

        world = World(client.get_world(), args.filter)
        controller = KeyboardControl(world, args.autopilot)

        clock = pygame.time.Clock()
        while True:
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock):
                return
            world.render(display)
            pygame.display.flip()

    finally:

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='800x600',
        help='window resolution (default: 800x600)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
