#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import *
import math
import time
import numpy as np

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))


class PlaneManager():

    #состояния самолета
    ON_SHIP = 1
    IN_AIR = 2
    RETURN_TO_SHIP = 3

    RETURN_TIME = 20.0

    def __init__(self, world, ship, planes):
        self.planes = planes
        self.ship = ship
        self.world = world
        self.joints = []
        self.planes_states = []
        self.planes_time = []
        for i, plane in enumerate(self.planes):
            self.planes_states.append(self.ON_SHIP)
            self.planes_time.append(float("inf"))
            self.stop_plane(i, plane)
           
    def start_plane(self):
        for i, plane in enumerate(self.planes):
            if self.planes_states[i] == self.ON_SHIP:
                self.world.DestroyJoint(self.joints[i])
                self.planes_states[i] = self.IN_AIR
                self.planes_time[i] = time.time()
                angle = 0
                for j in range(len(self.planes)):
                    if self.planes_time[j] < self.planes_time[i]:
                        angle += 2 * math.pi / 5
                plane.body.angle = angle
                direction = plane.body.GetWorldVector((0, 1))
                plane.body.ApplyForceToCenter(direction * Plane.max_linear_speed, True)
                break

    def stop_plane(self, i, plane):
        plane.body.position.angle = self.ship.body.angle
        plane.body.position.worldCenter = self.ship.body.worldCenter
        joint = self.world.CreateRevoluteJoint(bodyA=plane.body, \
                                            bodyB=self.ship.body, \
                                            anchor=self.ship.body.worldCenter)
        if len(self.joints) <= i:
            self.joints.append(joint)
        else:
            self.joints[i] = joint
        self.planes_time[i] = float("inf")

    def return_to_ship(self, plane):
        dir = self.ship.body.worldCenter - plane.body.worldCenter
        dir_angle = angle(dir, plane.body.linearVelocity)
        if math.fabs(dir_angle) > 0.1:
            plane.body.linearVelocity.Normalize()
            plane.body.linearVelocity *= Plane.max_linear_speed
            plane.angular_speed = plane.max_angular_speed
            if dir_angle > 0:
                plane.turn = 1.0
            else:
                plane.turn = -1.0
        else:
            plane.angular_speed = 0

    def move_in_air(self, plane):
        dir = self.ship.body.worldCenter - plane.body.worldCenter
        angle_dir = angle(dir, plane.body.GetWorldVector((0, 1)))
        if (math.fabs(plane.body.worldCenter.y) >= self.max_distance \
            or math.fabs(plane.body.worldCenter.x) >= self.max_distance) and angle != math.pi/2:
        #if math.fabs(plane.body.worldCenter.y) + 10 > min(100, self.max_distance) or \
        #        math.fabs(plane.body.worldCenter.x) + 10 > min(100, self.max_distance):
            plane.angular_speed = plane.max_angular_speed
            plane.turn = -1.0
        else:
            plane.angular_speed = 0.0
            plane.turn = 0.0

    def update(self, settings):
        Plane.min_linear_speed = settings.minLinearVelocity
        Plane.max_linear_speed = settings.maxLinearVelocity
        Plane.max_angular_speed = settings.maxAngularVelocity / 10

        #максимальная дистанция за которое самолет должен вернуться
        distance = (10 - math.pi * 2 / Plane.max_angular_speed) \
             * Plane.max_linear_speed
        #выбор радиуса окржности вдоль который будет летать самолет
        self.max_distance = min(distance, 90)

        for i, plane in enumerate(self.planes):
            if self.planes_states[i] == self.IN_AIR:
                if (time.time() - self.planes_time[i]) > 20:
                    self.planes_states[i] = self.RETURN_TO_SHIP
                self.move_in_air(plane)
                plane.update()

            if self.planes_states[i] == self.RETURN_TO_SHIP:
                self.return_to_ship(plane)
                dist = self.ship.body.worldCenter - plane.body.worldCenter
                if math.fabs(dist.lengthSquared) < 1:
                    self.planes_states[i] = self.ON_SHIP
                    self.stop_plane(i, plane)
                plane.update()

class Plane(object):
    vertices = [(2.0, -4.0),
                (0.0, 4.0),
                (-2.4, -4.0),]

    min_linear_speed = 0
    max_linear_speed = 0
    max_angular_speed = 0

    def __init__(self, world, vertices=None, density=0.1, position=(0, 0)):
        if vertices is None: vertices = Plane.vertices

        self.linear_speed_sqr = 0.0
        self.body = world.CreateDynamicBody(position=position)
        fixture = self.body.CreatePolygonFixture(vertices=vertices,\
                                                 density=density)
        self.body.angularDamping = 1.1
        self.body.linearDamping = 1.1
        fixture.filterData.groupIndex = -8
                
        self.turn = 0.0
        self.angular_speed = 0.0

    def update_linear(self):
        self.body.linearVelocity = self.body.GetWorldVector((0, 1)) * \
            self.body.linearVelocity.lengthSquared
        if self.body.linearVelocity.lengthSquared < self.min_linear_speed:
            self.body.linearVelocity.Normalize()
            self.body.linearVelocity *= self.min_linear_speed
        elif self.body.linearVelocity.lengthSquared > self.max_linear_speed:
            self.body.linearVelocity.Normalize()
            self.body.linearVelocity *= self.max_linear_speed
        self.linear_speed_sqr = self.body.linearVelocity.lengthSquared

    def update_angular(self, angular_speed, turn):
        self.body.angularVelocity = angular_speed * turn

    def update(self):
        self.update_linear()
        self.update_angular(self.angular_speed, self.turn)
        

class Ship(object):
    vertices = [(1.5, 0.0),
                (3.0, 5.0),
                (2.8, 11.0),
                (1.0,20.0),
                (-1.0,20.0),
                (-2.8, 11.0),
                (-3.0, 5.0),
                (-1.5, 0.0),]

    LINEAR_SPEED = 50
    ANGULAR_SPEED = 0.1
    ANGULAR_MAX_IMPULSE = 1.5


    def __init__(self, world, vertices=None, density=0.1, position=(0, 0)):

        self.linear_speed_sqr = 0

        if vertices is None: vertices = Ship.vertices

        self.body = world.CreateDynamicBody(position=position)
        fixture = self.body.CreatePolygonFixture(vertices=vertices, density=density)
        self.body.angularDamping = 1.1
        self.body.linearDamping = 1.1
        fixture.filterData.groupIndex = -8


    def update_linear(self, throttle):
        direction = self.body.GetWorldVector((0, 1))
        self.body.ApplyForceToCenter(self.LINEAR_SPEED * throttle * direction, True)
        self.linear_speed_sqr = self.body.linearVelocity.lengthSquared

    def update_angular(self, turn):
        angular_impulse = self.ANGULAR_SPEED * self.linear_speed_sqr
        if angular_impulse > self.ANGULAR_MAX_IMPULSE: angular_impulse = self.ANGULAR_MAX_IMPULSE
        self.body.ApplyAngularImpulse(angular_impulse * turn, True)

    def update(self, keys):

        throttle = 0
        if 'up' in keys: throttle += 1
        if 'down' in keys: throttle -= 1
        self.update_linear(throttle)
    
        turn = 0
        if 'left' in keys: turn += 1
        if 'right' in keys: turn -= 1
        self.update_angular(turn)

class ShipGame(Framework):
    name = "Ship Game"
    description = "Keys: accel = w, reverse = s, left = a, right = d"

    def __init__(self):
        super(ShipGame, self).__init__()

        # Top-down -- no gravity in the screen plane
        self.world.gravity = (0, 0)
        self.key_map = {Keys.K_w: 'up', Keys.K_s: 'down', Keys.K_a: 'left', Keys.K_d: 'right'}
        self.key_up_map = {Keys.K_h: "plane"}

        # Keep track of the pressed keys
        self.pressed_keys = set()
        self.unpressed_keys = set()

        # The walls
        boundary = self.world.CreateStaticBody(position=(0, 20))
        boundary.CreateEdgeChain([(-120,-120),
                                  (-120, 120),
                                  (120, 120),
                                  (120,-120),
                                  (-120,-120)])

        # A couple regions of differing traction
        self.car = Ship(self.world)
        #gnd1 = self.world.CreateStaticBody()
        #fixture = gnd1.CreatePolygonFixture(box=(9, 7, (-20, 15),
        #math.radians(20)))

        #gnd2 = self.world.CreateStaticBody()
        #fixture = gnd2.CreatePolygonFixture(box=(4, 8, (5, 40),
        #math.radians(-40)))
        self.planes = \
            [Plane(world = self.world, position = self.car.body.worldCenter) for i in range(5)]
        self.plane_manager = PlaneManager(self.world, self.car, self.planes)


    def Keyboard(self, key):
        key_map = self.key_map
        if key in key_map:
            self.pressed_keys.add(key_map[key])
        else:
            super(ShipGame, self).Keyboard(key)

    def KeyboardUp(self, key):
        key_map = self.key_map
        if key in key_map:
            self.pressed_keys.remove(key_map[key])
        elif key in self.key_up_map:
            self.plane_manager.start_plane()
        else:
            super(ShipGame, self).KeyboardUp(key)
        

    def Step(self, settings):
        self.car.update(self.pressed_keys)
        self.plane_manager.update(settings)
        super(ShipGame, self).Step(settings)
        self.Print('Linear speed sqr: %s' % self.car.linear_speed_sqr)
        self.Print('Max angular speed: %s' % Plane.max_angular_speed)
        for plane in self.planes:
            self.Print('Linear speed sqr: %s' % plane.linear_speed_sqr)
            self.Print('Pos: %s' % plane.body.worldCenter)

if __name__ == "__main__":
     main(ShipGame)
