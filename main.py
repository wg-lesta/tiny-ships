#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import *
import math
import time

g_is_key_up = False


class BoidsSystem:
    def __init__(self):
        self._boids = {}

    def add_boid(self, boid):
        if boid:
            self._boids[boid.name] = boid
            return True
        return False

    def update(self, keys):
        flagship = self._boids.get("flag_ship", None)
        if flagship:
            flagship.update(keys)

            if 'flight' in keys:
                global g_is_key_up
                if g_is_key_up:
                    for plane in self._boids.itervalues():
                        if isinstance(plane, ScoutPlane):
                            if not plane.takeoff:
                                plane.flight()
                                g_is_key_up = False
                                break

            transform = b2Transform(b2Vec2(flagship.world_centre), b2Rot(flagship.angle))
            for boid in self._boids.itervalues():
                if isinstance(boid, ScoutPlane):
                    boid.update(self._boids, transform)


class BaseDynamicEntity(object):
    def __init__(self, name, world, vertices, density, position):
        self._name = name
        self.linear_speed_sqr = 0

        self.body = world.CreateDynamicBody(position=position)
        fixture = self.body.CreatePolygonFixture(vertices=vertices, density=density)
        fixture.filterData.groupIndex = -8

    @property
    def name(self):
        return self._name

    @property
    def linear_velocity(self):
        return self.body.linearVelocity

    @property
    def world_centre(self):
        return self.body.worldCenter

    @property
    def position(self):
        return self.body.position

    @position.setter
    def position(self, value):
        self.body.position = value

    @property
    def angle(self):
        return self.body.angle

    @angle.setter
    def angle(self, value):
        self.body.angle = value

    @property
    def transform(self):
        return self.body.transform


class ScoutPlane(BaseDynamicEntity):
    vertices = [
        (2.0,  -4.0),
        (0.0,   4.0),
        (-2.0, -4.0), ]

    LINEAR_SPEED = 150
    ANGULAR_SPEED = 1.5
    #ANGULAR_MAX_IMPULSE = 1.5
    ANGULAR_MAX_IMPULSE = 0.1
    MAX_IMPULSE = 3

    MAX_FUEL = 30

    def __init__(self, name, world, vertices=None, density=0.1, position=(0, 0)):
        if vertices is None:
            vertices = ScoutPlane.vertices

        BaseDynamicEntity.__init__(self, name, world, vertices, density, position)
        self.body.angularDamping = 3.1
        self.body.linearDamping = 1.1

        self._takeoff = False
        self._time_start = 0
        self._acceleration = b2Vec2(0, 0)

    @property
    def takeoff(self):
        return self._takeoff

    def flight(self):
        if not self._takeoff:
            self._time_start = time.time()
            time.clock()
            self._takeoff = True

    def landing(self):
        self._takeoff = False
        self._time_start = 0
        self._acceleration = b2Vec2(0, 0)

    def update_linear(self, acceleration):
        self.body.ApplyLinearImpulse(acceleration, self.position, True)
        self.linear_speed_sqr = self.body.linearVelocity.lengthSquared

    def update_angular(self):
        angular_force = self.ANGULAR_SPEED * self.linear_speed_sqr
        if angular_force > self.ANGULAR_MAX_IMPULSE:
            angular_force = self.ANGULAR_MAX_IMPULSE
        self.body.ApplyAngularImpulse(angular_force, True)

    def forcing(self, impulse):
        direction = self.body.GetWorldVector((0, 1))
        self.body.ApplyLinearImpulse(direction * impulse, self.body.position, True)

    def get_fuel(self):
        fuel = self.MAX_FUEL - (time.time() - self._time_start)
        time.clock()
        if fuel < 0:
            fuel = 0
        return fuel

    def flock(self, boids):
        separation = self.separate(boids)
        align = self.align(boids)
        cohesion = self.cohesion(boids)

        separation *= 1.5
        align *= 1.0
        cohesion *= 1.1

        acceleration = separation + align + cohesion

        self.update_linear(acceleration)
        self.update_angular()

    def seek(self, target):
        desired = target - self.position
        if desired.length == 0:
            return b2Vec2(0, 0)

        desired.Normalize()
        desired *= self.LINEAR_SPEED

        steer = desired - self.linear_velocity

        if steer.length > self.MAX_IMPULSE:
            steer.Normalize()
            steer *= self.MAX_IMPULSE
        return steer

    def steer_to_base(self, target):
        desired = target - self.position
        desired *= 0.05
        self.body.ApplyLinearImpulse(desired, self.position, True)
        distance = b2DistanceSquared(target, self.position)
        if distance < 5:
            self.landing()

    def separate(self, neighbours):
        separation = 2500
        steer = b2Vec2(0, 0)
        count = 0
        for boid in neighbours.itervalues():
            distance = b2DistanceSquared(self.position, boid.position)
            '''if isinstance(boid, FlagShip):
                separation = 300'''
            if (distance > 0) and (distance < separation):
                diff = self.position - boid.position
                diff.Normalize()
                diff *= 1.0/distance
                steer += diff
                count += 1

        if count > 0:
            steer *= 1.0/count

        if steer.length > 0:
            steer.Normalize()
            steer *= self.LINEAR_SPEED
            steer -= self.linear_velocity
            if steer.length > self.MAX_IMPULSE:
                steer.Normalize()
                steer *= self.MAX_IMPULSE
        return steer

    def align(self, neighbours):
        neighbordist = 10000
        steer = b2Vec2(0, 0)
        count = 0

        for boid in neighbours.itervalues():
            distance = b2DistanceSquared(self.position, boid.position)
            if (distance > 0) and (distance < neighbordist):
                steer += boid.linear_velocity
                count += 1

        if count > 0:
            steer *= 1.0/count

        if steer.length > 0:
            steer.Normalize()
            steer *= self.LINEAR_SPEED
            steer -= self.linear_velocity
            if steer.length > self.MAX_IMPULSE:
                steer.Normalize()
                steer *= self.MAX_IMPULSE
        return steer

    def cohesion(self, neighbours):
        neighbordist = 100000
        sum = b2Vec2(0, 0)
        count = 0
        for boid in neighbours.itervalues():
            distance = b2DistanceSquared(self.position, boid.position)
            if (distance > 0) and (distance < neighbordist):
                sum += boid.position
                count += 1

        if count > 0:
            sum *= 1.0/count
            return self.seek(sum)
        else:
            return b2Vec2(0, 0)

    def update(self, boids, transform):
        if self.takeoff:
            if self.get_fuel() > 29:
                self.forcing(3)
            elif self.get_fuel() < 15:
                self.steer_to_base(transform.position)
            else:
                self.flock(boids)
        else:
            self.position = transform.position
            self.angle = transform.angle


class FlagShip(BaseDynamicEntity):
    vertices = [(1.5, 0.0),
                (3.0, 5.0),
                (2.8, 11.0),
                (1.0, 20.0),
                (-1.0, 20.0),
                (-2.8, 11.0),
                (-3.0, 5.0),
                (-1.5, 0.0), ]

    LINEAR_SPEED = 50
    ANGULAR_SPEED = 0.1
    ANGULAR_MAX_IMPULSE = 1.5

    MAX_PLANE = 5

    def __init__(self, name, world, vertices=None, density=0.1, position=(0, 0)):
        if vertices is None:
            vertices = FlagShip.vertices
        BaseDynamicEntity.__init__(self, name, world, vertices, density, position)
        self._planes = {}
        self.body.angularDamping = 1.1
        self.body.linearDamping = 1.1

    def update_linear(self, throttle):
        direction = self.body.GetWorldVector((0, 1))
        self.body.ApplyForceToCenter(self.LINEAR_SPEED * throttle * direction, True)
        self.linear_speed_sqr = self.body.linearVelocity.lengthSquared

    def update_angular(self, turn):
        angular_impulse = self.ANGULAR_SPEED * self.linear_speed_sqr
        if angular_impulse > self.ANGULAR_MAX_IMPULSE:
            angular_impulse = self.ANGULAR_MAX_IMPULSE
        self.body.ApplyAngularImpulse(angular_impulse * turn, True)

    def update(self, keys):
        throttle = 0
        if 'up' in keys:
            throttle += 1
        if 'down' in keys:
            throttle -= 1
        self.update_linear(throttle)

        turn = 0
        if 'left' in keys:
            turn += 1
        if 'right' in keys:
            turn -= 1
        self.update_angular(turn)


class ShipGame(Framework):
    name = "Ship Game"
    description = "Keys: accel = w, reverse = s, left = a, right = d, flight = h"

    def __init__(self):
        super(ShipGame, self).__init__()

        # Top-down -- no gravity in the screen plane
        self.world.gravity = (0, 0)
        self.key_map = {Keys.K_w: 'up', Keys.K_s: 'down', Keys.K_a: 'left', Keys.K_d: 'right', Keys.K_h: 'flight'}

        # Keep track of the pressed keys
        self.pressed_keys = set()

        # The walls
        boundary = self.world.CreateStaticBody(position=(0, 20))
        boundary.CreateEdgeChain([(-120,-120),
                                  (-120, 120),
                                  ( 120, 120),
                                  ( 120,-120),
                                  (-120,-120)] )

        # A couple regions of differing traction

        gnd1 = self.world.CreateStaticBody()
        fixture = gnd1.CreatePolygonFixture(box=(9, 7, (-20, 15), math.radians(20)))

        gnd2 = self.world.CreateStaticBody()
        fixture = gnd2.CreatePolygonFixture(box=(4, 8, (5, 40), math.radians(-40)))

        self._boidssystem = BoidsSystem()

        for b in [ScoutPlane('scout_'+str(i), self.world) for i in range(5)]:
            self._boidssystem.add_boid(b)

        self.ship1 = FlagShip('flag_ship', self.world)

        self._boidssystem.add_boid(self.ship1)

    def Keyboard(self, key):
        key_map = self.key_map
        if key in key_map:
            self.pressed_keys.add(key_map[key])
        else:
            super(ShipGame, self).Keyboard(key)

    def KeyboardUp(self, key):
        global g_is_key_up
        g_is_key_up = True
        key_map = self.key_map
        if key in key_map:
            self.pressed_keys.remove(key_map[key])
        else:
            super(ShipGame, self).KeyboardUp(key)

    def Step(self, settings):
        self._boidssystem.update(self.pressed_keys)

        super(ShipGame, self).Step(settings)
        self.Print('Linear speed sqr: %s' % self.ship1.linear_speed_sqr)
        time.sleep(0.001)

if __name__ == "__main__":
     main(ShipGame)

