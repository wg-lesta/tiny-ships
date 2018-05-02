#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import b2Vec2
from math import *

class PlaneState:
    TWO_PI = pi * 2
    STRAIGHT_ANGLE = pi / 2

    def __init__(self, plane):
        self.plane = plane

    def update(self, keys):
        self.plane.game.Print("Vel: %.1f. Time: %.1f s." % (self.body.linearVelocity.length, (self.plane.ticks_in_air / self.settings.hz)))

    def on_state_enable(self):
        pass

    def on_state_disable(self):
        pass

    def change_speed(self, delta_vector):
        self.body.linearVelocity += delta_vector

    def get_vector_to_point(self, point):
        current_pos = self.body.position
        vector = b2Vec2(point.x - current_pos.x, point.y - current_pos.y)
        vector.Normalize()
        return vector

    @property
    def settings(self):
        return self.plane.game.step_settings

    @property
    def body(self):
        return self.plane.body

    @property
    def ship_center(self):
        return self.plane.ship.body.position + self.plane.ship.body.GetWorldVector((0, 1)) * 10

    @property
    def begin_landing_point(self):
        ship = self.plane.ship.body
        return ship.position + ship.GetWorldVector((0, -1)) * (self.plane.trajectory_radius / 2)


class LiftoffState(PlaneState):

    liftoff_ticks = 30

    def __init__(self, plane):
        PlaneState.__init__(self, plane)
        self.ticks = 0

        self.force_per_tick = None

    def on_state_enable(self):
        # calculate acceleration per frame for reaching plane speed at liftoff end
        # this formula includes ships drifting effect
        self.body.linearVelocity = self.plane.ship.body.linearVelocity
        target_force = self.plane.front_vector * self.settings.scoutVelocity
        target_force -= self.body.linearVelocity
        self.force_per_tick = self.plane.front_vector * (target_force.length / self.liftoff_ticks)

    # ignoring planes acceleration here
    def update(self, keys):
        self.ticks += 1
        self.plane.move_to(self.force_per_tick)
        PlaneState.update(self, keys)
        if self.ticks >= self.liftoff_ticks:
            self.plane.to_scouting_state()


class ScoutingState(PlaneState):
    RETURN_BUFFER_TIME = 5  # seconds

    def on_state_enable(self):
        PlaneState.on_state_enable(self)
        self.body.angularVelocity = self.settings.scoutAngularVelocity

    def update(self, keys):
        # 1) найдем текущий радиус
        radius = (self.body.position - self.ship_center).length

        # 2) высчитаем требуемый вектор движения
        angle = self.desired_flight_angle(radius)
        vector = b2Vec2(cos(angle), sin(angle))
        vector += self.plane.evading_planes_vector
        self.plane.move_to(vector)

        # 3) проверим, что пора идти на посадку
        PlaneState.update(self, keys)
        self.check_and_return_to_ship()

    def desired_flight_angle(self, current_radius):
        require_radius = self.plane.trajectory_radius
        pos = self.body.position
        ship_pos = self.plane.ship.body.position
        x = ship_pos.x - pos.x
        y = ship_pos.y - pos.y
        angle_to_ship = atan2(y, x)

        if current_radius > require_radius * 2:
            # fly directly to ship
            return angle_to_ship

        delta = (current_radius - require_radius) / float(require_radius)
        return angle_to_ship - self.STRAIGHT_ANGLE + self.STRAIGHT_ANGLE * delta

    def check_and_return_to_ship(self):
        if self.should_return():
            self.plane.to_returning_state()

    def should_return(self):
        # расчет времени возврата по прямой.
        # пренебрежем временем на разворот, добавим несколько итераций в расчеты
        current_speed = self.body.linearVelocity.length
        distance = self.body.position - self.begin_landing_point
        time_to_return = distance.length / current_speed + self.RETURN_BUFFER_TIME
        time_to_return *= self.settings.hz
        return (self.plane.ticks_in_air + time_to_return) > self.settings.scoutTicksInAir


class ReturningState(PlaneState):

    def update(self, keys):
        current_pos = self.body.position
        destination = self.begin_landing_point
        vector = self.get_vector_to_point(destination) + self.plane.evading_planes_vector
        self.plane.move_to(vector)
        PlaneState.update(self, keys)
        if (current_pos - destination).length < 10:
            self.plane.to_landing_state()


class LandingState(PlaneState):

    @property
    def distance_to_ship(self):
        return (self.body.position - self.ship_center).length

    @property
    def has_landed(self):
        return self.distance_to_ship < 3

    def update(self, keys):
        destination = self.ship_center
        current_pos = self.body.position

        angle_to_ship = atan2(destination.y - current_pos.y, destination.x - current_pos.x)
        ship_angle = self.plane.ship.body.angle + pi/2
        require_angle = angle_to_ship + (angle_to_ship - ship_angle)
        destination = b2Vec2(cos(require_angle), sin(require_angle)) * (self.ship_center - current_pos).length
        self.plane.move_to(destination)
        PlaneState.update(self, keys)
        if self.has_landed:
            self.plane.ship.destroy_plane(self.plane)
