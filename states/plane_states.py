#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import b2Vec2

class PlaneState:
    def __init__(self, plane):
        self.plane = plane

    def update(self, keys):
        self.trottle()
        self.plane.normalize_speed()
        self.plane.normalize_rotation()

    def trottle(self):
        self.body.ApplyForceToCenter(self.front_vector * self.settings.scoutAcceleration, True)

    def on_state_enable(self):
        pass

    def on_state_disable(self):
        pass

    def change_speed(self, delta_vector):
        self.body.linearVelocity += delta_vector

    @property
    def front_vector(self):
        return self.body.GetWorldVector((0, 1))

    @property
    def settings(self):
        return self.plane.game.step_settings

    @property
    def body(self):
        return self.plane.body


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
        target_force = self.front_vector * self.settings.scoutVelocity
        target_force -= self.body.linearVelocity
        self.force_per_tick = self.front_vector * (target_force.length / self.liftoff_ticks)

    # ignoring planes acceleration here
    def update(self, keys):
        self.ticks += 1
        self.change_speed(self.force_per_tick)
        if self.ticks >= self.liftoff_ticks:
            self.plane.to_scouting_state()


class ScoutingState(PlaneState):

    def on_state_enable(self):
        PlaneState.on_state_enable(self)
        self.body.angularVelocity = self.settings.scoutAngularVelocity

    def update(self, keys):
        # 1) найдем текущий радиус
        radius = (self.body.position - self.plane.ship.body.position).length

        # 2) высчитаем сколько нужно к заданной скорости добавить(если наш радиус больше)
        #    или отнять (если наш радиус меньше)
        delta_velocity = self.delta_angular_velocity(radius)
        # 3) применим угловую скорость
        self.body.angularVelocity += delta_velocity
        PlaneState.update(self, keys)
        self.check_and_return_to_ship()

    def delta_angular_velocity(self, current_radius):
        delta_radius = current_radius - self.plane.trajectory_radius
        if delta_radius < 0:
            return -self.body.angularVelocity
        else:
            angular_velocity_at_radius = self.settings.scoutVelocity / current_radius
            return self.settings.scoutAngularVelocity - angular_velocity_at_radius

    def check_and_return_to_ship(self):
        if self.should_return():
            self.plane.to_returning_state()

    def should_return(self):
        return False
        # TODO: логика подсчета времени на обратный путь
        return self.plane.ticks_in_air >= self.settings.scoutTicksInAir / 2


class ReturningState(PlaneState):
    pass


class LandingState(PlaneState):
    pass
