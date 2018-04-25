from entity_base import EntityBase
from states.state_machine import StateMachine
from states.plane_states import *


class Plane(EntityBase, StateMachine):
    LIFTOFF_STATE = 'liftoff_state'
    SCOUTING_STATE = 'scouting_state'
    RETURNING_STATE = 'returning_state'
    LANDING_STATE = 'landing_state'

    def __init__(self, ship, game, vertices=None, density=None, position=(0, 0)):
        if density is None: density = game.step_settings.scoutPlaneDensity
        super(Plane, self).__init__(game, vertices, density, position)
        self.ship = ship
        self.ticks_in_air = 0

        # disable collisions
        fixture = self.body.fixtures[0]
        fixture.filterData.categoryBits = 0x0004
        fixture.filterData.maskBits = 0x0004
        fixture.filterData.groupIndex = -1
        self.body.position = ship.body.worldCenter
        self.body.angle = ship.body.angle

        StateMachine.__init__(self)

    @property
    def min_velocity(self):
        return self.game.settings.scoutMinVelocity

    @property
    def max_velocity(self):
        return self.game.settings.scoutVelocity

    @property
    def trajectory_radius(self):
        return self.game.settings.scoutVelocity / self.game.settings.scoutAngularVelocity

    def _vertices(self):
        return [
            (2.0, -4.0),
            (0.0, 4.0),
            (-2.4, -4.0),
        ]

    # probably it'd be better to do strategies
    # with single strategy instance for all planes
    def init_states(self):
        self._states[self.LIFTOFF_STATE] = LiftoffState(self)
        self._states[self.SCOUTING_STATE] = ScoutingState(self)
        self._states[self.RETURNING_STATE] = ReturningState(self)
        self._states[self.LANDING_STATE] = LandingState(self)

    @property
    def state_on_init(self):
        return self._states[self.LIFTOFF_STATE]

    def to_scouting_state(self):
        self.switch_state_to(self._states[self.SCOUTING_STATE])

    def to_returning_state(self):
        self.switch_state_to(self._states[self.RETURNING_STATE])

    def to_landing_state(self):
        self.switch_state_to(self._states[self.LANDING_STATE])

    def update(self, keys):
        self.ticks_in_air += 1
        self._state.update(keys)

    def normalize_speed(self):
        velocity = self.body.linearVelocity
        if velocity.length < self.min_velocity:
            self.body.linearVelocity.Normalize()
            self.body.linearVelocity *= self.min_velocity
        elif velocity.length > self.max_velocity:
            self.body.linearVelocity.Normalize()
            self.body.linearVelocity *= self.max_velocity

    def normalize_rotation(self):
        rotation_speed = self.body.angularVelocity
        max_rotation_speed = self.game.settings.scoutAngularVelocity
        if rotation_speed > max_rotation_speed:
            self.body.angularVelocity = max_rotation_speed
        elif rotation_speed < -max_rotation_speed:
            self.body.angularVelocity = -max_rotation_speed
