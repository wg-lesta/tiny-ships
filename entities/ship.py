from entity_base import EntityBase


class Ship(EntityBase):
    LINEAR_SPEED = 50
    ANGULAR_SPEED = 0.1
    ANGULAR_MAX_IMPULSE = 1.5

    def _vertices(self):
        return [(1.5, 0.0),
                (3.0, 5.0),
                (2.8, 11.0),
                (1.0, 20.0),
                (-1.0, 20.0),
                (-2.8, 11.0),
                (-3.0, 5.0),
                (-1.5, 0.0),
                ]

    def __init__(self, game, vertices=None, density=0.1, position=(0, 0)):
        super(Ship, self).__init__(game, vertices, density, position)
        self.linear_speed_sqr = 0

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
