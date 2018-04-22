from entity_base import EntityBase


class Plane(EntityBase):
    def __init__(self, ship, game, vertices=None, density=0.1, position=(0, 0)):
        super(Plane, self).__init__(game, vertices, density, position)
        self.ship = ship
        self.body.angle = ship.body.angle

        # disable collisions
        fixture = self.body.fixtures[0]
        fixture.filterData.categoryBits = 0x0004
        fixture.filterData.maskBits = 0x0004
        fixture.filterData.groupIndex = -1
        self.body.position = ship.body.worldCenter


    def _vertices(self):
        return [
            (2.0, -4.0),
            (0.0, 4.0),
            (-2.4, -4.0),
        ]

    # def set_collision_group(self, fixture):
    #     fixture.gropIndex = -1
    #     fixture.categoryBits = 0x0002
    #     fixture.maskBits = 0x0004

    def update(self, keys):
        pass
