class EntityBase(object):
    def __init__(self, game, vertices=None, density=0.1, position=(0, 0)):
        if vertices is None: vertices = self._vertices()
        self.game = game

        self.body = game.world.CreateDynamicBody(position=position)
        self.body.CreatePolygonFixture(vertices=vertices, density=density)

        self.body.angularDamping = 1.1
        self.body.linearDamping = 1.1

        game.add_entity(self)

    def _vertices(self):
        return []

    def update(self, keys):
        pass
