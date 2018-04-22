#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math

from entities import *
from framework import *

class ShipGame (Framework):
	name="Ship Game"
	description="Keys: accel = w, reverse = s, left = a, right = d"

	instance = None

	def __init__(self):
		super(ShipGame, self).__init__()
		ShipGame.instance = self
		self._entities = []
		self.step_settings = None

		# Top-down -- no gravity in the screen plane
		self.world.gravity = (0, 0)
		self.key_map = {Keys.K_w: 'up', Keys.K_s: 'down', Keys.K_a: 'left', Keys.K_d: 'right', Keys.K_h: 'send_plane'}
		
		# Keep track of the pressed keys
		self.pressed_keys = set()
		
		# The walls
		boundary = self.world.CreateStaticBody(position=(0, 20))
		boundary.CreateEdgeChain([(-120,-120),
								  (-120, 120),
								  ( 120, 120),
								  ( 120,-120),
								  (-120,-120)]
								 )
		
		# A couple regions of differing traction
		self.create_ground()
		Carrier(self)

	def create_ground(self):
		gnd = self.world.CreateStaticBody()
		gnd.CreatePolygonFixture(box=(9, 7, (-20, 15), math.radians(20)))

		gnd = self.world.CreateStaticBody()
		gnd.CreatePolygonFixture(box=(4, 8, (5, 40), math.radians(-40)))

	def add_entity(self, entity):
		self._entities.append(entity)

	def remove_entity(self, entity):
		if entity in self._entities:
			self._entities.remove(entity)

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
		else:
			super(ShipGame, self).KeyboardUp(key)

	def Step(self, settings):
		self.step_settings = settings
		for _entity in self._entities:
			_entity.update(self.pressed_keys)

		super(ShipGame, self).Step(settings)

if __name__=="__main__":
	 main(ShipGame)
