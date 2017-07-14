#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import *
import math

CATEGORY_WATER = 0x0001
CATEGORY_AIR = 0x0002

class Plane(object):
	vertices = [( 2.0, -4.0),
				( 0.0, 4.0),
				( -2.0, -4.0),
				]

	fixture = b2FixtureDef(
		shape=b2PolygonShape(vertices=vertices),
		density=0.1,
		filter=b2Filter(
			groupIndex=0,			# do not collide between this group
			maskBits=CATEGORY_AIR,	# do not collide between air and water
		)
	)	
	
	def __init__(self, manager, world):
		self.Manager = manager
		self.World = world
		
	def appear(self, position=(0, 0), angle=0):
		self.body = self.World.CreateDynamicBody(
			position=(position + (
				10 * math.cos(angle + math.pi / 2),
				10 * math.sin(angle + math.pi / 2)
			)),
			angle=angle,
			fixtures=Plane.fixture,
		)
		
	def disappear(self):
		self.body = None
		
	#def update(self, keys):
	
class PlaneManager(object):
	
	MAX_PLANES = 5
	
	airplane_is_set = False
	airplane_in_air = 0
	airplane_last = None
	
	def __init__(self, world, ship):
		self.Hangar = []
		self.World = world
		self.Ship = ship
		
		for i in range(self.MAX_PLANES):
			self.Hangar.append(Plane(self, self.World))
	
	def update(self, keys):
		if ('airplane' in keys):
			if (not self.airplane_is_set and self.airplane_in_air < self.MAX_PLANES):
				self.airplane_new = self.Hangar[self.airplane_in_air].appear(
					self.Ship.body.position,
					self.Ship.body.angle
				)				
				self.airplane_last = self.airplane_new
				self.airplane_in_air += 1
			self.airplane_is_set = True
		else:
			self.airplane_is_set = False
	
class Ship(object):
	vertices = [( 1.5, 0.0),
				( 3.0, 5.0),
				( 2.8, 11.0),
				( 1.0,20.0),
				(-1.0,20.0),
				(-2.8, 11.0),
				(-3.0, 5.0),
				(-1.5, 0.0),
				]
	
	LINEAR_SPEED = 50
	ANGULAR_SPEED = 0.1
	ANGULAR_MAX_IMPULSE = 1.5

	def __init__(self, world, vertices=None, density=0.1, position=(0, 0)):
		
		self.linear_speed_sqr = 0
		
		if vertices is None: vertices = Ship.vertices
		
		self.body = world.CreateDynamicBody(position=position)
		self.body.CreatePolygonFixture(vertices=vertices, density=density)
		self.body.angularDamping = 1.1
		self.body.linearDamping = 1.1
	
	def update_linear(self, throttle):
		direction = self.body.GetWorldVector((0, 1))
		self.body.ApplyForceToCenter(self.LINEAR_SPEED * throttle * direction, True)
		self.linear_speed_sqr = self.body.linearVelocity.lengthSquared

	def update_angular(self, turn):
		angular_impulse = self.ANGULAR_SPEED * self.linear_speed_sqr
		if angular_impulse > self.ANGULAR_MAX_IMPULSE: angular_impulse = self.ANGULAR_MAX_IMPULSE
		self.body.ApplyAngularImpulse( angular_impulse * turn, True )

	def update(self, keys):
		
		throttle = 0
		if 'up' in keys: throttle += 1
		if 'down' in keys: throttle -= 1
		self.update_linear(throttle)
		
		turn = 0
		if 'left' in keys: turn += 1
		if 'right' in keys: turn -= 1
		self.update_angular(turn)

class ShipGame (Framework):
	name="Ship Game"
	description="Keys: accel = w, reverse = s, left = a, right = d, airplane = h"
	
	def __init__(self):
		super(ShipGame, self).__init__()
		
		# Top-down -- no gravity in the screen plane
		self.world.gravity = (0, 0)
		self.key_map = {Keys.K_w: 'up', Keys.K_s: 'down', Keys.K_a: 'left', Keys.K_d: 'right', Keys.K_h: 'airplane', }
		
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
		self.car = Ship(self.world)
		self.planes = PlaneManager(self.world, self.car)
		
		gnd1 = self.world.CreateStaticBody()
		fixture = gnd1.CreatePolygonFixture(box=(9, 7, (-20, 15), math.radians(20)))
		
		gnd2 = self.world.CreateStaticBody()
		fixture = gnd2.CreatePolygonFixture(box=(4, 8, (5, 40), math.radians(-40)))	
	
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
		self.car.update(self.pressed_keys)
		self.planes.update(self.pressed_keys)
		super(ShipGame, self).Step(settings)
		
		self.Print('Linear speed sqr: %s' % self.car.linear_speed_sqr)

if __name__=="__main__":
	 main(ShipGame)