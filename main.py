#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import *
from plane     import *
import math

class PlaneManager(object):
	
	MAX_PLANES = 5	
	
	airplane_last = None
	
	def __init__(self, world, ship):
		self.Hangar = []
		self.Ship = ship
		
		# Timer for planes interval takeoff
		self._waiting_for_takeoff = False
		self._time_passed = 0
		
		for i in range(self.MAX_PLANES):
			self.Hangar.append(Plane(
				world,
				ship
			))
	
	def update(self, keys, settings):	
		self._waiting_time = 60
		if (self.airplane_last != None and self.airplane_last.inHangar()):
			self.airplane_last = None
		
		#Cooldown for launching airplanes == 1 sec.
		if (self._waiting_for_takeoff):
			self._time_passed += 1 / (settings.hz / 60.0)
			if (self._time_passed >= self._waiting_time):
				self._time_passed = 0
				self._waiting_for_takeoff = False

		for i in range(self.MAX_PLANES):			
			if ('h' in keys and self.Hangar[i].inHangar() and not self._waiting_for_takeoff and self.Ship.isGoingForward()):
				self.airplane_new = self.Hangar[i].appear(self.airplane_last)				
				self.airplane_last = self.airplane_new
				self._waiting_for_takeoff = True

			self.Hangar[i].update(settings)

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
		
	def isGoingForward(self):
		if (math.fabs(self.body.angularVelocity) < 0.1):
			return True
		return False

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
	description="Keys: accel = w, reverse = s, left = a, right = d, airplane launch = h"
	
	def __init__(self):
		super(ShipGame, self).__init__()
		
		# Top-down -- no gravity in the screen plane
		self.world.gravity = (0, 0)
		self.key_map = {Keys.K_w: 'up', Keys.K_s: 'down', Keys.K_a: 'left', Keys.K_d: 'right', Keys.K_h: 'h', }
		
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
		self.planes.update(self.pressed_keys, settings)
		super(ShipGame, self).Step(settings)
		
		self.Print('Linear speed sqr: %s' % self.car.linear_speed_sqr)
		for i in range(PlaneManager.MAX_PLANES):
			self.Print('=--------------------------------------------------- ')
			self.Print('Time in air for plane #%i is : %.1f sec.' % (i+1, self.planes.Hangar[i].getTimeInAir()))
			
if __name__=="__main__":
	 main(ShipGame)