#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import *
import math
import random
import time


def relative_angle(x0, y0, x1, y1):
	return math.atan2(y1 - y0, x1 - x0) * 180 / math.pi


def to_radian(alpha):
	return alpha * math.pi / 180


class Plane(object):
	PlaneShape = [
		(2.0, -4.0),
		(0.0, 4.0),
		(-2.4, -4.0),
	]

	DIST_BETWEEN_PLANES = 20.0

	def __init__(self, world, squadron, min_speed=30, max_speed=70, max_angular=1.5):
		self.body = world.CreateDynamicBody(position=(0, 0))
		self.world = world
		self.squadron = squadron
		self.time_start = 0.0

		self.min_linear_speed = min_speed
		self.max_linear_speed = max_speed
		self.max_angular_speed = max_angular
		self.available = True

	def start_plane(self):
		if self.body:
			self.world.DestroyBody(self.body)
			self.body = None
		self.body = self.world.CreateDynamicBody(position=self.squadron.ship.body.position)
		self.body.CreatePolygonFixture(vertices=Plane.PlaneShape, density=0.1)
		self.available = False
		self.time_start = time.time()

	def to_hangar(self):
		if self.body:
			self.world.DestroyBody(self.body)
			self.body = None
		self.available = True

	def is_landing_time(self, ret_time):
		return time.time() - self.time_start >= ret_time

	def update_linear(self, throttle):
		pass

	def update(self):
		pass


class Squadron(object):
	planes = []
	plane_count = 0
	current_plane_id = 0

	MIN_DISTANCE = 25.0
	TIME_TO_RETURN = 20.0

	def __init__(self, ship, capacity=5):
		self.ship = ship
		self.capacity = capacity
		self.planes_in_air = 0

	def add_plane(self, plane):
		if self.plane_count + 1 <= self.capacity:
			self.planes.append(plane)
			self.plane_count += 1

	def next_plane_id(self):
		# если есть хотя бы один самолет и он находится на борту
		if self.plane_count != 0 and self.planes_in_air < self.plane_count:
			plain_id = self.current_plane_id % self.plane_count
			while not self.planes[plain_id].available:
				self.current_plane_id += 1
				plain_id = self.current_plane_id % self.plane_count
			return plain_id

	def next_plane_flight(self):
		plane_id = self.next_plane_id()
		if plane_id is not None and self.planes[plane_id].available:
			self.planes[plane_id].start_plane()
			self.planes_in_air += 1

	def next_landing_plane(self, plane):
		plane.to_hangar()
		self.planes_in_air -= 1

	def update_squadron(self):
		if self.planes_in_air > 0:
			for plane in self.planes:
				if not plane.available:
					distance = b2DistanceSquared(plane.body.position, self.ship.body.position)
					if plane.is_landing_time(Squadron.TIME_TO_RETURN):# and distance < Squadron.MIN_DISTANCE:
						self.next_landing_plane(plane)
					plane.update()


class Ship(object):
	vertices = [(1.5, 0.0),
				(3.0, 5.0),
				(2.8, 11.0),
				(1.0, 20.0),
				(-1.0, 20.0),
				(-2.8, 11.0),
				(-3.0, 5.0),
				(-1.5, 0.0),
				]

	LINEAR_SPEED = 50
	ANGULAR_SPEED = 0.1
	ANGULAR_MAX_IMPULSE = 1.5

	def __init__(self, world, vertices=None, density=0.1, position=(30, 30)):
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
	description = "Keys: accel = w, reverse = s, left = a, right = d, plane = h"

	def __init__(self):
		super(ShipGame, self).__init__()

		# Top-down -- no gravity in the screen plane
		self.world.gravity = (0, 0)
		self.key_map = {Keys.K_w: 'up', Keys.K_s: 'down', Keys.K_a: 'left', Keys.K_d: 'right', Keys.K_h: 'plane'}

		# Keep track of the pressed keys
		self.pressed_keys = set()

		#
		self.launch_plane_flag = False

		# The walls
		boundary = self.world.CreateStaticBody(position=(0, 20))
		boundary.CreateEdgeChain([(-120, -120),
								  (-120, 120),
								  (120, 120),
								  (120, -120),
								  (-120, -120)]
								 )

		# A couple regions of differing traction
		self.car = Ship(self.world)

		squadron_size = 5
		self.squadron = Squadron(self.car, squadron_size)

		for i in range(squadron_size):
			self.squadron.add_plane(Plane(self.world, self.squadron))

		gnd1 = self.world.CreateStaticBody()
		fixture = gnd1.CreatePolygonFixture(box=(9, 7, (-20, 15), math.radians(20)))

		gnd2 = self.world.CreateStaticBody()
		fixture = gnd2.CreatePolygonFixture(box=(4, 8, (5, 40), math.radians(-40)))

		# Kill me
		plane_test = self.world.CreateStaticBody(position=(10, 0))
		fixture = plane_test.CreatePolygonFixture(vertices=Plane.PlaneShape)

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
			if key_map[key] == 'plane':
				self.launch_plane_flag = True  # говорим, что хотим запустить самолет
		else:
			super(ShipGame, self).KeyboardUp(key)

	def Step(self, settings):
		if self.launch_plane_flag:
			self.squadron.next_plane_flight()
			self.launch_plane_flag = False

		self.car.update(self.pressed_keys)
		self.squadron.update_squadron()
		super(ShipGame, self).Step(settings)
		self.Print('Linear speed sqr: %s' % self.car.linear_speed_sqr)
		for plane in self.squadron.planes:
			if not plane.available:
				ret_time = plane.time_start + Squadron.TIME_TO_RETURN - time.time()
				if ret_time < 0.0:
					ret_time = 0.0
				self.Print('Plane #%s Time returns: %.4s' % ((self.squadron.planes.index(plane) + 1), (ret_time)))


if __name__ == "__main__":
	main(ShipGame)
