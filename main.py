#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import *
import math
import time

PlaneShape = [
	( 2.0, -4.0),
	( 0.0, 4.0),
	( -2.4, -4.0),
]


class Plane(object):
	vertices = PlaneShape
	LINEAR_ACCELERATION = 1.0
	LINEAR_MIN_SPEED = 5.0
	LINEAR_MAX_SPEED = 10.0
	ANGULAR_MAX_SPEED = 0.01
	MAX_FLIGHT_TIME = 20.0

	def __init__(self, world, vertices=None, density=0.1, position=(0, 0), ship=None):
		self.linear_speed_sqr = 0
		if vertices is None: vertices = Plane.vertices
		self.body = world.CreateDynamicBody(position=position)
		self.body.CreatePolygonFixture(vertices=vertices, density=density)
		self.body.angularDamping = 1.1
		self.body.linearDamping = 1.1
		# self.body.enabled = False
		self.status = 'aboard'  # flight, crashing?
		self.flight_time = 0.0
		self.ship = ship
		self.min_ship_dist = 10.0
		self.min_landing_dist = 5
		self.prev_plane = None
		self.min_prev_plane_dist = 10.0
		self.max_return_time = 30.0
		self.last_time = time.time()

	def takeoff(self, ship_dist, prev_plane, prev_plane_dist, max_return_time):
		self.status = 'flight'
		self.flight_time = 0.0
		self.max_return_time = max_return_time
		self.last_time = time.time()
		self.prev_plane = prev_plane
		self.min_ship_dist = ship_dist
		self.min_prev_plane_dist = prev_plane_dist
		print('Takeoff')

	def landing(self):
		self.status = 'aboard'
		self.prev_plane = None
		print('Landing')

	def return_to_ship(self):
		ship_position = self.ship.body.worldCenter
		ship_linear_velocity = self.ship.body.linearVelocity
		plane_position = self.body.worldCenter
		print(f'SHIP POSITION: {ship_position}, PLANE POSITION: {plane_position}')
		plane_linear_velocity = self.body.linearVelocity
		future_ship_position = ship_position
		future_ship_direction = future_ship_position - plane_position
		ship_distance = future_ship_direction.Normalize()
		print(f'SHIP DISTANCE: {ship_distance}')
		if ship_distance < self.min_landing_dist:
			self.landing()
		forward_direction = self.body.GetWorldVector((0, 1))  # Normalized by default
		mass = self.body.mass
		print(f'MASS: {mass}')
		if plane_linear_velocity.length < self.LINEAR_MIN_SPEED:  # Initial
			impulse = mass * self.LINEAR_MIN_SPEED * forward_direction
			self.body.ApplyLinearImpulse(impulse, self.body.worldCenter, True)
		elif plane_linear_velocity.length < self.LINEAR_MAX_SPEED:
			force = mass * self.LINEAR_ACCELERATION * forward_direction
			self.body.ApplyForceToCenter(force, True)
		else:  # Don't apply force if velocity > LINEAR_MAX_SPEED
			pass
		dot = forward_direction.dot(future_ship_direction)
		cross = forward_direction.cross(future_ship_direction)
		print(f'DOT: {dot}, CROSS: {cross}')
		if cross > 0:
			angular_speed = self.ANGULAR_MAX_SPEED * 0.5 * (1 - dot)
		else:
			angular_speed = -self.ANGULAR_MAX_SPEED * 0.5 * (1 - dot)
		print(f'ANGULAR SPEED: {angular_speed}')
		linear_speed_sqr = self.body.linearVelocity.lengthSquared
		angular_impulse = mass * linear_speed_sqr * angular_speed
		print(f'ANGULAR IMPULSE: {angular_impulse}')
		self.body.ApplyAngularImpulse(angular_impulse, True)

	def scout(self):
		ship_position = self.ship.body.worldCenter
		ship_linear_velocity = self.ship.body.linearVelocity
		plane_position = self.body.worldCenter
		plane_linear_velocity = self.body.linearVelocity
		forward_direction = self.body.GetWorldVector((0, 1))  # Normalized by default
		mass = self.body.mass
		print(f'LINEAR VELOCITY: {plane_linear_velocity.length}')
		print(f'MASS: {mass}')
		if plane_linear_velocity.length < self.LINEAR_MIN_SPEED:  # Initial
			impulse = mass * self.LINEAR_MIN_SPEED * forward_direction
			self.body.ApplyLinearImpulse(impulse, self.body.worldCenter, True)
		elif plane_linear_velocity.length < self.LINEAR_MAX_SPEED:
			force = mass * self.LINEAR_ACCELERATION * forward_direction
			self.body.ApplyForceToCenter(force, True)
		else:  # Don't apply force if velocity > LINEAR_MAX_SPEED
			pass
		future_ship_position = ship_position
		future_ship_direction = future_ship_position - plane_position
		ship_distance = future_ship_direction.Normalize()
		print(f'SHIP DISTANCE: {ship_distance}')
		dot = forward_direction.dot(future_ship_direction)
		cross = forward_direction.cross(future_ship_direction)
		print(f'DOT: {dot}, CROSS: {cross}')
		if cross > 0:
			angular_speed = self.ANGULAR_MAX_SPEED * 0.5 * (1 - dot)
		else:
			angular_speed = -self.ANGULAR_MAX_SPEED * 0.5 * (1 - dot)
		if ship_distance < self.min_ship_dist:
			angular_speed = -angular_speed
		print(f'ANGULAR SPEED: {angular_speed}')
		linear_speed_sqr = self.body.linearVelocity.lengthSquared
		angular_impulse = mass * linear_speed_sqr * angular_speed
		print(f'ANGULAR IMPULSE: {angular_impulse}')
		# Previous plane
		if self.prev_plane is not None:
			prev_plane_position = self.prev_plane.body.position
			prev_plane_linear_velocity = self.prev_plane.body.linearVelocity
			prev_plane_linear_position = prev_plane_position
			future_plane_direction = prev_plane_linear_position - plane_position
			prev_plane_distance = future_plane_direction.Normalize()
			print(f'SHIP DISTANCE: {prev_plane_distance}')
			dot = forward_direction.dot(future_plane_direction)
			cross = forward_direction.cross(future_plane_direction)
			print(f'DOT: {dot}, CROSS: {cross}')
			if cross > 0:
				angular_speed = self.ANGULAR_MAX_SPEED * 0.5 * (1 - dot)
			else:
				angular_speed = -self.ANGULAR_MAX_SPEED * 0.5 * (1 - dot)
			if prev_plane_distance < self.min_prev_plane_dist:
				angular_speed = -angular_speed
			print(f'ANGULAR SPEED: {angular_speed}')
			linear_speed_sqr = self.body.linearVelocity.lengthSquared
			angular_impulse = mass * linear_speed_sqr * angular_speed
			print(f'ANGULAR IMPULSE: {angular_impulse}')
			self.body.ApplyAngularImpulse(angular_impulse, True)

	def update(self, keys):
		if self.status == 'aboard':
			# TODO implement
			# self.body.position = self.ship.body.position
			pass
		elif self.status == 'flight':
			cur_time = time.time()
			delta_time = cur_time - self.last_time
			self.flight_time += delta_time
			self.last_time = cur_time
			max_time = min(self.MAX_FLIGHT_TIME, self.max_return_time)
			time_left = max_time - self.flight_time
			# if self.flight_time > self.MAX_FLIGHT_TIME:
			# 	print('Oops...')
			ship_position = self.ship.body.position
			ship_distance = ship_position.Normalize()
			ship_time = ship_distance / self.LINEAR_MIN_SPEED
			print(f'FLIGHT TIME: {self.flight_time}, SHIP TIME: {ship_time}, TIME LEFT: {time_left}')
			if time_left > ship_time:
				self.scout()
			else:
				self.return_to_ship()
		else:
			raise NotImplementedError(self.status)


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


	def __init__(self, world, vertices=None, density=0.1, position=(0, 0), planes=None):
		
		self.linear_speed_sqr = 0
		
		if vertices is None: vertices = Ship.vertices
		
		self.body = world.CreateDynamicBody(position=position)
		self.body.CreatePolygonFixture(vertices=vertices, density=density)
		self.body.angularDamping = 1.1
		self.body.linearDamping = 1.1

		self.planes = [] if planes is None else planes
		self.last_plane = None
		self.plane_key_down = False
		self.plane_key_up = False
	
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

		if 'plane' in keys:  # down
			self.plane_key_down = True
			self.plane_key_up = False
		else:  # 'plane' not in keys:
			if self.plane_key_down:  # down -> up
				self.plane_key_up = True
				self.plane_key_down = False
			else:  # nothing
				self.plane_key_up = False
				self.plane_key_down = False
		if self.plane_key_up:
			new_plane_i = -1
			for i, p in enumerate(self.planes):
				if self.planes[i].status == 'aboard':
					p.takeoff(
						prev_plane=self.last_plane,
						prev_plane_dist=50.0,
						ship_dist=50.0,
						max_return_time=30.0)
					new_plane_i = i
					self.last_plane = p
					break
			if new_plane_i >= 0:
				print(f'Plane {new_plane_i + 1} is taking off')
			else:
				print(f'No planes available at the moment!')


class ShipGame (Framework):
	name="Ship Game"
	description="Keys: accel = w, reverse = s, left = a, right = d"

	def __init__(self):
		super(ShipGame, self).__init__()
		
		# Top-down -- no gravity in the screen plane
		self.world.gravity = (0, 0)
		self.key_map = {Keys.K_w: 'up', Keys.K_s: 'down', Keys.K_a: 'left',
						Keys.K_d: 'right', Keys.K_h: 'plane'}
		
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
		gnd1 = self.world.CreateStaticBody()
		fixture = gnd1.CreatePolygonFixture(box=(9, 7, (-20, 15), math.radians(20)))
		
		gnd2 = self.world.CreateStaticBody()
		fixture = gnd2.CreatePolygonFixture(box=(4, 8, (5, 40), math.radians(-40)))
		
		# Kill me
		plane_test = self.world.CreateStaticBody(position=(10,0))
		fixture = plane_test.CreatePolygonFixture(vertices=PlaneShape)

		# Planes
		num_planes = 5
		self.planes = [Plane(self.world, ship=self.car) for _ in range(num_planes)]
		self.car.planes = self.planes
	
	
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
		for p in self.planes:
			p.update(self.pressed_keys)
		super(ShipGame, self).Step(settings)
		self.Print('Linear speed sqr: %s' % self.car.linear_speed_sqr)

if __name__=="__main__":
	 main(ShipGame)

