#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import *
import math
import time


class Plane(object):
	vertices = [
		(2.0, -4.0),
		(0.0, 4.0),
		(-2.4, -4.0)
	]
	LINEAR_ACCELERATION = 20.0
	LINEAR_MIN_SPEED = 15.0
	LINEAR_MAX_SPEED = 30.0
	ANGULAR_MAX_SPEED = 1.0
	MAX_FLIGHT_TIME = 20.0

	class Strategy(object):
		def __init__(self, plane):
			self.plane = plane

	class Aboard(Strategy):
		def __init__(self, plane):
			super().__init__(plane)

		def update(self):
			ship_position = self.plane.ship.body.worldCenter
			if self.plane.body.active:
				self.plane.body.active = False
			self.plane.body.transform = (ship_position, 0)

	class Scout(Strategy):
		def __init__(
			self, plane, planes=None,
			min_ship_dist=50.0, min_plane_dist=10.0, max_return_time=30.0
		):
			super().__init__(plane)
			self.min_ship_dist = min_ship_dist
			self.min_plane_dist = min_plane_dist
			self.max_return_time = max_return_time
			self.planes = [] if planes is None else planes

		def update(self):
			if self.plane.status == 'aboard':
				self.plane.takeoff()
				return
			if not self.plane.body.active:
				self.plane.body.active = True
			# RETURN LOGIC
			ship_position = self.plane.ship.body.worldCenter
			position = self.plane.body.worldCenter
			ship_direction = ship_position - position
			ship_distance = ship_direction.Normalize()
			time_to_ship = ship_distance / self.plane.LINEAR_MIN_SPEED
			max_time = min(self.plane.MAX_FLIGHT_TIME, self.max_return_time)
			time_left = max_time - self.plane.status_time
			if self.plane.status_time > self.plane.MAX_FLIGHT_TIME:
				print('Oops...')  # Probably plane is crushed...
			is_return = False if time_left > time_to_ship else True
			if is_return and ship_distance < self.plane.min_landing_dist:
				self.plane.landing()
				return
			# LINEAR
			self.plane.update_linear_speed()
			# ANGULAR
			mass = self.plane.body.mass
			ship_distance = self.min_ship_dist if not is_return else None
			angular_ship_coefficient = self.plane.get_angular_coefficient_pursuit_evade(
				target=self.plane.ship, min_max_distance=ship_distance)
			planes_coefficients = []
			for p in self.planes:  # Yes we follow not only previous but all planes
				if p.status == 'flight':
					if not is_return:  # Trying to pursuit plane when scouting
						c = self.plane.get_angular_coefficient_pursuit_evade(
							target=p, min_max_distance=self.min_plane_dist)
					else:  # Trying to avoid plane when returning
						c = self.plane.get_angular_coefficient_pursuit_evade(
							target=p, is_evade=True, min_max_distance=0)
				else:
					c = 0
				planes_coefficients.append(c)
			angular_coefficient = angular_ship_coefficient + sum(planes_coefficients)
			# Limit angular speed between in interval [-ANGULAR_MAX_SPEED, ANGULAR_MAX_SPEED]
			if angular_coefficient > 1.0:
				angular_coefficient = 1.0
			elif angular_coefficient < -1.0:
				angular_coefficient = -1.0
			angular_speed = self.plane.ANGULAR_MAX_SPEED * angular_coefficient
			linear_speed_sqr = self.plane.body.linearVelocity.lengthSquared
			angular_impulse = mass * linear_speed_sqr * angular_speed
			self.plane.body.ApplyAngularImpulse(angular_impulse, True)
			# FIXME Workaround with ANGULAR_MAX_SPEED. Maybe limit ANGULAR_MAX_IMPULSE in the ship like?
			av = self.plane.body.angularVelocity
			if av > self.plane.ANGULAR_MAX_SPEED:
				self.plane.body.angularVelocity = self.plane.ANGULAR_MAX_SPEED
			elif av < -self.plane.ANGULAR_MAX_SPEED:
				self.plane.body.angularVelocity = -self.plane.ANGULAR_MAX_SPEED

	def __init__(
		self, world, vertices=None, density=0.1, position=(0, 0), ship=None,
		min_landing_dist=20
	):
		self.linear_speed_sqr = 0
		if vertices is None: vertices = Plane.vertices
		self.body = world.CreateDynamicBody(position=position)
		# FIXME Do we need to implement collision logic between plane-ground or plane-plane?
		# Because they are small and flying high
		self.body.CreatePolygonFixture(
			vertices=vertices,
			density=density,
			filter=b2Filter(groupIndex=0, categoryBits=0x0002, maskBits=0x0000))
		self.body.angularDamping = 1.1
		self.body.linearDamping = 1.1
		self.strategy = Plane.Aboard(self)
		self.status = 'aboard'  # flight, crashing, repairing, preparation, ...?
		self.status_time = 0.0
		self.ship = ship
		self.min_landing_dist = min_landing_dist
		self.last_time = time.time()

	def takeoff(self):
		print('Takeoff')
		self.status = 'flight'
		self.status_time = 0.0
		self.last_time = time.time()
		# LOGIC
		ship_forward_direction = self.ship.body.GetWorldVector((0, 1))  # Normalized by default
		ship_position = self.ship.body.worldCenter
		start_position = ship_position + ship_forward_direction * self.min_landing_dist
		start_angle = math.atan2(-ship_forward_direction.x, ship_forward_direction.y)
		self.body.transform = (start_position, start_angle)
		# TODO beautiful takeoff
		mass = self.body.mass
		forward_direction = self.body.GetWorldVector((0, 1))  # Normalized by default
		impulse = mass * self.LINEAR_MIN_SPEED * forward_direction
		self.body.ApplyLinearImpulse(impulse, self.body.worldCenter, True)

	def landing(self):
		print('Landing')
		self.status = 'aboard'
		self.status_time = 0.0
		self.last_time = time.time()
		# TODO beautiful landing
		self.strategy = Plane.Aboard(self)

	def update_linear_speed(self):
		mass = self.body.mass
		forward_direction = self.body.GetWorldVector((0, 1))  # Normalized by default
		force = mass * self.LINEAR_ACCELERATION * forward_direction
		self.body.ApplyForceToCenter(force, True)
		# FIXME Workaround LINEAR_MAX_SPEED Should I Control speed only with Force/Impulse?
		plane_linear_velocity = self.body.linearVelocity
		plane_linear_velocity_length = plane_linear_velocity.length
		if plane_linear_velocity_length > 0:
			if plane_linear_velocity_length > self.LINEAR_MAX_SPEED:
				self.body.linearVelocity = plane_linear_velocity / plane_linear_velocity_length * self.LINEAR_MAX_SPEED
			elif plane_linear_velocity_length < self.LINEAR_MIN_SPEED:
				self.body.linearVelocity = plane_linear_velocity / plane_linear_velocity_length * self.LINEAR_MIN_SPEED
		# plane_linear_velocity = self.body.linearVelocity
		# if plane_linear_velocity.length < self.LINEAR_MIN_SPEED:
		# 	impulse = mass * self.LINEAR_MIN_SPEED * forward_direction
		# 	self.body.ApplyLinearImpulse(impulse, self.body.worldCenter, True)
		# if plane_linear_velocity.length < self.LINEAR_MAX_SPEED:
		# 	force = mass * self.LINEAR_ACCELERATION * forward_direction
		# 	self.body.ApplyForceToCenter(force, True)
		# else:  # Don't apply force if velocity > LINEAR_MAX_SPEED
		# 	pass

	def get_angular_coefficient_pursuit_evade(
		self, target, is_evade=False, min_max_distance=None):
		target_position = target.body.worldCenter
		position = self.body.worldCenter
		target_direction = target_position - position
		target_distance = target_direction.Normalize()
		min_time_to_target = target_distance / self.LINEAR_MAX_SPEED
		target_velocity = target.body.linearVelocity
		future_target_position = target_position + target_velocity * min_time_to_target
		future_target_direction = future_target_position - position
		future_target_distance = future_target_direction.Normalize()
		forward_direction = self.body.GetWorldVector((0, 1))  # Normalized by default
		# Yes, we could use atan2 like in takeoff,
		# but dot/cross is more clear and convenient here from my point of view
		future_dot = forward_direction.dot(future_target_direction)
		future_cross = forward_direction.cross(future_target_direction)
		# Scale dot to [0, 1], where 0 - same direction, 1 - opposite direction
		future_scaled_dot = 1 - 0.5 * (1 + future_dot)
		if future_cross > 0:  # Counterclockwise
			angular_coefficient = future_scaled_dot
		elif future_cross < 0:  # Clockwise
			angular_coefficient = -future_scaled_dot
		else:
			angular_coefficient = 0
		if is_evade:
			angular_coefficient = -angular_coefficient
		if min_max_distance is not None:
			if is_evade:
				if target_distance > min_max_distance:
					angular_coefficient = 0
			else:  # pursuit
				if target_distance < min_max_distance:
					angular_coefficient = -angular_coefficient
		return angular_coefficient

	def update(self, keys):
		cur_time = time.time()
		delta_time = cur_time - self.last_time
		self.status_time += delta_time
		self.last_time = cur_time
		self.strategy.update()


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

	def __init__(self, world, vertices=None, density=0.1, position=(0, 0), planes=None):

		self.linear_speed_sqr = 0

		if vertices is None: vertices = Ship.vertices

		self.body = world.CreateDynamicBody(position=position)
		self.body.CreatePolygonFixture(vertices=vertices, density=density)
		self.body.angularDamping = 1.1
		self.body.linearDamping = 1.1

		self.planes = [] if planes is None else planes
		self.plane_key_down = False
		self.plane_key_up = False

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
			new_plane_strategy = None
			for i, p in enumerate(self.planes):
				if isinstance(self.planes[i].strategy, Plane.Aboard):
					planes = [y for x, y in enumerate(self.planes) if x != i]
					new_strategy = Plane.Scout(
						plane=p, planes=planes, min_ship_dist=50, min_plane_dist=30,
						max_return_time=15  # With a margin due to MAX_FLIGHT_TIME = 20
					)
					p.strategy = new_strategy
					new_plane_i = i
					break
			if new_plane_i >= 0:
				print(f'Assign strategy {type(new_plane_strategy).__name__} to plane {new_plane_i + 1} ')
			else:
				print(f'No planes available at the moment!')


class ShipGame(Framework):
	name = "Ship Game"
	description = "Keys: accel = w, reverse = s, left = a, right = d"

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
		boundary.CreateEdgeChain([(-120, -120),
								  (-120, 120),
								  (120, 120),
								  (120, -120),
								  (-120, -120)]
								 )

		# A couple regions of differing traction
		self.car = Ship(self.world)
		gnd1 = self.world.CreateStaticBody()
		fixture = gnd1.CreatePolygonFixture(box=(9, 7, (-20, 15), math.radians(20)))

		gnd2 = self.world.CreateStaticBody()
		fixture = gnd2.CreatePolygonFixture(box=(4, 8, (5, 40), math.radians(-40)))

		# Kill me
		# plane_test = self.world.CreateStaticBody(position=(10,0))
		# fixture = plane_test.CreatePolygonFixture(vertices=PlaneShape)

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
		for i, p in enumerate(self.planes):
			p.update(self.pressed_keys)
			self.Print(f'Plane {i + 1} linear speed: {p.body.linearVelocity.length:.3f}')
			self.Print(f'Plane {i + 1} angular speed: {p.body.angularVelocity:.3f}')
			self.Print(f'Plane {i + 1} status {p.status} time: {p.status_time:.3f}')

		super(ShipGame, self).Step(settings)
		self.Print('Linear speed sqr: %s' % self.car.linear_speed_sqr)


if __name__ == "__main__":
	main(ShipGame)
