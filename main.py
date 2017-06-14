#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import *
import math
import time

class Mathematics:
	@staticmethod
	def angle(v1, v2):
		angleValue = v1.dot(v2) / (v1.length * v2.length);

		if (math.fabs(angleValue) > 1.0):
			angleValue -= 1.0 if angleValue > 0.0 else -1.0

		return math.acos(angleValue)

class GameObject(object):
	def __init__(self, world, vertices, density, position):
		self.body = world.CreateDynamicBody(position=position)
		fixture = self.body.CreatePolygonFixture(vertices=vertices, density=density)
		fixture.filterData.groupIndex = -1

	def update_linear(self, throttle):
		direction = self.body.GetWorldVector((0, 1))
		self.body.ApplyForceToCenter(self.LINEAR_SPEED * throttle * direction, True)
		self.linear_speed_sqr = self.body.linearVelocity.lengthSquared

	def update_angular(self, turn): pass


class Plane(GameObject):
	LINEAR_SPEED = 100.0

	MAX_YAW_ANGLE = math.radians(25.0)

	FLY_TIME_LIMIT = 20		#sec

	PLANE_DISTANCE = 15
	TARGET_DISTANCE = 50
	LANDING_DISTANCE = 10

	STATE_FLYING = 1
	STATE_ONBOARD = 2
	STATE_RETURNING = 3

	vertices = [( 2.0, -4.0),
				( 0.0, 4.0),
				( -2.4, -4.0),
				]

	def __init__(self, world, parent, leading = None, vertices = None, density = 0.1, position= (0, 0)):
		super(Plane, self).__init__(world, vertices if vertices is not None else Plane.vertices, density, position)

		self.state = Plane.STATE_ONBOARD
		self.leading = leading
		self.ship = parent

		self.launchTime = 0
		self.throttle = 0.0

		self.linear_speed_sqr = 0
		self.body.angularDamping = 0.0
		self.body.linearDamping = 0.2

		self.linearSpeedMax = 0.0
		self.linearSpeedMin = 0.0
		self.angularSpeed = 0.0

	def launch(self, launchTime, settings, leading):
		self.state = Plane.STATE_FLYING
		self.launchTime = launchTime
		self.leading = leading

		self.linearSpeedMax = settings.maxLinearVelocity / Plane.LINEAR_SPEED
		self.linearSpeedMin = settings.minLinearVelocity / Plane.LINEAR_SPEED
		self.angularSpeed = float(settings.angularVelocity)

	def update_angular(self, angle):
		if math.fabs(angle) > Plane.MAX_YAW_ANGLE:
			self.throttle = -(math.fabs(angle) / math.pi);

		self.body.angularVelocity = min(max(angle, -Plane.MAX_YAW_ANGLE), Plane.MAX_YAW_ANGLE) * self.angularSpeed;

	def update_linear(self, throttle = 0):
		velocity = self.body.linearVelocity.length / self.LINEAR_SPEED;

		if velocity < self.linearSpeedMin:
			throttle = (self.linearSpeedMin - velocity + b2Random(0.0, self.linearSpeedMax - self.linearSpeedMin )) / self.linearSpeedMax

			if (self.throttle < throttle):
				self.throttle = throttle

		elif self.linearSpeedMin <= velocity <= self.linearSpeedMax:
			self.throttle = 0.1
		else:
			self.throttle = 0.0

		super(Plane, self).update_linear(self.throttle)

	def update(self, timeStamp):
		if self.state == Plane.STATE_ONBOARD:
			self.body.position = self.ship.body.worldCenter
			self.body.angle = self.ship.body.angle
			self.body.linearVelocity = (0.0, 0.0)
			self.body.angularVelocity = 0.0
		else:
			self.angular_compensation()
			shipDistance = self.ship.body.worldCenter - self.body.position

			angle = 0.0
			if self.state == Plane.STATE_RETURNING:
				angle = Mathematics.angle(shipDistance, self.body.GetWorldVector((0, 1)))

				if (shipDistance.length < Plane.LANDING_DISTANCE):
					self.state = Plane.STATE_ONBOARD
					self.leading = None

			elif self.state == Plane.STATE_FLYING:
				if shipDistance.length > Plane.TARGET_DISTANCE:
					angle = Mathematics.angle(shipDistance, self.body.GetWorldVector((0, 1))) - math.radians(90)

				if self.leading:
					planeDistance = self.leading.body.worldCenter - self.body.position

					if planeDistance.length < Plane.PLANE_DISTANCE:
						angle += Mathematics.angle(planeDistance, self.body.GetWorldVector((0, 1))) - math.radians(90)

				if timeStamp - self.launchTime > Plane.FLY_TIME_LIMIT:
					self.state = Plane.STATE_RETURNING

			if shipDistance.length > 0:
				self.update_angular(angle)

			self.update_linear()

	def angular_compensation(self):
		velocity = self.body.linearVelocity
		direction = self.body.GetWorldVector((0, -1) if velocity.y >= 0 else (0,1))
		self.body.linearVelocity = direction * velocity.dot(direction)

#Aerodrome
class Ship(GameObject):
	LINEAR_SPEED = 50.0
	ANGULAR_SPEED = 0.1
	ANGULAR_MAX_IMPULSE = 1.5

	MAX_AVAILABILITY_PLANES = 5
	COOLDOWN_LAUNCH_TIME = 2	#sec

	vertices = [( 1.5, 0.0),
				( 3.0, 5.0),
				( 2.8, 11.0),
				( 1.0,20.0),
				(-1.0,20.0),
				(-2.8, 11.0),
				(-3.0, 5.0),
				(-1.5, 0.0),
				]
	
	def __init__(self, world, vertices = None, density = 0.1, position= (0, 0)):
		super(Ship, self).__init__(world, vertices if vertices is not None else Ship.vertices, density, position)

		self.linear_speed_sqr = 0
		self.body.angularDamping = 1.1
		self.body.linearDamping = 1.1

		self.aerodrome = [Plane(world = world, parent = self, position = self.body.worldCenter) for i in range(Ship.MAX_AVAILABILITY_PLANES)]
		self.lastLaunchPlane = None
		self.lastLaunchTime = 0

	def update_angular(self, turn):
		angular_impulse = self.ANGULAR_SPEED * self.linear_speed_sqr
		if angular_impulse > self.ANGULAR_MAX_IMPULSE: angular_impulse = self.ANGULAR_MAX_IMPULSE
		self.body.ApplyAngularImpulse( angular_impulse * turn, True )

	def update(self, keys, settings):
		throttle = 0
		if 'up' in keys: throttle += 1
		if 'down' in keys: throttle -= 1
		self.update_linear(throttle)
		
		turn = 0
		if 'left' in keys: turn += 1
		if 'right' in keys: turn -= 1
		self.update_angular(turn)

		timeStamp = time.time()
		for plane in self.aerodrome:
			if plane.state == Plane.STATE_ONBOARD:
				if 'launch' in keys and timeStamp - self.lastLaunchTime > Ship.COOLDOWN_LAUNCH_TIME:
					plane.launch(timeStamp, settings, self.lastLaunchPlane)
					self.lastLaunchTime = timeStamp
					self.lastLaunchPlane = plane

				elif self.lastLaunchPlane == plane:
					self.lastLaunchPlane = None;

			plane.update(timeStamp)

class ShipGame (Framework):
	name="Ship Game"
	description="Keys: accel = w, reverse = s, left = a, right = d, launch plane = h"

	def __init__(self):
		super(ShipGame, self).__init__()

		# Top-down -- no gravity in the screen plane
		self.world.gravity = (0, 0)
		self.key_map = {Keys.K_w: 'up', Keys.K_s: 'down', Keys.K_a: 'left', Keys.K_d: 'right', Keys.K_h: 'launch'}

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
		self.ship = Ship(self.world)

		gnd1 = self.world.CreateStaticBody()
		fixture = gnd1.CreatePolygonFixture(box=(9, 7, (-20, 15), math.radians(20)))

		gnd2 = self.world.CreateStaticBody()
		fixture = gnd2.CreatePolygonFixture(box=(4, 8, (5, 40), math.radians(-40)))

		# Kill me
		plane_test = self.world.CreateStaticBody(position=(10,0))
		fixture = plane_test.CreatePolygonFixture(vertices=Plane.vertices)
	
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
		self.ship.update(self.pressed_keys, settings)
		super(ShipGame, self).Step(settings);

		if settings.drawStats:
			self.Print("plane vel: min = %d max = %d angl = %d" % (settings.minLinearVelocity, settings.maxLinearVelocity, settings.angularVelocity))

		for plane in self.ship.aerodrome:
			if (plane.state != Plane.STATE_ONBOARD):
				self.Print("Plane Vel: %.3f, Thr: %.3f" % (plane.body.linearVelocity.length, plane.throttle))

		self.Print('Linear speed sqr: %s' % self.ship.linear_speed_sqr)

if __name__=="__main__":
	 main(ShipGame)
