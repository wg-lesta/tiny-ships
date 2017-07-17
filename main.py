#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import *
import math

CATEGORY_WATER = 0x0001
CATEGORY_AIR = 0x0002

class Plane(object):
	STATE_READY = 1
	STATE_IN_AIR = 2
	STATE_RETURN = 3
	
	#distance between ship and airplanes
	_pref_dist = 100	
	_time_in_air = 0
	_angle_to_next_point = math.pi / 9
	_rotMatrix = [(math.cos(_angle_to_next_point), -math.sin(_angle_to_next_point)),
				  (math.sin(_angle_to_next_point),  math.cos(_angle_to_next_point))
				 ]
	_vertices = [( 2.0,  -4.0),
				 ( 0.0,   4.0),
				 ( -2.0, -4.0),
				]
	fixture = b2FixtureDef(
		shape=b2PolygonShape(vertices=_vertices),
		density=0.1,
		filter=b2Filter(
			groupIndex=0,			# do not collide between this group
			maskBits=CATEGORY_AIR,	# do not collide between air and water
		)
	)	
	
	def __init__(self, world, ship):
		self.world = world
		self.shipBody = ship
		self.targetPlane = None
		self.targetPoint = None
		
		self._state = Plane.STATE_READY
		self.cur_lin_speed = 0
		self.cur_lin_speed_sqr = 0
		self.min_lin_speed = 0
		self.max_lin_speed = 0
		
		self.cur_ang_speed = 0
		self.max_ang_speed = 0
		self.max_ang_impulse = 1.5
		
		self.body = self.world.CreateDynamicBody(
			position=ship.GetWorldVector((0, 1)) * 10,
			angle=ship.angle,
			fixtures=Plane.fixture,
		)
		self.joint = world.CreateWeldJoint(
			bodyA=self.body,
			bodyB=ship,
			anchor=self.body.worldCenter
		)
		
	def appear(self, target):		
		self._state = Plane.STATE_IN_AIR
		self.world.DestroyJoint(self.joint)
		self.joint = None
		self.body.angularVelocity = 0	
		self.targetPlane = target
			
		self.shipSpeed = self.body.linearVelocity.length
		self.body.linearVelocity = self.body.GetWorldVector((0, 1))
		self.body.linearVelocity *= max(self.min_lin_speed, self.shipSpeed)		
		return self
		
	#moving airplane to hangar position and joint it. Set airplane ready to flight
	def _disappear(self):
		self.body.angularVelocity = 0
		self.body.angle=self.ship.angle		
		self.body.linearVelocity = (0, 0)
		self.body.position=ship.GetWorldVector((0, 1)) * 10,
		self.joint = self.world.CreateWeldJoint(
			bodyA=self.body,
			bodyB=self.ship,
			anchor=self.body.worldCenter
		)		
		self._state = Plane.STATE_READY
		self.time_in_air = 0
		
	def _rotateVect2(self, vec, mat):
		return ((vec[0] * mat[0][0] + vec[1] * mat[0][1]) ,(vec[0] * mat[1][0] + vec[1] * mat[1][1]))
	def _getNextTargetPoint(self):
		self.dist_till_ship = self.body.position - self.shipBody.position
		self.dist_till_ship /= self.dist_till_ship.length
		self.dist_till_ship *= self._pref_dist
		self.dist_till_ship = self._rotateVect2(self.dist_till_ship, self._rotMatrix)
		return self.shipBody.position + self.dist_till_ship
	def _leftTurn(self, vec1, vec2):
		lt = vec1[0] * vec2[1] - vec1[1] * vec2[0]
		if (lt > 0):
			return 1
		if (lt < 0):
			return -1
		return 0
		
	def getTimeInAir(self):
		return self._time_in_air
	def getState(self):
		return self._state
	def setStateToReturn(self):
		self._state = Plane.STATE_RETURN

	def update_linear(self, throttle, in_return = False):
		direction = self.body.GetWorldVector((0, 1))
		self.cur_lin_speed = self.body.linearVelocity.length
		self.body.ApplyForceToCenter(self.cur_lin_speed * throttle * direction, True)
		
		dir_vel = self.body.linearVelocity + (0, 0)
		self.cur_lin_speed = self.body.linearVelocity.length		
		dir_vel /= self.cur_lin_speed
		scalar = direction[0] * dir_vel[0] + direction[1] * dir_vel[1]	#if >0 then we go forward
		
		if (self.cur_lin_speed > self.max_lin_speed):
			if (scalar > 0):
				throttle = -1
			else:
				throttle = 2
			self.body.ApplyForceToCenter(self.cur_lin_speed * throttle * direction, True)
		elif (self.cur_lin_speed < self.min_lin_speed):
			if (scalar > 0):
				throttle = 1
			self.body.ApplyForceToCenter(self.cur_lin_speed * throttle * direction, True)
		
		self.cur_lin_speed_sqr = self.body.linearVelocity.lengthSquared
		
	def update_angular(self, turn, hz):		
		turn /= 20.0
		if ((self.body.angularVelocity * hz > self.max_ang_speed   and turn > 0) or 
			(self.body.angularVelocity * hz < - self.max_ang_speed and turn < 0)):
			turn *= -1
		self.body.ApplyAngularImpulse(turn, True)
	#	self.body.ApplyTorque(-turn, True)
		
		
	def update(self, settings):			
		self.min_lin_speed = settings.minPlaneLinearSpeed
		self.max_lin_speed = settings.maxPlaneLinearSpeed
		self.max_ang_speed = settings.maxPlaneAngularSpeed
		
		if (self._state == Plane.STATE_READY):
			return
	
		self._time_in_air += 1
			
		if (self._state == Plane.STATE_IN_AIR):
			throttle = 0
			if (self.targetPlane != None):
				if ((self.targetPlane.body.position - self.body.position).length < self._pref_dist):
					throttle -= 1
				else:
					throttle += 1
			else:
				throttle += 1
			self.update_linear(throttle)
		
			point_to_move = self._getNextTargetPoint()
			vec_to_move = point_to_move - self.body.position
			
			turn = self._leftTurn(self.body.GetWorldVector((0, 1)), vec_to_move)
			self.update_angular(turn, settings.hz)
	
class PlaneManager(object):
	
	MAX_PLANES = 5
	
	_time_to_return = 25	# in sec
	
	airplane_last = None
	
	def __init__(self, world, ship):
		self.Hangar = []
		self.Ship = ship
				
		self._waiting_for_takeoff = False
		self._time_passed = 0
		
		for i in range(self.MAX_PLANES):
			self.Hangar.append(Plane(
				world,
				ship.body
			))
	
	def update(self, keys, settings):	
		self._waiting_time = settings.hz
		
		#Cooldown for launching airplanes == 1 sec.
		if (self._waiting_for_takeoff):
			self._time_passed += 1
			if (self._time_passed >= self._waiting_time):
				self._time_passed = 0
				self._waiting_for_takeoff = False

		for i in range(self.MAX_PLANES):			
			if ('h' in keys and self.Hangar[i].getState() == Plane.STATE_READY and not self._waiting_for_takeoff and self.Ship.isGoingForward()):
				self.airplane_new = self.Hangar[i].appear(self.airplane_last)				
				self.airplane_last = self.airplane_new
				self._waiting_for_takeoff = True

	#		if (self.Hangar[i].getTimeInAir() > PlaneManager._time_to_return * settings.hz and self.Hangar[i].getState() == Plane.STATE_IN_AIR):
	#			self.Hangar[i].setStateToReturn()

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
		if (self.body.linearVelocity.length == 0):
			return True
		self.va = self.body.GetWorldVector((0, 1))
		self.vb = self.body.linearVelocity + (0, 0)
		self.vb /= self.body.linearVelocity.length
		if ((self.va.x * self.vb.x + self.va.y * self.vb.y) > 0.98):
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
			self.Print('Time in Air for plane #%i is : %.1f sec.' % (i+1, self.planes.Hangar[i].getTimeInAir() / float(settings.hz)))
			lin_speed = 0;
			ang_speed = 0;
			if (self.planes.Hangar[i].getState() != Plane.STATE_READY):
				lin_speed = self.planes.Hangar[i].body.linearVelocity.length
				ang_speed = self.planes.Hangar[i].body.angularVelocity * settings.hz
			self.Print('   Linear/Angular speed : %.1f/%.1f' % (lin_speed, ang_speed))
			
if __name__=="__main__":
	 main(ShipGame)