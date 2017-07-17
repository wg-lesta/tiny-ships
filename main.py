#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import *
import math

CATEGORY_WATER = 0x0001
CATEGORY_AIR = 0x0002

class Plane(object):
	STATE_READY         = 1
	STATE_MOVE_TO_ORBIT = 2
	STATE_KEEP_DIST     = 3
	STATE_RETURN        = 4
	
	_density_coef = 100.0
	
	#distance between ship and airplanes
	_pref_dist = 100.0
	_time_in_air = 0
	_vertices = [( 2.0,  -4.0),
				 ( 0.0,   4.0),
				 ( -2.0, -4.0),
				]
	_fixture = b2FixtureDef(
		shape=b2PolygonShape(vertices=_vertices),
		density=0.1 / _density_coef,# make mass of plane lesser
		filter=b2Filter(
			groupIndex=0,			# do not collide between this group
			maskBits=CATEGORY_AIR,	# do not collide between air and water
		)
	)	
	
	def __init__(self, world, ship):
		self.world = world
		self.shipBody = ship
		self.targetPlane = None
		
		self._state = Plane.STATE_READY
		self.cur_lin_speed = 0
		self.min_lin_speed = 0
		self.max_lin_speed = 0		
		self.max_ang_speed = 0
		
		self.body = world.CreateDynamicBody(
			position=ship.GetWorldVector((0, 1)) * 10,
			angle=ship.angle,
			fixtures=Plane._fixture,
		)
		self.joint = world.CreateWeldJoint(
			bodyA=self.body,
			bodyB=ship,
			anchor=self.body.worldCenter
		)
		
	def appear(self, target):		
		self._state = Plane.STATE_MOVE_TO_ORBIT
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
		
	def _rotateVect(self, vec, ang):
		return b2Vec2(
			(vec[0] * math.cos(ang) + vec[1] * -math.sin(ang)),
			(vec[0] * math.sin(ang) + vec[1] *  math.cos(ang))
		)
	def _scalarMult(self, vec1, vec2):
		return vec1[0] * vec2[0] + vec1[1] * vec2[1]
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

	def update_vel(self, rec_lin_speed, rec_ang_speed, hz):
		delta_hz = (hz / 60.0) 
		self.cur_lin_speed = self.body.linearVelocity.length
		dir_face = self.body.GetWorldVector((0, 1))
		dir_ship = self.shipBody.position - self.body.position
		
		if (self._state == self.STATE_KEEP_DIST and self.targetPlane != None):
			if ((self.targetPlane.body.position - self.body.position).length < self._pref_dist and rec_lin_speed * 0.9 > self.min_lin_speed):
				self.cur_lin_speed *= 0.95
	
		if (self.cur_lin_speed < rec_lin_speed):
			self.cur_lin_speed += 1 / delta_hz
		elif (self.cur_lin_speed > rec_lin_speed):
			self.cur_lin_speed -= 1 / delta_hz
		if (self.cur_lin_speed > self.max_lin_speed):
			self.cur_lin_speed -= 1 / delta_hz
		vec_move = dir_face * self.cur_lin_speed
		
		self.body.ApplyForceToCenter(vec_move - self.body.linearVelocity, True)
					
		turn = rec_ang_speed / 10.0 / Plane._density_coef
		need_to_reduce_radius = False
		if (self._state == self.STATE_KEEP_DIST and dir_ship.length > self._pref_dist * 1.05 and
			(self.body.angularVelocity < self.max_ang_speed / 60.0)):
			need_to_reduce_radius = True
		scalar = self._scalarMult(dir_ship / dir_ship.length, dir_face)
		
		if (self._state == self.STATE_RETURN):
			if (self.body.angularVelocity < turn):
				turn = -self.body.angularVelocity
			elif (self.body.angularVelocity > 0 and turn > 0):
				turn *= -1
		elif (need_to_reduce_radius):
			if (scalar > 0.1 and turn > 0):			
				turn *= -1
		else:
			if ((self.body.angularVelocity > rec_ang_speed / 60.0) and turn > 0):
				turn *= -1
			if (scalar > 0 and turn > 0):			
				turn *= -1
		
		self.body.ApplyTorque(turn, True)
		
		if (self._state == Plane.STATE_MOVE_TO_ORBIT and self.shipBody.position != self.body.position and scalar > 0):
			self._state = self.STATE_KEEP_DIST
		if (self._state == self.STATE_RETURN and dir_ship.length < 5):
			self.disappear()
			

	def update(self, settings):	
		self.min_lin_speed = settings.minPlaneLinearSpeed
		self.max_lin_speed = settings.maxPlaneLinearSpeed
		self.max_ang_speed = settings.maxPlaneAngularSpeed
		
		if (self._state == Plane.STATE_READY):
			return	
		self._time_in_air += 1
		
		pref_lin_speed = self.max_lin_speed
		pref_ang_speed = self.max_ang_speed
		linVel_on_maxAng = self._pref_dist * (self.max_ang_speed / 60.0)
		if (linVel_on_maxAng > self.max_lin_speed):
			pref_ang_speed = self.max_lin_speed / self._pref_dist * 60.0
		elif (linVel_on_maxAng < self.min_lin_speed):
			pref_lin_speed = self.min_lin_speed
		else:
			pref_lin_speed = linVel_on_maxAng
				
		if (self._state == Plane.STATE_MOVE_TO_ORBIT):
			self.update_vel(max(self.min_lin_speed, pref_lin_speed / 2.0), pref_ang_speed, settings.hz)			
		if (self._state == Plane.STATE_KEEP_DIST):
			self.update_vel(pref_lin_speed, pref_ang_speed, settings.hz)
		if (self._state == Plane.STATE_RETURN):
			self.update_vel(self.min_lin_speed, self.max_ang_speed, settings.hz)
	
class PlaneManager(object):
	
	MAX_PLANES = 5
	
	_time_to_return = 60	# in sec
	
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

			if (self.Hangar[i].getTimeInAir() > PlaneManager._time_to_return * settings.hz and self.Hangar[i].getState() != Plane.STATE_READY):
				self.Hangar[i].setStateToReturn()

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
		# used hz to draw smoothly, mb should use fps
		self.planes.update(self.pressed_keys, settings)
		super(ShipGame, self).Step(settings)
		
		self.Print('Linear speed sqr: %s' % self.car.linear_speed_sqr)
		self.Print('Min/Max linear speed of plane: %.1f/%.1f' % (settings.minPlaneLinearSpeed, settings.maxPlaneLinearSpeed))
		self.Print('Max angular speed of plane: %.1f' % settings.maxPlaneAngularSpeed)
		for i in range(PlaneManager.MAX_PLANES):
			self.Print('=--------------------------------------------------- ')
			self.Print('Time in Air for plane #%i is : %.1f sec.' % (i+1, self.planes.Hangar[i].getTimeInAir() / float(settings.hz)))
			lin_speed = 0;
			ang_speed = 0;
			delta_hz = settings.hz / 60.0
			if (self.planes.Hangar[i].getState() != Plane.STATE_READY):
				lin_speed = self.planes.Hangar[i].body.linearVelocity.length
				ang_speed = self.planes.Hangar[i].body.angularVelocity * settings.hz / delta_hz
			self.Print('   Linear/Angular speed : %+06.1f/%+05.1f' % (lin_speed, ang_speed))
			self.Print('   Distance to ship is %+06.1f' % (self.planes.Hangar[i].body.position - self.car.body.position).length)
			self.Print('   State is %d' % self.planes.Hangar[i].getState())
			target_dist = 0
			if (self.planes.Hangar[i].targetPlane != None):
				target_dist = (self.planes.Hangar[i].targetPlane.body.position - self.planes.Hangar[i].body.position).length
			self.Print('   Distance to next plane = %.1f' % target_dist)
			
			
if __name__=="__main__":
	 main(ShipGame)