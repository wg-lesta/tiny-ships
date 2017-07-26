#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

# const general variables
time_to_return   = 20
dist_ship_plane  = 100.0
dist_plane_plane = 100.0

# some functions that are general 
def scalarMult(vec1, vec2):
	return vec1[0] * vec2[0] + vec1[1] * vec2[1]
def leftTurn(vec1, vec2):
	lt = vec1[0] * vec2[1] - vec1[1] * vec2[0]
	if (lt > 0):
		return 1
	if (lt < 0):
		return -1
	return 0

# sort of enum for plane states
class PlaneState():
	READY         = 1
	MOVE_TO_ORBIT = 2
	KEEP_DIST     = 3
	RETURN        = 4

class Plane(object):
	# mass coeficient between ship and plane
	_density_coef     = 100.0
	# time in air in frames (1/60 of second)
	_frames_in_air = 0
	
	_vertices = [( 2.0,  -4.0),
				 ( 0.0,   4.0),
				 ( -2.0, -4.0),
				]	
	
	def __init__(self, world, ship):
		self.world = world
		self.ship = ship
		self.targetPlane = None
		
		self._state = PlaneState.READY
		self.delta_hz = 1
		
		self.cur_lin_speed = 0
		self.min_lin_speed = 0
		self.max_lin_speed = 0		
		self.max_ang_speed = 0
		
		self.pref_lin_speed = 0
		self.pref_ang_speed = 0
		self.pref_dist_to_ship = 0
		
		self._ret_rotating = True
		
		self.body = world.CreateDynamicBody(
			position=self.ship.body.position + ship.body.GetWorldVector((0, 1)) * 10,
			angle=ship.body.angle,
		)
		fixture = self.body.CreatePolygonFixture(
			vertices=Plane._vertices,
			density=0.1 / Plane._density_coef,  # make mass of plane lesser
		)
		fixture.filterData.groupIndex = 0       # do not collide between this group
		fixture.filterData.maskBits = 0x0002    # do not collide with water objects
		
		self.joint = world.CreateWeldJoint(
			bodyA=self.body,
			bodyB=ship.body,
			anchor=self.body.worldCenter
		)
		
	def appear(self, target):		
		self._state = PlaneState.MOVE_TO_ORBIT
		self.world.DestroyJoint(self.joint)
		self.joint = None
		self.body.angularVelocity = 0	
		self.targetPlane = target
		
		self._ret_rotating = True
		
		self.shipSpeed = self.body.linearVelocity.length
		self.body.linearVelocity = self.body.GetWorldVector((0, 1))
		self.body.linearVelocity *= max(self.min_lin_speed, self.shipSpeed)
		return self
		
	#moving airplane to hangar position and joint it. Set airplane ready to flight
	def _disappear(self):
		self.body.angularVelocity = 0
		self.body.angle=self.ship.body.angle		
		self.body.linearVelocity = (0, 0)
		self.body.position = self.ship.body.position + self.ship.body.GetWorldVector((0, 1)) * 10 
		self.joint = self.world.CreateWeldJoint(
			bodyA=self.body,
			bodyB=self.ship.body,
			anchor=self.body.worldCenter
		)		
		self._state = PlaneState.READY
		self._frames_in_air = 0
	
	# every update will recalculate recomended linear and angular speed with circle radius
	def _setPrefStats(self):		
		self.pref_lin_speed = self.max_lin_speed
		self.pref_ang_speed = self.max_ang_speed
		self.pref_dist_to_ship = dist_ship_plane
		
		linSpeed_on_maxAng = dist_ship_plane * math.radians(self.max_ang_speed)
		if (linSpeed_on_maxAng > self.max_lin_speed):
			self.pref_ang_speed = math.degrees(self.max_lin_speed / dist_ship_plane)
		elif (linSpeed_on_maxAng < self.min_lin_speed):
			self.pref_lin_speed = self.min_lin_speed
			self.pref_dist_to_ship = self.min_lin_speed / math.radians(self.max_ang_speed)
		else:
			self.pref_lin_speed = linSpeed_on_maxAng
	
	def getTimeInAir(self):
		# time in air in frames (1/60 of second)
		return self._frames_in_air / 60.0		
	def inHangar(self):
		if (self._state == PlaneState.READY): return True
		return False
	def inAir(self):
		return not self.inHangar()
	def inReturn(self):
		if (self._state == PlaneState.RETURN): return True
		return False
	def setStateToReturn(self):
		self._state = PlaneState.RETURN

	def update_vel(self, linSpeed, angSpeed):
		self.cur_lin_speed = self.body.linearVelocity.length
		dir_face = self.body.GetWorldVector((0, 1))
		pos_ship = self.ship.body.position + self.ship.body.GetWorldVector((0, 1)) * 9
		dir_ship = pos_ship - self.body.position
		
		if (self._state == PlaneState.KEEP_DIST and (self.targetPlane != None and self.targetPlane != self) and not (self.targetPlane.inReturn() or self.targetPlane.inHangar())):
			if ((self.targetPlane.body.position - self.body.position).length < dist_plane_plane and linSpeed * 0.9 > self.min_lin_speed):
				self.cur_lin_speed *= 0.95
	
		if (self.cur_lin_speed < linSpeed):
			self.cur_lin_speed += 1 / self.delta_hz
		elif (self.cur_lin_speed > linSpeed):
			self.cur_lin_speed -= 1 / self.delta_hz
		if (self.cur_lin_speed > self.max_lin_speed):
			self.cur_lin_speed -= 1 / self.delta_hz
		vec_move = dir_face * self.cur_lin_speed
		
		self.body.ApplyForceToCenter(vec_move - self.body.linearVelocity, True)
		
		if (self._state != PlaneState.RETURN):
			self.update_angle_vel(angSpeed, dir_face, dir_ship)
		else:
			self.update_angle_vel_ret(linSpeed, angSpeed, dir_face, dir_ship)
		
	def update_angle_vel(self, angSpeed, dir_face, dir_ship):
		turn = angSpeed / 10.0 / Plane._density_coef
		need_to_reduce_radius = False
		if (self._state == PlaneState.KEEP_DIST and dir_ship.length > dist_ship_plane * 1.05 and (self.body.angularVelocity < math.radians(self.max_ang_speed))):
			need_to_reduce_radius = True
			
		scalar = scalarMult(dir_ship / dir_ship.length, dir_face)		
		if (need_to_reduce_radius):
			if (scalar > 0.1 and turn > 0):			
				turn *= -1
		else:
			if ((self.body.angularVelocity > math.radians(angSpeed)) and turn > 0):
				turn *= -1
			if (scalar > 0 and turn > 0):			
				turn *= -1		
		self.body.ApplyTorque(turn, True)
		
		if (self._state == PlaneState.MOVE_TO_ORBIT and scalar > 0):
			self._state = PlaneState.KEEP_DIST
			
	#it is so hard to realize beauty solve in so little time (-_-)
	def update_angle_vel_ret(self, linSpeed, angSpeed, dir_face, dir_ship):		
		if (self._ret_rotating):
			if (leftTurn(dir_face, dir_ship) > 0):
				turn = angSpeed / 10.0 / Plane._density_coef
				self.body.ApplyTorque(turn, True)
			else:			
				self._ret_rotating = False
		else:
			self.body.angularVelocity = 0
			lt = leftTurn(dir_face, dir_ship)
			if (lt != 0):
				cos_angle = scalarMult(dir_ship / dir_ship.length, self.body.linearVelocity / self.body.linearVelocity.length)
				if (cos_angle < 1 and cos_angle > -1):
					angle = math.acos(cos_angle) / math.pi
				else:
					angle = 0
				if (leftTurn(dir_face, dir_ship) > 0):
					self.body.angle += angle
				else:
					self.body.angle -= angle
			
		if (dir_ship.length < 1):
			self._disappear()

	def update(self, settings):	
		if (self._state == PlaneState.READY):
			return
		if (self.getTimeInAir() > time_to_return):
			self.setStateToReturn()
		
		self.min_lin_speed = settings.minPlaneLinearSpeed
		self.max_lin_speed = settings.maxPlaneLinearSpeed
		self.max_ang_speed = settings.maxPlaneAngularSpeed
		
		# delta between curent hz and standart hz (60.0) to make similar moves for all fps
		self.delta_hz = (settings.hz / 60.0)
		self._frames_in_air += 1 / self.delta_hz
		
		self._setPrefStats()
				
		if (self._state == PlaneState.MOVE_TO_ORBIT):
			self.update_vel(max(self.min_lin_speed, self.pref_lin_speed / 2.0), self.pref_ang_speed)			
		if (self._state == PlaneState.KEEP_DIST):
			self.update_vel(self.pref_lin_speed, self.pref_ang_speed)
		if (self._state == PlaneState.RETURN):
			self.update_vel(min(self.min_lin_speed * 2, dist_ship_plane * 0.9 / 2 * math.pi / 4), self.max_ang_speed)			