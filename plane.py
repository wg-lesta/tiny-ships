#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

# const general variables
time_to_return   = 20
dist_ship_plane  = 100.0
dist_plane_plane = 100.0

# scalar multiplication of 2d vectors. Expected normalized vectors
def scalarMult(vec1, vec2):
	return vec1[0] * vec2[0] + vec1[1] * vec2[1]

# sort of enum for plane states
class PlaneState():
	READY         = 1
	FLYING        = 2
	RETURN        = 3

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
		
		self.minLinearSpeed  = 0
		self.maxLinearSpeed  = 0		
		self.maxAngularSpeed = 0
		
		self.prefLinearSpeed  = 0
		self.prefAngularSpeed = 0
		self.prefRadius       = 0
		self.midPoint         = ship.body.position
		
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
		self._state = PlaneState.FLYING
		self.world.DestroyJoint(self.joint)
		self.joint = None
		self.body.angularVelocity = 0
		if (target == self):
			target = None
		self.targetPlane = target
		
		self.shipSpeed = self.body.linearVelocity.length
		self.body.linearVelocity = self.body.GetWorldVector((0, 1))
		self.body.linearVelocity *= max(self.minLinearSpeed, self.shipSpeed)
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
	def _setPrefStats(self, point):		
		self.prefLinearSpeed  = self.maxLinearSpeed
		self.prefAngularSpeed = self.maxAngularSpeed
		self.prefRadius       = dist_ship_plane
		self.midPoint         = point
		
		linSpeed_on_maxAng = dist_ship_plane * math.radians(self.maxAngularSpeed)
		if (linSpeed_on_maxAng > self.maxLinearSpeed):
			self.prefAngularSpeed = math.degrees(self.maxLinearSpeed / dist_ship_plane)
		elif (linSpeed_on_maxAng < self.minLinearSpeed):
			self.prefLinearSpeed = self.minLinearSpeed
			self.prefRadius = self.minLinearSpeed / math.radians(self.maxAngularSpeed)
		else:
			self.prefLinearSpeed = linSpeed_on_maxAng			
	
	def getTimeInAir(self):
		# time in air in frames (1/60 of second)
		return self._frames_in_air / 60.0		
	def inHangar(self):
		if (self._state == PlaneState.READY): return True
		return False
	def inScout(self):
		if (self._state == PlaneState.FLYING): return True
		return False
	def inReturn(self):
		if (self._state == PlaneState.RETURN): return True
		return False
	
	def _updateLinear(self):
		cur_lin_speed = self.body.linearVelocity.length		
		dir_face = self.body.GetWorldVector((0, 1))

		deceleration = False
		# decelerate before landing
		if (self.inReturn()):
			pos_ship = self.ship.body.position + self.ship.body.GetWorldVector((0, 1)) * 10
			dir_ship = pos_ship - self.body.position
			if (dir_ship.length < dist_ship_plane / 5.0):
				deceleration = True
		# decelerate to increase distance between planes
		if (self.inScout() and self.targetPlane != None and self.targetPlane.inScout()):
			if ((self.targetPlane.body.position - self.body.position).length < dist_plane_plane):
				deceleration = True
				
		if (deceleration):
			if (cur_lin_speed - 1 / self.delta_hz > self.minLinearSpeed):
				cur_lin_speed -= 1 / self.delta_hz
			else:
				cur_lin_speed = self.minLinearSpeed
		else:
			if (cur_lin_speed > self.prefLinearSpeed):
				if (cur_lin_speed - 1 / self.delta_hz < self.prefLinearSpeed):
					cur_lin_speed = self.prefLinearSpeed
				else:
					cur_lin_speed -= 1 / self.delta_hz
			elif (cur_lin_speed < self.prefLinearSpeed):
				if (cur_lin_speed + 1 / self.delta_hz > self.prefLinearSpeed):
					cur_lin_speed = self.prefLinearSpeed
				else:
					cur_lin_speed += 1 / self.delta_hz
		vec_move = dir_face * cur_lin_speed
		
		self.body.ApplyForceToCenter(vec_move - self.body.linearVelocity, True)
		
	def _updateAngular(self):
		dir_face = self.body.GetWorldVector((0, 1))
		dir_midPoint = self.midPoint - self.body.position
		
		# angle deceleration from current linear speed
		if (self.body.linearVelocity.length > 0):
			self.prefAngularSpeed /= self.prefLinearSpeed / self.body.linearVelocity.length
			
		# ApplyTorque(1) adds 14.13547 degree to plane.angularVelocity
		inDegree = 1.0 / 14.13547 
		
		if (dir_midPoint.length > dist_ship_plane):			
			# scalar = cos of current angle between 'dir_face' and 'dir_midPoint'
			scalar = scalarMult(dir_midPoint / dir_midPoint.length, dir_face)
			# sinAngle = sin of required angle between 'dir_face' and 'dir_midPoint'
			sinAngle = dist_ship_plane / dir_midPoint.length
			
			if (scalar > 0 and sinAngle*sinAngle + scalar*scalar > 1):
				# we have turned more than required, time to turn back
				turn = (-self.prefAngularSpeed - math.degrees(self.body.angularVelocity))
			else:
				# we are out of 90 degree sector to direction
				turn = (self.prefAngularSpeed - math.degrees(self.body.angularVelocity))
		else:
			# we are IN radius circle, so can fly like on orbit - sometime we will fly away
			turn = (self.prefAngularSpeed - math.degrees(self.body.angularVelocity))
			
		if (math.fabs(turn) < 1):
			# when acceleretion is very small we will turn it a bit quicker to it's position
			turn *= inDegree
		else:
			# coef (1/5) is a manual delay to make full angular turn smoothly
			turn = turn * inDegree * (1.0 / 5)
			
		self.body.ApplyTorque(turn * self.delta_hz * (100.0 / self._density_coef), True)

	def update(self, settings):	
		if (self._state == PlaneState.READY):
			return
		if (self.getTimeInAir() > time_to_return):
			self._state = PlaneState.RETURN
		
		self.minLinearSpeed = settings.minPlaneLinearSpeed
		self.maxLinearSpeed = settings.maxPlaneLinearSpeed
		self.maxAngularSpeed = settings.maxPlaneAngularSpeed
		
		# delta between curent hz and standart hz (60.0) to make similar moves for all fps
		self.delta_hz = (settings.hz / 60.0)
		self._frames_in_air += 1 / self.delta_hz
		
		midPoint = self.ship.body.position + self.ship.body.GetWorldVector((0, 1)) * 9	
		
		if (self.inReturn()):
			if ((midPoint - self.body.position).length < 2):
				self._disappear()
			midPoint += self.ship.body.GetWorldVector((-1, 0)) * dist_ship_plane
		self._setPrefStats(midPoint)
		
		self._updateLinear()
		self._updateAngular()