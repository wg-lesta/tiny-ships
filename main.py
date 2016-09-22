#!/usr/bin/env python
# -*- coding: utf-8 -*-

from framework import *
import math
import time

class Entity(object):
	LINEAR_SPEED        = 50;
	ANGULAR_SPEED       = 0.1;
	ANGULAR_MAX_IMPULSE = 1.5;

	def __init__(self, world, vertices=None, density=0.1, position=(0, 0)):
		self.linear_speed_sqr = 0;
		
		self.body    = world.CreateDynamicBody(position=position);
		self.fixture = self.body.CreatePolygonFixture(vertices=vertices, density=density);
		
		self.body.angularDamping = 1.1;
		self.body.linearDamping  = 1.1;

		self.fixture.filterData.groupIndex = -1;

	def UpdateLinear(self, throttle):
		direction = self.body.GetWorldVector((0, 1));

		self.body.ApplyForceToCenter(self.LINEAR_SPEED * throttle * direction, True);
		self.linear_speed_sqr    = self.body.linearVelocity.lengthSquared;

	def UpdateAngular(self, turn):
		angular_impulse = self.ANGULAR_SPEED * self.linear_speed_sqr;

		if angular_impulse > self.ANGULAR_MAX_IMPULSE:
			angular_impulse = self.ANGULAR_MAX_IMPULSE;

		self.body.ApplyAngularImpulse(angular_impulse * turn, True);

class Ship(Entity):
	vertices = [( 1.5, 0.0),
				( 3.0, 5.0),
				( 2.8, 11.0),
				( 1.0,20.0),
				(-1.0,20.0),
				(-2.8, 11.0),
				(-3.0, 5.0),
				(-1.5, 0.0),
				];

	def __init__(self, world, vertices=None, density=0.1, position=(0, 0)):
		super(Ship, self).__init__(world, vertices if vertices is not None else Ship.vertices, density, position);

	def DoPulse(self, keys):
		
		throttle = 0;
		if 'up' in keys: throttle += 1;
		if 'down' in keys: throttle -= 1;
		self.UpdateLinear(throttle);
		
		turn = 0
		if 'left' in keys: turn += 1;
		if 'right' in keys: turn -= 1;
		self.UpdateAngular(turn);

class Plane(Entity):
	vertices = [( 2.0, -4.0),
				( 0.0, 4.0),
				( -2.4, -4.0),
				];

	LINEAR_SPEED           = 100;
	LINEAR_SPEED_MIN       = 0.5;
	LINEAR_SPEED_MAX       = 1.0;

	ANGULAR_SPEED          = 5.0;
	MAX_ANGLE              = 20.0;

	TARGET_DISTANCE        = 35.0;

	SHIP_ROUTE_RADIUS      = 75.0;
	CHECKPOINT_RADIUS      = 20.0;

	LANDING_DISTANCE       = 100;

	TIME_FLYING_LIMIT      = 20;

	STATE_READY            = 1;
	STATE_LANDING          = 2;
	STATE_LANDING_APPROACH = 3;
	STATE_FLYING           = 4;

	def __init__(self, manager, vertices=None, density=0.2, position=(0, 0)):
		super(Plane, self).__init__(manager.World, vertices if vertices is not None else Plane.vertices, density, position);

		self.Manager      = manager;
		self.Ship         = manager.Ship;
		self.State        = Plane.STATE_READY;
		self.StartTime    = 0;
		self.Target       = None;
		self.LeadingPlane = None;

		self.Route        = [];
		self.RouteIndex   = 0;

		self.LandingDownRadius   = 0.0;

		self.body.linearDamping  = 0.2;
		self.body.angularDamping = 0.0;
		self.body.friction       = 0.3;
		self.body.restitution    = 0.4;

		self.Throttle = 0.0;
		self.Angle    = 0.0;

	def Start(self):
		self.SetState(self.STATE_FLYING);

	def LandingDown(self):
		self.SetState(self.STATE_LANDING_APPROACH);

	def SetState(self, state):
		if (state == self.STATE_FLYING):
			self.StartTime = time.time();

			self.CreateDefaultRoute();

			self.Target = self.Route[self.RouteIndex];

		elif (state == self.STATE_READY):
			self.body.position = self.Ship.body.worldCenter;
			self.body.angle    = self.Ship.body.angle;

		elif (state == self.STATE_LANDING_APPROACH):
			self.LandingDownRadius = self.SHIP_ROUTE_RADIUS;

		self.State = state;

	def GetAngle(self, position):
		return math.atan2(position.y - self.body.position.y, position.x - self.body.position.x) - math.radians(90);

	def GetOffsetPosition(self, position, rotation, distance):
		return b2Vec2(
			position.x + (math.cos(rotation + math.radians(90)) * distance), 
			position.y + (math.sin(rotation + math.radians(90)) * distance), 
		);

	def GetNextPoint(self):
		self.RouteIndex = (self.RouteIndex + 1) % len(self.Route);

		return self.Route[self.RouteIndex];

	def CreateDefaultRoute(self):
		self.Route      = [];
		self.RouteIndex = 0;

		position = self.Ship.body.position;
		rotation = self.Ship.body.angle;

		i = 0;

		for angle in range(0, 360):
			if (angle % 45 == 0):
				self.Route.append(self.GetOffsetPosition(position, rotation + math.radians(angle), self.SHIP_ROUTE_RADIUS));

	def UpdateRotation(self):
		if (self.Target is None):
			return;

		rotation   = (360.0 - math.degrees(self.Angle)) % 360.0;
		p_rotation = (360.0 - math.degrees(self.body.angle)) % 360.0; 

		diff_rotation = (360.0 - (rotation - p_rotation)) % 360.0;

		diff_rotation = (diff_rotation + 180.0) % 360.0;

		if (diff_rotation < 0.0):
			diff_rotation += 360.0;

		diff_rotation -= 180.0;

		if (diff_rotation > self.MAX_ANGLE or diff_rotation < -self.MAX_ANGLE):
			self.Throttle = -(abs(diff_rotation) / 180.0);

		self.body.angularVelocity = math.radians(min(max(diff_rotation, -self.MAX_ANGLE), self.MAX_ANGLE)) * self.ANGULAR_SPEED;

	def UpdateLinear(self):
		if self.State == Plane.STATE_READY:
			return;

		vel = self.body.linearVelocity.length / self.LINEAR_SPEED;

		if (vel < self.LINEAR_SPEED_MIN):
			throttle = (self.LINEAR_SPEED_MIN - vel) / self.LINEAR_SPEED_MAX;

			if (self.Throttle < throttle): self.Throttle = throttle;

			if (self.Throttle < 0.0): self.Throttle = 0.0;

		self.Throttle = min(max(self.Throttle, -1.0), 1.0);

		if (vel > self.LINEAR_SPEED_MAX):
			self.Throttle = -0.1;

		self.Manager.Game.Print("Plane V: %.3f, T: %.3f" % (vel, self.Throttle));

		self.body.ApplyForce(self.body.GetWorldVector(b2Vec2(0, self.Throttle * self.LINEAR_SPEED_MAX)) * self.LINEAR_SPEED, self.body.worldCenter, True);

	def Update(self, timestamp):
		if self.State == Plane.STATE_READY:
			self.body.position       = self.Ship.body.worldCenter;
			self.body.angle          = self.Ship.body.angle;
			self.body.linearVelocity = (0.0, 0.0);
		
		elif self.State == Plane.STATE_LANDING:
			self.Target = self.Ship.body.worldCenter;

			vel = self.body.linearVelocity.length / self.LINEAR_SPEED;

			self.Throttle = 1.0 - (vel / self.LINEAR_SPEED_MIN);

			distance = self.Target - self.body.position;

			if (distance.length < 10.0):
				self.SetState(self.STATE_READY);

		elif self.State == Plane.STATE_LANDING_APPROACH:
			vel = self.body.linearVelocity.length / self.LINEAR_SPEED;

			self.Throttle = 1.0 - (vel / self.LINEAR_SPEED_MIN);

			position_front = self.Ship.body.GetWorldVector(b2Vec2(0, self.LandingDownRadius));
			position_back  = self.Ship.body.GetWorldVector(b2Vec2(0, -self.LandingDownRadius));

			front = position_front - self.body.position;
			back  = position_back - self.body.position;

			self.Target = position_front if front.length < back.length else position_back;

			distance = self.Target - self.body.position;

			if (distance.length < self.CHECKPOINT_RADIUS):
				self.SetState(Plane.STATE_LANDING);

		elif self.State == Plane.STATE_FLYING:
			self.Throttle = 0.5;

			vel = self.body.linearVelocity.length / self.LINEAR_SPEED;

			if (self.RouteIndex == 0):
				self.Throttle = 1.0;

			if (self.LeadingPlane and self.RouteIndex > 0):
				pos = self.LeadingPlane.body.worldCenter;

				distance = pos - self.body.position;

				self.Throttle = ((distance.length - self.TARGET_DISTANCE) / self.TARGET_DISTANCE);

				if (self.Throttle < 0.0 and vel < self.LINEAR_SPEED_MIN):
					self.Throttle = 0.0;

				if (distance.length < self.TARGET_DISTANCE * 0.5):
					self.Angle = self.GetAngle(self.body.GetWorldVector((0, -1)));

			distance = self.Target - self.body.position;

			if (distance.length < self.CHECKPOINT_RADIUS):
				self.Target = self.GetNextPoint();

			if (timestamp - self.StartTime > self.TIME_FLYING_LIMIT):
				self.LandingDown();

	def DoPulse(self, keys, settings):
		timestamp = time.time();

		self.Throttle = 0.0;
		self.Angle    = 0.0;

		if (self.Target):
			self.Angle = self.GetAngle(self.Target);

		self.ANGULAR_SPEED    = settings.AngularVelocity; 
		self.LINEAR_SPEED_MIN = settings.LinearVelocityMin / 50.0;
		self.LINEAR_SPEED_MAX = settings.LinearVelocityMax / 50.0;

		self.RemoveSidewaysVelocity();

		self.Update(timestamp);

		self.UpdateRotation();
		self.UpdateLinear();

	def GetDirectionVector(self):
		local_velocity = self.body.GetLocalVector(self.body.GetLinearVelocityFromLocalPoint(b2Vec2(0, 0)));

		vector = b2Vec2(0, 1) if local_velocity.y > 0 else b2Vec2(0, -1);

		x = vector.x * math.cos(self.body.angle) - vector.y * math.sin(self.body.angle);
		y = vector.x * math.sin(self.body.angle) + vector.y * math.cos(self.body.angle);

		return b2Vec2(x, y);

	def RemoveSidewaysVelocity(self):
		velocity = self.body.linearVelocity;
		sideways = self.GetDirectionVector();

		dotprod = velocity.x * sideways.x + velocity.y * sideways.y;

		self.body.linearVelocity = b2Vec2(sideways.x * dotprod, sideways.y * dotprod);

class PlaneManager():

	MAX_PLANES = 5;

	def __init__(self, game):
		self.Planes    = [];
		self.Game      = game;
		self.Ship      = game.car;
		self.World     = game.world;
		self.LastPlane = None;

		self.KeyPressedTime = 0;

		for i in range(PlaneManager.MAX_PLANES):
			self.Planes.append(Plane(self));

	def DoPulse(self, keys, settings):
		t = time.time();

		for i in range(PlaneManager.MAX_PLANES):
			plane = self.Planes[i];

			if (plane.LeadingPlane and plane.LeadingPlane.State == Plane.STATE_READY):
				plane.LeadingPlane = None;

			if (t - self.KeyPressedTime > 1 and 'h' in keys and plane.State == Plane.STATE_READY):
				self.KeyPressedTime = t;

				plane.Start();
				plane.LeadingPlane = self.LastPlane;

				self.LastPlane = plane;

			if (plane.State == Plane.STATE_READY):
				plane.LeadingPlane = None;

				if (self.LastPlane == plane):
					self.LastPlane = None;

			plane.DoPulse(keys, settings);

class ShipGame (Framework):
	name="Ship Game"
	description="Keys: accel = w, reverse = s, left = a, right = d"

	def __init__(self):
		super(ShipGame, self).__init__()
		
		# Top-down -- no gravity in the screen plane
		self.world.gravity = (0, 0)
		self.key_map = {Keys.K_w: 'up', Keys.K_s: 'down', Keys.K_a: 'left', Keys.K_d: 'right', Keys.K_h: 'h' }
		
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
		fixture = gnd2.CreatePolygonFixture(box=(4, 8, (15, 40), math.radians(-40)))

		# Kill me
		plane_test = self.world.CreateStaticBody(position=(10, 0))
		fixture = plane_test.CreatePolygonFixture(vertices=Plane.vertices)

		self.PlaneManager = PlaneManager(self);
	
	
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
		self.car.DoPulse(self.pressed_keys)
		self.PlaneManager.DoPulse(self.pressed_keys, settings);
		super(ShipGame, self).Step(settings)
		self.Print('Linear speed sqr: %s' % self.car.linear_speed_sqr)

if __name__=="__main__":
	 main(ShipGame)

