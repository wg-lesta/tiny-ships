import math
import time

PLANE_SPAWN_KEY = 'spawnPlane'
PLANES_CONUT = 5

class PLANE_FLYING_STEPS:
	FLYING, LANDING, LANDED = range(3)

class Plane(object):
	vertices = [
		( 2.0, -4.0),
		( 0.0, 4.0),
		( -2.4, -4.0),
	]

	TURNING_TIME = 5
	FLYING_TIME = TURNING_TIME + 20
	
	# Distance between ship
	DISTANCE_SHIP = 70

	# distanse / DISTANCE_SHIP
	# At a value greater than this, plane can adjust the distance relative to nearest plane
	DISTANCE_SHIP_ACCURACY = 0.9

	# Distance between plane
	DISTANCE_PLANE = 20

	def __init__(self, world, position, angle, settings):
		super(Plane, self).__init__()

		self.step = None
		self.stepTime = None

		self.world = world
		self.body = world.CreateDynamicBody(position=position, angle=angle, mass=0.0)
		fixture = self.body.CreatePolygonFixture(vertices=self.vertices, density=0.1)
		fixture.filterData.groupIndex = -1 # Collision avoidance with other planes
		
		self.body.angularDamping = 25.0
		self.body.linearDamping = 1.5

		self.updateSettings(settings)

	@property
	def isLanded(self):
		return self.step == PLANE_FLYING_STEPS.LANDED

	@property
	def isFlying(self):
		return self.step == PLANE_FLYING_STEPS.FLYING

	def updateSettings(self, settings):
		self.linearSpeed = settings.planeLinearSpeed
		self.angularImpulse = settings.planeAngularSpeed

	def nextStep(self):
		self.stepTime = time.time()
		if self.step is None:
			self.step = PLANE_FLYING_STEPS.FLYING
		else:
			self.step += 1

	def destroy(self):
		self.world.DestroyBody(self.body)
		self.body = None
		self.world = None

	def move(self, shipBody, plane):
		flyDirection = (0, 1)

		velocity = shipBody.worldCenter - self.body.worldCenter
		if self.step != PLANE_FLYING_STEPS.LANDING:
			if plane and velocity.length / self.DISTANCE_SHIP > self.DISTANCE_SHIP_ACCURACY:
				velocity = plane.body.worldCenter - self.body.worldCenter
				needDistance = self.DISTANCE_PLANE
			else:
				needDistance = self.DISTANCE_SHIP

			if velocity.length < needDistance:
				velocity = -velocity
			velocity.Normalize()

			worldDir = self.body.GetWorldVector(flyDirection)
			worldDir.Normalize()
			velocity += worldDir

		velocity.Normalize()

		cross = self.body.GetWorldVector(flyDirection).cross(velocity)
		angular = self.angularImpulse * (-1 if cross < 0 else 1)

		self.body.ApplyForceToCenter(velocity * self.linearSpeed, True)
		self.body.ApplyAngularImpulse(angular, True)

	def update(self, shipBody, plane, settings):
		self.updateSettings(settings)

		if self.step == None:
			self.nextStep()

		self.move(shipBody, plane)

		# Update step
		if self.step == PLANE_FLYING_STEPS.FLYING:
			if self.FLYING_TIME < time.time() - self.stepTime:
				self.nextStep()
		elif self.step == PLANE_FLYING_STEPS.LANDING:
			# Plane can be landed only if it touches the ship 
			for contact in shipBody.contacts:
				if contact.other == self.body:
					self.nextStep()
					break

class PlaneCtrl(object):
	def __init__(self, world):
		super(PlaneCtrl, self).__init__()
		self.world = world
		self.planes = []

		# Fix multiple handling key event
		self._isPlaneSpawned = False

	def update(self, shipBody, keys, settings):
		# Updating planes
		prevPlane = None
		for plane in list(self.planes):
			plane.update(shipBody, prevPlane, settings)
			if plane.isLanded:
				self.planes.remove(plane)
				plane.destroy()
			elif plane.isFlying:
				prevPlane = plane

		if PLANE_SPAWN_KEY in keys:
			if not self._isPlaneSpawned and len(self.planes) < PLANES_CONUT:
				position = shipBody.worldCenter + shipBody.GetWorldVector((8, 0))
				angle = shipBody.angle - math.pi/2
				self.planes.append(Plane(self.world, position, angle, settings))
			self._isPlaneSpawned = True
		else:
			self._isPlaneSpawned = False
