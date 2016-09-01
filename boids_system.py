from framework import *


class BoidsSystem:
    def __init__(self):
        self._boids = {}
        self._obstacles = []
        self._num_active_boids = 0

        self._separation_factor = 1.5
        self._cohesion_factor = 1.1
        self._align_factor = 1.0
        self._avoid_factor = 10

    @property
    def num_active_boids(self):
        return self._num_active_boids

    @property
    def separation_factor(self):
        return self._separation_factor

    @separation_factor.setter
    def separation_factor(self, value):
        self._separation_factor = value

    @property
    def cohesion_factor(self):
        return self._cohesion_factor

    @cohesion_factor.setter
    def cohesion_factor(self, value):
        self._cohesion_factor = value

    @property
    def align_factor(self):
        return self._align_factor

    @align_factor.setter
    def align_factor(self, value):
        self._align_factor = value

    @property
    def avoid_factor(self):
        return self._avoid_factor

    @avoid_factor.setter
    def avoid_factor(self, value):
        self._avoid_factor = value

    def add_boid(self, boid):
        if boid:
            self._boids[boid.name] = boid
            return True
        return False

    def add_obstacle(self, obstacle):
        if obstacle:
            self._obstacles.append(obstacle)

    def get_boid(self, name):
        return self._boids.get(name, None)

    def flock(self, _in_boid):
        separation = self.separate(_in_boid)
        align = self.align(_in_boid)
        cohesion = self.cohesion(_in_boid)
        avd = self.avoid(_in_boid, 100)

        avd *= self._avoid_factor
        separation *= self._separation_factor
        align *= self._align_factor
        cohesion *= self._cohesion_factor

        return separation + align + cohesion + avd

    def seek(self, _in_boid, target):
        desired = target - _in_boid.position
        if desired.length == 0:
            return b2Vec2(0, 0)

        desired.Normalize()
        desired *= _in_boid.LINEAR_SPEED

        steer = desired - _in_boid.linear_velocity

        if steer.length > _in_boid.MAX_IMPULSE:
            steer.Normalize()
            steer *= _in_boid.MAX_IMPULSE
        return steer

    def separate(self, _in_boid, separation=2500):
        steer = b2Vec2(0, 0)
        count = 0
        for boid in self._boids.itervalues():
            distance = b2DistanceSquared(_in_boid.position, boid.position)
            if (distance > 0) and (distance < separation):
                diff = _in_boid.position - boid.position
                diff.Normalize()
                diff *= 1.0 / distance
                steer += diff
                count += 1

        if count > 0:
            steer *= 1.0 / count

        if steer.length > 0:
            steer.Normalize()
            steer *= _in_boid.LINEAR_SPEED
            steer -= _in_boid.linear_velocity
            if steer.length > _in_boid.MAX_IMPULSE:
                steer.Normalize()
                steer *= _in_boid.MAX_IMPULSE
        return steer

    def align(self, _in_boid, neighbordist=10000):
        steer = b2Vec2(0, 0)
        count = 0

        for boid in self._boids.itervalues():
            distance = b2DistanceSquared(_in_boid.position, boid.position)
            if (distance > 0) and (distance < neighbordist):
                steer += boid.linear_velocity
                count += 1

        if count > 0:
            steer *= 1.0 / count

        if steer.length > 0:
            steer.Normalize()
            steer *= _in_boid.LINEAR_SPEED
            steer -= _in_boid.linear_velocity
            if steer.length > _in_boid.MAX_IMPULSE:
                steer.Normalize()
                steer *= _in_boid.MAX_IMPULSE
        return steer

    def cohesion(self, _in_boid, neighbordist=100000):
        sum = b2Vec2(0, 0)
        count = 0
        for boid in self._boids.itervalues():
            distance = b2DistanceSquared(_in_boid.position, boid.position)
            if (distance > 0) and (distance < neighbordist):
                sum += boid.position
                count += 1

        if count > 0:
            sum *= 1.0 / count
            return self.seek(_in_boid, sum)
        else:
            return b2Vec2(0, 0)

    def avoid(self, _in_boid, radius):
        avd = b2Vec2(0, 0)
        for ob in self._obstacles:
            distance = b2DistanceSquared(_in_boid.position, ob.centroid())
            if distance < 3000:
                closest = ob.closest_point(_in_boid.position)
                if b2DistanceSquared(_in_boid.position, closest) <= radius:
                    force = self.seek(_in_boid, closest)
                    force *= -1.0
                    force *= _in_boid.MAX_IMPULSE
                    avd += force
        return avd


'''
    Obstacles
'''


class ObstacleLine:
    def __init__(self, loc1, loc2):
        self._loc1 = loc1.copy()
        self._loc2 = loc2.copy()
        loc1 += loc2
        self._centroid = loc1.copy()
        self._centroid /= 2

    def centroid(self):
        return self._centroid

    def closest_point(self, pos):
        diffx = self._loc2.x - self._loc1.x
        diffy = self._loc2.y - self._loc1.y
        u = ((pos.x - self._loc1.x) * diffx + (pos.y - self._loc1.y) * diffy) / (diffx * diffx + diffy * diffy)
        u = max(min(u, 1), 0)
        return b2Vec2(self._loc1.x + u * diffx, self._loc1.y + u * diffy)


class ObstaclePoint:
    def __init__(self, loc):
        self._location = b2Vec2(loc)

    def centroid(self):
        return self._location

    def closest_point(self, pos):
        return self._location


class ObstacleCircle:
    def __init__(self, loc, radius):
        self._location = b2Vec2(loc)
        self._radius = radius

    def centroid(self):
        return self._location

    def closest_point(self, pos):
        radial = pos - self._location
        radial.Normalize()
        radial *= self._radius
        radial += self._location
        return radial
