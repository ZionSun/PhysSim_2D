import math

import pygame

from collision import detect_collision, resolve_collision
from shapes import Circle, Rectangle


class RigidBody:
    """A 2D rigid body with linear and angular motion."""

    def __init__(
        self,
        position,
        mass,
        shape,
        angle=0.0,
        restitution=0.8,
        friction=0.3,
        is_static=False,
    ):
        if not is_static and mass <= 0:
            raise ValueError("A dynamic rigid body must have a positive mass")
        moment_of_inertia = shape.moment_of_inertia(mass)
        if not is_static and moment_of_inertia <= 0:
            raise ValueError(
                "A dynamic rigid body must have a positive moment of inertia"
            )

        self.position = pygame.Vector2(position)
        self.velocity = pygame.Vector2()
        self.force = pygame.Vector2()

        self.angle = float(angle)
        self.angular_velocity = 0.0
        self.torque = 0.0

        self.mass = float(mass)
        self.shape = shape
        self.moment_of_inertia = float(moment_of_inertia)
        self.is_static = is_static

        self.inverse_mass = 0.0 if is_static else 1.0 / self.mass
        self.inverse_inertia = 0.0 if is_static else 1.0 / self.moment_of_inertia

        self.restitution = float(restitution)
        self.friction = float(friction)

    @classmethod
    def circle(cls, position, mass, radius, **kwargs):
        """Create a rigid body with the inertia of a solid circle."""
        return cls(position, mass, Circle(radius), **kwargs)

    @classmethod
    def rect(cls, position, mass, width, height, **kwargs):
        """Create a rigid body with the inertia of a solid rectangle."""
        return cls(position, mass, Rectangle(width, height), **kwargs)

    @property
    def radius(self):
        """Bounding radius used by the current quadtree broad phase."""
        return self.shape.bounding_radius

    @property
    def aabb(self):
        """World-space axis-aligned bounds of the body's shape."""
        return self.shape.world_aabb(self.position, self.angle)

    def apply_force(self, force, point=None):
        """Apply a force, optionally at a point in world coordinates."""
        if self.is_static:
            return

        force = pygame.Vector2(force)
        self.force += force

        if point is not None:
            offset = pygame.Vector2(point) - self.position
            self.torque += offset.cross(force)

    def apply_torque(self, torque):
        if not self.is_static:
            self.torque += torque

    def update(self, dt):
        """Advance the body using semi-implicit Euler integration."""
        if dt < 0:
            raise ValueError("dt cannot be negative")

        if self.is_static:
            self.clear_accumulators()
            return

        acceleration = self.force * self.inverse_mass
        self.velocity += acceleration * dt
        self.position += self.velocity * dt

        angular_acceleration = self.torque * self.inverse_inertia
        self.angular_velocity += angular_acceleration * dt
        self.angle += self.angular_velocity * dt
        self.angle %= 2 * math.pi

        self.clear_accumulators()

    def clear_accumulators(self):
        """Clear forces and torques after a simulation step."""
        self.force.update(0, 0)
        self.torque = 0.0

    def collide(self, other):
        collision = detect_collision(self, other)
        if collision is None:
            return False
        resolve_collision(collision)
        return True

    def resolve_world_bounds(self, width, height):
        """Keep a circular body inside an axis-aligned world boundary."""
        if self.is_static:
            return

        if self.position.x - self.radius < 0:
            self.position.x = self.radius
            if self.velocity.x < 0:
                self.velocity.x *= -self.restitution
        elif self.position.x + self.radius > width:
            self.position.x = width - self.radius
            if self.velocity.x > 0:
                self.velocity.x *= -self.restitution

        if self.position.y - self.radius < 0:
            self.position.y = self.radius
            if self.velocity.y < 0:
                self.velocity.y *= -self.restitution
        elif self.position.y + self.radius > height:
            self.position.y = height - self.radius
            if self.velocity.y > 0:
                self.velocity.y *= -self.restitution

    def draw(self, screen):
        self.shape.draw(screen, self.position, self.angle)
