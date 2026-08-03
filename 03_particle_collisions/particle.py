import pygame
import random

from settings import WIDTH, HEIGHT, GRAVITY, ATTRACTION_MAG, RESTITUTION


class Particle:
    def __init__(self, position, velocity):
        self.position = pygame.Vector2(position)
        self.velocity = pygame.Vector2(velocity)
        self.acceleration = pygame.Vector2()
        self.mass = random.uniform(1.0, 3.0)
        self.radius = round(7 + self.mass * 3)
        self.color = (70, 160, 255)

    def apply_force(self, force):
        self.acceleration += force / self.mass

    def calculate_acceleration(
        self, position, velocity, time, mouse_position=None, attracting=False
    ):
        acceleration = GRAVITY.copy()

        if attracting and mouse_position is not None:
            direction = mouse_position - position
            if direction.length_squared() > 0:
                attraction_force = direction.normalize() * ATTRACTION_MAG
                acceleration += attraction_force / self.mass

        return acceleration

    def explicit_euler(self, dt):
        self.position += self.velocity * dt
        self.velocity += self.acceleration * dt
        self.acceleration.update(0, 0)

    def semi_implicit_euler(self, dt):
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt
        self.acceleration.update(0, 0)

    def verlet(self, dt, time, mouse_position=None, attracting=False):
        applied_acceleration = self.acceleration.copy()
        old_acceleration = applied_acceleration + self.calculate_acceleration(
            self.position, self.velocity, time, mouse_position, attracting
        )

        new_position = (
            self.position
            + self.velocity * dt
            + 0.5 * old_acceleration * dt**2
        )

        estimated_velocity = self.velocity + old_acceleration * dt
        new_acceleration = applied_acceleration + self.calculate_acceleration(
            new_position,
            estimated_velocity,
            time + dt,
            mouse_position,
            attracting,
        )

        self.position = new_position
        self.velocity += 0.5 * (old_acceleration + new_acceleration) * dt
        self.acceleration.update(0, 0)

    def rk4(self, dt, time, mouse_position=None, attracting=False):
        applied_acceleration = self.acceleration.copy()

        k1_position = self.velocity
        k1_velocity = applied_acceleration + self.calculate_acceleration(
            self.position, self.velocity, time, mouse_position, attracting
        )

        k2_position = self.velocity + k1_velocity * dt / 2
        k2_velocity = applied_acceleration + self.calculate_acceleration(
            self.position + k1_position * dt / 2,
            self.velocity + k1_velocity * dt / 2,
            time + dt / 2,
            mouse_position,
            attracting,
        )

        k3_position = self.velocity + k2_velocity * dt / 2
        k3_velocity = applied_acceleration + self.calculate_acceleration(
            self.position + k2_position * dt / 2,
            self.velocity + k2_velocity * dt / 2,
            time + dt / 2,
            mouse_position,
            attracting,
        )

        k4_position = self.velocity + k3_velocity * dt
        k4_velocity = applied_acceleration + self.calculate_acceleration(
            self.position + k3_position * dt,
            self.velocity + k3_velocity * dt,
            time + dt,
            mouse_position,
            attracting,
        )

        self.position += (
            k1_position + 2 * k2_position + 2 * k3_position + k4_position
        ) * dt / 6
        self.velocity += (
            k1_velocity + 2 * k2_velocity + 2 * k3_velocity + k4_velocity
        ) * dt / 6
        self.acceleration.update(0, 0)

    def update(
        self,
        dt,
        time,
        integration_method="semi_implicit_euler",
        mouse_position=None,
        attracting=False,
    ):
        if integration_method == "rk4":
            self.rk4(dt, time, mouse_position, attracting)
            return

        if integration_method == "verlet":
            self.verlet(dt, time, mouse_position, attracting)
            return

        self.acceleration += self.calculate_acceleration(
            self.position, self.velocity, time, mouse_position, attracting
        )
        integrators = {
            "explicit_euler": self.explicit_euler,
            "semi_implicit_euler": self.semi_implicit_euler,
        }
        integrators[integration_method](dt)

    def bounce_off_walls(self):
        if self.position.x - self.radius < 0:
            penetration = self.radius - self.position.x
            self.position.x = self.radius + penetration * RESTITUTION
            if self.velocity.x < 0:
                self.velocity.x *= -RESTITUTION
        elif self.position.x + self.radius > WIDTH:
            penetration = self.position.x + self.radius - WIDTH
            self.position.x = WIDTH - self.radius - penetration * RESTITUTION
            if self.velocity.x > 0:
                self.velocity.x *= -RESTITUTION

        if self.position.y - self.radius < 0:
            penetration = self.radius - self.position.y
            self.position.y = self.radius + penetration * RESTITUTION
            if self.velocity.y < 0:
                self.velocity.y *= -RESTITUTION
        elif self.position.y + self.radius > HEIGHT:
            penetration = self.position.y + self.radius - HEIGHT
            self.position.y = HEIGHT - self.radius - penetration * RESTITUTION
            if self.velocity.y > 0:
                self.velocity.y *= -RESTITUTION

    def collide(self, particles):
        has_collided = False

        for other in particles:
            if other is self:
                continue

            difference = other.position - self.position
            combined_radius = self.radius + other.radius

            if difference.length_squared() > combined_radius**2:
                continue

            has_collided = True
            distance = difference.length()

            if distance == 0:
                normal = pygame.Vector2(1, 0)
            else:
                normal = difference / distance

            inverse_mass_self = 1 / self.mass
            inverse_mass_other = 1 / other.mass
            inverse_mass_sum = inverse_mass_self + inverse_mass_other

            overlap = combined_radius - distance
            self.position -= normal * overlap * inverse_mass_self / inverse_mass_sum
            other.position += normal * overlap * inverse_mass_other / inverse_mass_sum

            relative_velocity = other.velocity - self.velocity
            velocity_along_normal = relative_velocity.dot(normal)

            if velocity_along_normal >= 0:
                continue

            impulse_magnitude = (
                -(1 + RESTITUTION) * velocity_along_normal / inverse_mass_sum
            )
            impulse = impulse_magnitude * normal

            self.velocity -= impulse * inverse_mass_self
            other.velocity += impulse * inverse_mass_other

        return has_collided

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, self.position, self.radius)