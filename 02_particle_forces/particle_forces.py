import random

import pygame


WIDTH, HEIGHT = 800, 600
BACKGROUND = (25, 25, 35)
GRAVITY = pygame.Vector2(0, 350)
ATTRACTION_MAG = 850


class Particle:
    def __init__(self, position, velocity):
        self.position = pygame.Vector2(position)
        self.velocity = pygame.Vector2(velocity)
        self.acceleration = pygame.Vector2()
        self.mass = random.uniform(1.0, 3.0)
        self.radius = round(7 + self.mass * 3)
        self.color = (70, 160, 255)

    def apply_force(self, force):
        # Newton's second law: acceleration = force / mass.
        self.acceleration += force / self.mass

    def calculate_acceleration(
        self, position, velocity, time, mouse_position=None, attracting=False
    ):
        """Return acceleration at a proposed state without changing the particle."""
        acceleration = GRAVITY.copy()

        # Mouse attraction changes with position, so RK4 must recalculate it.
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

        # This estimate matters if calculate_acceleration later uses velocity.
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
        # Preserve acceleration added through apply_force for this frame.
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
        restitution = 0.75

        if self.position.x - self.radius < 0:
            self.position.x = self.radius
            self.velocity.x *= -restitution
        elif self.position.x + self.radius > WIDTH:
            self.position.x = WIDTH - self.radius
            self.velocity.x *= -restitution

        if self.position.y - self.radius < 0:
            self.position.y = self.radius
            self.velocity.y *= -restitution
        elif self.position.y + self.radius > HEIGHT:
            self.position.y = HEIGHT - self.radius
            self.velocity.y *= -restitution

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, self.position, self.radius)


def create_particle(position):
    #velocity = (random.uniform(-150, 150), random.uniform(-150, 50))
    velocity = (0, 0)
    return Particle(position, velocity)


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Particle Forces - Semi Implicit Euler")
    clock = pygame.time.Clock()

    #particles = [create_particle((WIDTH / 2, HEIGHT / 3)) for _ in range(12)]
    particles = []
    integration_method = "semi_implicit_euler"
    running = True
    simulation_time = 0.0

    while running:
        dt = min(clock.tick(60) / 1000, 0.05)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    particles.clear()
                elif event.key == pygame.K_n:
                    particles.append(create_particle(pygame.Vector2(pygame.mouse.get_pos())))
                elif event.key == pygame.K_1:
                    integration_method = "explicit_euler"
                elif event.key == pygame.K_2:
                    integration_method = "semi_implicit_euler"
                elif event.key == pygame.K_3:
                    integration_method = "verlet"
                elif event.key == pygame.K_4:
                    integration_method = "rk4"

                pygame.display.set_caption(
                    f"Particle Forces - {integration_method.replace('_', ' ').title()}"
                )
                

        mouse_position = pygame.Vector2(pygame.mouse.get_pos())
        attracting = pygame.mouse.get_pressed()[0]

        screen.fill(BACKGROUND)
        for particle in particles:
            particle.update(
                dt,
                simulation_time,
                integration_method,
                mouse_position,
                attracting,
            )
            particle.bounce_off_walls()
            particle.draw(screen)

        pygame.display.flip()
        simulation_time += dt

    pygame.quit()


if __name__ == "__main__":
    main()
