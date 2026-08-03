import random
import pygame

from particle import Particle
from quadtree import Quadtree
from settings import WIDTH, HEIGHT, BACKGROUND

def create_particle(position):
    #velocity = (random.uniform(-150, 150), random.uniform(-150, 50))
    velocity = (0, 0)
    particle = Particle(position, velocity)
    return particle


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Particle Forces - RK4")
    clock = pygame.time.Clock()

    #particles = [create_particle((WIDTH / 2, HEIGHT / 3)) for _ in range(12)]
    particles = []
    qtree = Quadtree((0, 0, WIDTH, HEIGHT))
    integration_method = "rk4"
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

        # Rebuild the tree after movement so it contains current positions.
        qtree.clear()
        for particle in particles:
            qtree.insert(particle)

        processed_pairs = set()
        for particle in particles:
            area = pygame.Rect(
                particle.position.x - particle.radius * 2,
                particle.position.y - particle.radius * 2,
                particle.radius * 4,
                particle.radius * 4,
            )
            nearby = qtree.query(area)

            for other in nearby:
                if other is particle:
                    continue

                pair = tuple(sorted((id(particle), id(other))))
                if pair in processed_pairs:
                    continue

                processed_pairs.add(pair)
                particle.collide([other])

        qtree.draw(screen)
        for particle in particles:
            particle.draw(screen)

        pygame.display.flip()
        simulation_time += dt

    pygame.quit()


if __name__ == "__main__":
    main()
