import random
import pygame

from rigidbody import RigidBody
from quadtree import Quadtree
from settings import WIDTH, HEIGHT, BACKGROUND, GRAVITY, ATTRACTION_MAG


def create_circle(position):
    mass = random.uniform(1.0, 3.0)
    radius = round(7 + mass * 3)
    return RigidBody.circle(position, mass, radius)


def create_rectangle(position):
    mass = random.uniform(1.0, 3.0)
    width = random.uniform(25, 50)
    height = random.uniform(15, 35)
    return RigidBody.rect(position, mass, width, height)


def create_walls():
    thickness = 100
    wall_settings = {"is_static": True, "restitution": 0.2, "friction": 0.7}
    return (
        RigidBody.rect(
            (-thickness / 2, HEIGHT / 2),
            0,
            thickness,
            HEIGHT + thickness * 2,
            **wall_settings,
        ),
        RigidBody.rect(
            (WIDTH + thickness / 2, HEIGHT / 2),
            0,
            thickness,
            HEIGHT + thickness * 2,
            **wall_settings,
        ),
        RigidBody.rect(
            (WIDTH / 2, -thickness / 2),
            0,
            WIDTH,
            thickness,
            **wall_settings,
        ),
        RigidBody.rect(
            (WIDTH / 2, HEIGHT + thickness / 2),
            0,
            WIDTH,
            thickness,
            **wall_settings,
        ),
    )


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Rigid Bodies - C: Circle, V: Rectangle")
    clock = pygame.time.Clock()

    bodies = []
    walls = create_walls()
    qtree = Quadtree((0, 0, WIDTH, HEIGHT))
    running = True

    while running:
        dt = min(clock.tick(60) / 1000, 0.05)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    bodies.clear()
                elif event.key == pygame.K_c:
                    bodies.append(create_circle(pygame.mouse.get_pos()))
                elif event.key == pygame.K_v:
                    bodies.append(create_rectangle(pygame.mouse.get_pos()))

        mouse_position = pygame.Vector2(pygame.mouse.get_pos())
        attracting = pygame.mouse.get_pressed()[0]

        screen.fill(BACKGROUND)
        for body in bodies:
            body.apply_force(GRAVITY * body.mass)

            direction = mouse_position - body.position
            if attracting and direction.length_squared() > 0:
                body.apply_force(direction.normalize() * ATTRACTION_MAG)

            body.update(dt)
            for wall in walls:
                body.collide(wall)

        # Rebuild the tree after movement so it contains current positions.
        qtree.clear()
        for body in bodies:
            qtree.insert(body)

        processed_pairs = set()
        for body in bodies:
            nearby = qtree.query(body.aabb)

            for other in nearby:
                if other is body:
                    continue

                pair = tuple(sorted((id(body), id(other))))
                if pair in processed_pairs:
                    continue

                processed_pairs.add(pair)
                body.collide(other)
        qtree.draw(screen)
        for body in bodies:
            body.draw(screen)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
