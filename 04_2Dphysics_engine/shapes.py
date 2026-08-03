import math

import pygame


class Circle:
    def __init__(self, radius, color=(70, 160, 255)):
        if radius <= 0:
            raise ValueError("Radius must be positive")
        self.radius = float(radius)
        self.color = color

    @property
    def bounding_radius(self):
        return self.radius

    def moment_of_inertia(self, mass):
        return 0.5 * mass * self.radius**2

    def world_aabb(self, position, angle):
        left = math.floor(position.x - self.radius)
        top = math.floor(position.y - self.radius)
        right = math.ceil(position.x + self.radius)
        bottom = math.ceil(position.y + self.radius)
        return pygame.Rect(
            left,
            top,
            right - left,
            bottom - top,
        )

    def draw(self, screen, position, angle):
        pygame.draw.circle(screen, self.color, position, self.radius)
        direction = pygame.Vector2(self.radius, 0).rotate_rad(angle)
        pygame.draw.line(
            screen, (225, 235, 255), position, position + direction, 2
        )


class Rectangle:
    def __init__(self, width, height, color=(70, 160, 255)):
        if width <= 0 or height <= 0:
            raise ValueError("Width and height must be positive")
        self.width = float(width)
        self.height = float(height)
        self.color = color

        half_width = self.width / 2
        half_height = self.height / 2
        self.local_vertices = (
            pygame.Vector2(-half_width, -half_height),
            pygame.Vector2(half_width, -half_height),
            pygame.Vector2(half_width, half_height),
            pygame.Vector2(-half_width, half_height),
        )

    @property
    def bounding_radius(self):
        return math.hypot(self.width / 2, self.height / 2)

    def moment_of_inertia(self, mass):
        return mass * (self.width**2 + self.height**2) / 12

    def world_vertices(self, position, angle):
        return [position + vertex.rotate_rad(angle) for vertex in self.local_vertices]

    def world_aabb(self, position, angle):
        vertices = self.world_vertices(position, angle)
        minimum_x = math.floor(min(vertex.x for vertex in vertices))
        maximum_x = math.ceil(max(vertex.x for vertex in vertices))
        minimum_y = math.floor(min(vertex.y for vertex in vertices))
        maximum_y = math.ceil(max(vertex.y for vertex in vertices))
        return pygame.Rect(
            minimum_x,
            minimum_y,
            maximum_x - minimum_x,
            maximum_y - minimum_y,
        )

    def draw(self, screen, position, angle):
        pygame.draw.polygon(
            screen, self.color, self.world_vertices(position, angle)
        )
