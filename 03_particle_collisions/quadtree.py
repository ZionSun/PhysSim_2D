import pygame

class Quadtree:
    def __init__(self, boundary, capacity=4, depth=0, max_depth=8):
        self.boundary = pygame.Rect(boundary)
        self.capacity = capacity
        self.depth = depth
        self.max_depth = max_depth
        self.particles = []
        self.children = []

    def subdivide(self):
        x, y = self.boundary.topleft
        left_width = self.boundary.width // 2
        right_width = self.boundary.width - left_width
        top_height = self.boundary.height // 2
        bottom_height = self.boundary.height - top_height

        child_boundaries = (
            (x, y, left_width, top_height),
            (x + left_width, y, right_width, top_height),
            (x, y + top_height, left_width, bottom_height),
            (x + left_width, y + top_height, right_width, bottom_height),
        )
        self.children = [
            Quadtree(bounds, self.capacity, self.depth + 1, self.max_depth)
            for bounds in child_boundaries
        ]

        # Move particles into children only when their entire circle fits.
        particles_to_reinsert = self.particles
        self.particles = []
        for particle in particles_to_reinsert:
            child = self._containing_child(particle)
            if child is None:
                self.particles.append(particle)
            else:
                child.insert(particle)

    def insert(self, particle):
        if not self._contains_circle(self.boundary, particle):
            return False

        if self.children:
            child = self._containing_child(particle)
            if child is not None:
                return child.insert(particle)

        self.particles.append(particle)

        if (
            len(self.particles) > self.capacity
            and self.depth < self.max_depth
            and not self.children
        ):
            self.subdivide()

        return True

    def query(self, area, found=None):
        area = pygame.Rect(area)
        if found is None:
            found = []

        if not self._rectangles_intersect(self.boundary, area):
            return found

        for particle in self.particles:
            if self._circle_intersects_rectangle(particle, area):
                found.append(particle)

        for child in self.children:
            child.query(area, found)

        return found

    def clear(self):
        self.particles.clear()
        self.children.clear()

    def draw(self, screen, color=(80, 80, 100)):
        pygame.draw.rect(screen, color, self.boundary, 1)
        for child in self.children:
            child.draw(screen, color)

    def _containing_child(self, particle):
        for child in self.children:
            if self._contains_circle(child.boundary, particle):
                return child
        return None

    @staticmethod
    def _contains_circle(rectangle, particle):
        return (
            particle.position.x - particle.radius >= rectangle.left
            and particle.position.x + particle.radius <= rectangle.right
            and particle.position.y - particle.radius >= rectangle.top
            and particle.position.y + particle.radius <= rectangle.bottom
        )

    @staticmethod
    def _rectangles_intersect(first, second):
        return not (
            first.right < second.left
            or first.left > second.right
            or first.bottom < second.top
            or first.top > second.bottom
        )

    @staticmethod
    def _circle_intersects_rectangle(particle, rectangle):
        closest_x = max(rectangle.left, min(particle.position.x, rectangle.right))
        closest_y = max(rectangle.top, min(particle.position.y, rectangle.bottom))
        difference_x = particle.position.x - closest_x
        difference_y = particle.position.y - closest_y
        return difference_x**2 + difference_y**2 <= particle.radius**2