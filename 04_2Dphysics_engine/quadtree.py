import pygame

class Quadtree:
    def __init__(self, boundary, capacity=4, depth=0, max_depth=8):
        self.boundary = pygame.Rect(boundary)
        self.capacity = capacity
        self.depth = depth
        self.max_depth = max_depth
        self.bodies = []
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

        # Move bodies into children only when their entire AABB fits.
        bodies_to_reinsert = self.bodies
        self.bodies = []
        for body in bodies_to_reinsert:
            child = self._containing_child(body)
            if child is None:
                self.bodies.append(body)
            else:
                child.insert(body)

    def insert(self, body):
        if not self._contains_body(self.boundary, body):
            return False

        if self.children:
            child = self._containing_child(body)
            if child is not None:
                return child.insert(body)

        self.bodies.append(body)

        if (
            len(self.bodies) > self.capacity
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

        for body in self.bodies:
            if self._body_intersects_rectangle(body, area):
                found.append(body)

        for child in self.children:
            child.query(area, found)

        return found

    def clear(self):
        self.bodies.clear()
        self.children.clear()

    def draw(self, screen, color=(80, 80, 100)):
        pygame.draw.rect(screen, color, self.boundary, 1)
        for child in self.children:
            child.draw(screen, color)

    def _containing_child(self, body):
        for child in self.children:
            if self._contains_body(child.boundary, body):
                return child
        return None

    @staticmethod
    def _contains_body(rectangle, body):
        bounds = body.aabb
        return (
            bounds.left >= rectangle.left
            and bounds.right <= rectangle.right
            and bounds.top >= rectangle.top
            and bounds.bottom <= rectangle.bottom
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
    def _body_intersects_rectangle(body, rectangle):
        return Quadtree._rectangles_intersect(body.aabb, rectangle)
