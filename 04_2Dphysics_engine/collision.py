import math
from dataclasses import dataclass

import pygame

from shapes import Circle, Rectangle


@dataclass
class Collision:
    first: object
    second: object
    normal: pygame.Vector2
    penetration: float
    contact_point: pygame.Vector2


def detect_collision(first, second):
    if isinstance(first.shape, Circle) and isinstance(second.shape, Circle):
        return _circle_circle(first, second)
    if isinstance(first.shape, Circle) and isinstance(second.shape, Rectangle):
        return _circle_rectangle(first, second)
    if isinstance(first.shape, Rectangle) and isinstance(second.shape, Circle):
        collision = _circle_rectangle(second, first)
        if collision is None:
            return None
        return Collision(
            first,
            second,
            -collision.normal,
            collision.penetration,
            collision.contact_point,
        )
    if isinstance(first.shape, Rectangle) and isinstance(second.shape, Rectangle):
        return _rectangle_rectangle(first, second)
    raise TypeError("Unsupported collision shape")


def _circle_circle(first, second):
    difference = second.position - first.position
    combined_radius = first.shape.radius + second.shape.radius
    distance_squared = difference.length_squared()
    if distance_squared > combined_radius**2:
        return None

    distance = distance_squared**0.5
    normal = difference / distance if distance else pygame.Vector2(1, 0)
    contact = first.position + normal * first.shape.radius
    return Collision(first, second, normal, combined_radius - distance, contact)


def _circle_rectangle(circle_body, rectangle_body):
    circle = circle_body.shape
    rectangle = rectangle_body.shape
    local_center = (circle_body.position - rectangle_body.position).rotate_rad(
        -rectangle_body.angle
    )
    half_width = rectangle.width / 2
    half_height = rectangle.height / 2
    closest = pygame.Vector2(
        max(-half_width, min(local_center.x, half_width)),
        max(-half_height, min(local_center.y, half_height)),
    )
    difference = closest - local_center

    if difference.length_squared() == 0:
        distance_to_x = half_width - abs(local_center.x)
        distance_to_y = half_height - abs(local_center.y)
        if distance_to_x < distance_to_y:
            sign = 1 if local_center.x >= 0 else -1
            normal_local = pygame.Vector2(-sign, 0)
            closest.x = sign * half_width
            penetration = circle.radius + distance_to_x
        else:
            sign = 1 if local_center.y >= 0 else -1
            normal_local = pygame.Vector2(0, -sign)
            closest.y = sign * half_height
            penetration = circle.radius + distance_to_y
    else:
        distance = difference.length()
        if distance > circle.radius:
            return None
        normal_local = difference / distance
        penetration = circle.radius - distance

    normal = normal_local.rotate_rad(rectangle_body.angle)
    contact = rectangle_body.position + closest.rotate_rad(rectangle_body.angle)
    return Collision(circle_body, rectangle_body, normal, penetration, contact)


def _rectangle_rectangle(first, second):
    first_vertices = first.shape.world_vertices(first.position, first.angle)
    second_vertices = second.shape.world_vertices(second.position, second.angle)
    smallest_overlap = float("inf")
    smallest_axis = None

    for vertices in (first_vertices, second_vertices):
        for index in range(len(vertices)):
            edge = vertices[(index + 1) % len(vertices)] - vertices[index]
            axis = pygame.Vector2(-edge.y, edge.x).normalize()
            first_projection = [vertex.dot(axis) for vertex in first_vertices]
            second_projection = [vertex.dot(axis) for vertex in second_vertices]
            overlap = min(max(first_projection), max(second_projection)) - max(
                min(first_projection), min(second_projection)
            )
            if overlap < 0:
                return None
            if overlap < smallest_overlap:
                smallest_overlap = overlap
                smallest_axis = axis

    if (second.position - first.position).dot(smallest_axis) < 0:
        smallest_axis = -smallest_axis

    contact_candidates = [
        vertex
        for vertex in first_vertices
        if _point_in_rectangle(vertex, second_vertices)
    ]
    contact_candidates.extend(
        vertex
        for vertex in second_vertices
        if _point_in_rectangle(vertex, first_vertices)
    )
    if contact_candidates:
        contact = sum(contact_candidates, pygame.Vector2()) / len(
            contact_candidates
        )
    else:
        first_support = max(
            first_vertices, key=lambda point: point.dot(smallest_axis)
        )
        second_support = min(
            second_vertices, key=lambda point: point.dot(smallest_axis)
        )
        contact = (first_support + second_support) / 2
    return Collision(first, second, smallest_axis, smallest_overlap, contact)


def resolve_collision(collision):
    first = collision.first
    second = collision.second
    inverse_mass_sum = first.inverse_mass + second.inverse_mass
    if inverse_mass_sum == 0:
        return

    correction = collision.normal * collision.penetration / inverse_mass_sum
    first.position -= correction * first.inverse_mass
    second.position += correction * second.inverse_mass

    first_offset = collision.contact_point - first.position
    second_offset = collision.contact_point - second.position
    first_contact_velocity = first.velocity + _angular_velocity_at_point(
        first.angular_velocity, first_offset
    )
    second_contact_velocity = second.velocity + _angular_velocity_at_point(
        second.angular_velocity, second_offset
    )
    relative_velocity = second_contact_velocity - first_contact_velocity
    velocity_along_normal = relative_velocity.dot(collision.normal)
    if velocity_along_normal >= 0:
        return

    first_lever = first_offset.cross(collision.normal)
    second_lever = second_offset.cross(collision.normal)
    denominator = (
        inverse_mass_sum
        + first_lever**2 * first.inverse_inertia
        + second_lever**2 * second.inverse_inertia
    )
    restitution = min(first.restitution, second.restitution)
    impulse = (
        -(1 + restitution) * velocity_along_normal / denominator
    ) * collision.normal

    first.velocity -= impulse * first.inverse_mass
    second.velocity += impulse * second.inverse_mass
    first.angular_velocity -= first_offset.cross(impulse) * first.inverse_inertia
    second.angular_velocity += second_offset.cross(impulse) * second.inverse_inertia

    relative_velocity = (
        second.velocity
        + _angular_velocity_at_point(second.angular_velocity, second_offset)
        - first.velocity
        - _angular_velocity_at_point(first.angular_velocity, first_offset)
    )
    tangent = relative_velocity - collision.normal * relative_velocity.dot(
        collision.normal
    )
    if tangent.length_squared() == 0:
        return
    tangent = tangent.normalize()

    first_tangent_lever = first_offset.cross(tangent)
    second_tangent_lever = second_offset.cross(tangent)
    tangent_denominator = (
        inverse_mass_sum
        + first_tangent_lever**2 * first.inverse_inertia
        + second_tangent_lever**2 * second.inverse_inertia
    )
    friction_magnitude = -relative_velocity.dot(tangent) / tangent_denominator
    coefficient = math.sqrt(first.friction * second.friction)
    maximum_friction = impulse.length() * coefficient
    friction_magnitude = max(
        -maximum_friction, min(friction_magnitude, maximum_friction)
    )
    friction_impulse = tangent * friction_magnitude

    first.velocity -= friction_impulse * first.inverse_mass
    second.velocity += friction_impulse * second.inverse_mass
    first.angular_velocity -= (
        first_offset.cross(friction_impulse) * first.inverse_inertia
    )
    second.angular_velocity += (
        second_offset.cross(friction_impulse) * second.inverse_inertia
    )


def _angular_velocity_at_point(angular_velocity, offset):
    return pygame.Vector2(
        -angular_velocity * offset.y,
        angular_velocity * offset.x,
    )


def _point_in_rectangle(point, vertices):
    sign = None
    for index in range(len(vertices)):
        edge = vertices[(index + 1) % len(vertices)] - vertices[index]
        offset = point - vertices[index]
        cross = edge.cross(offset)
        if abs(cross) < 1e-7:
            continue
        current_sign = cross > 0
        if sign is None:
            sign = current_sign
        elif current_sign != sign:
            return False
    return True
