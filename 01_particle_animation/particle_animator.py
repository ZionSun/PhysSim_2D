import pygame 
import random

class Particle:
    def __init__(self, radius, color, s, v, a):
        self.radius = radius
        self.color = color
        self.s = s 
        self.v = v 
        self.a = a

    def update(self, dt):
        self.v += self.a * dt
        self.s += self.v * dt 

    def handle_collisions(self, width, height, res):
        hasCollided = False
        if self.s.x - self.radius <= 0:
            self.s.x = self.radius
            self.v.x *= -res
            hasCollided = True

        elif self.s.x + self.radius >= width:
            self.s.x = width - self.radius
            self.v.x *= -res
            hasCollided = True

        if self.s.y - self.radius <= 0:
            self.s.y = self.radius
            self.v.y *= -res
            hasCollided = True

        elif self.s.y + self.radius >= height:
            self.s.y = height - self.radius
            self.v.y *= -res
            hasCollided = True

        return hasCollided

    def draw(self, screen):
        pygame.draw.circle(
            screen,
            self.color,
            (round(self.s.x), round(self.s.y)),
            self.radius,
        )

pygame.init()

WIDTH = 800
HEIGHT = 600 

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("2D Particle Simulator")

clock = pygame.time.Clock()

particles = []
restitution = 0.85

def add_particle():
    particle_position = pygame.Vector2(WIDTH / 2, HEIGHT / 2)
    particle_velocity = pygame.Vector2(random.randint(-300, 300), random.randint(-300, 300))
    particle_acceleration = pygame.Vector2(0, 300)
    particle_radius = 20
    particle_color = (70, 160, 255)

    particle = Particle(particle_radius, particle_color, particle_position, particle_velocity, particle_acceleration)
    particles.append(particle)

def update_screen(particles, screen, dt):
     screen.fill((25, 25, 35))
     for particle in particles:
          particle.update(dt)
          particle.draw(screen)
    
def handle_collisions(particles, width, height, res):
     for particle in particles:
        particle.handle_collisions(width, height, res)

running = True
while running:
    dt = clock.tick(60) / 1000

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_r:
                particles.clear()
                
            if event.key == pygame.K_n:
                add_particle()

    handle_collisions(particles, WIDTH, HEIGHT, restitution)
    update_screen(particles, screen, dt)

    pygame.display.flip()




pygame.quit()