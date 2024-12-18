import pygame
import math

import firebase_admin
from firebase_admin import credentials, db

cred = credentials.Certificate('key.json')

firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://projectiot-ee562-default-rtdb.europe-west1.firebasedatabase.app/'
})
ref = db.reference('/SensorData')

pygame.init()

width, height = 800, 600
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Drone Equilibration")

white = (255, 255, 255)
black = (0, 0, 0)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (0, 0, 255)

def draw_drone(screen, x, y, yaw, pitch, roll):
    arm_length = 60
    body_radius = 20
    
    arm_positions = []
    for angle in [0, 90, 180, 270]:
        yaw_rad = math.radians(angle + yaw)
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)

        arm_x = x + arm_length * math.cos(yaw_rad) * math.cos(pitch_rad)
        arm_y = y + arm_length * math.sin(yaw_rad) * math.cos(roll_rad)
        print(arm_x)
        print(arm_y)
        print(yaw, pitch, roll)
        arm_positions.append((arm_x, arm_y))
    
    pygame.draw.line(screen, red, arm_positions[0], arm_positions[2], 5)
    pygame.draw.line(screen, green, arm_positions[1], arm_positions[3], 5)
    
    pygame.draw.circle(screen, black, (x, y), body_radius)
    
    for arm_x, arm_y in arm_positions:
        pygame.draw.circle(screen, blue, (int(arm_x), int(arm_y)), 10)

running = True
clock = pygame.time.Clock()

yaw, pitch, roll = 0, 0, 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    data = ref.get()
    
    yaw = data['Yaw']
    pitch = data['Pitch']
    roll = data['Roll']

    screen.fill(white)

    draw_drone(screen, width // 2, height // 2, yaw, pitch, roll)

    pygame.display.flip()

    clock.tick(30)

pygame.quit()
