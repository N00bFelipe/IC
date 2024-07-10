import random
from Parameters import *
from Robot import Robot
import pygame
from pygame.locals import *
from sys import exit

pygame.init()
clock = pygame.time.Clock()
paused = True

screen = pygame.display.set_mode((WIDTH, HEIGHT), RESIZABLE)
pygame.display.set_caption('Simulation')

robots = []
while len(robots) < NUMROB:
    if len(robots) > NUMROB//2:
        robots.append(Robot((WIDTH*random.random(), HEIGHT*random.random()), (-BIAS, 0), color = (0, 200, 200)))
    else:
        robots.append(Robot((WIDTH*random.random(), HEIGHT*random.random()), (BIAS, 0), color = (255, 0, 0)))

while True:
    clock.tick(FPS)
    
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            exit()
        elif event.type == KEYDOWN:
            if event.key == K_SPACE:
                paused = not paused 

    if not paused:
        screen.fill((15, 15, 15))
    
        for robot in robots:
            robot.moving(robots)
            
    else:
        screen.fill((20, 20, 20))
 
    pygame.draw.line(screen, (255, 255, 255), (MARGIN, MARGIN), (WIDTH - MARGIN, MARGIN))
    pygame.draw.line(screen, (255, 255, 255), (WIDTH - MARGIN, MARGIN), (WIDTH - MARGIN, HEIGHT - MARGIN))
    pygame.draw.line(screen, (255, 255, 255), (WIDTH - MARGIN, HEIGHT - MARGIN), (MARGIN, HEIGHT - MARGIN))
    pygame.draw.line(screen, (255, 255, 255), (MARGIN, HEIGHT - MARGIN), (MARGIN, MARGIN))

    for robot in robots:
        robot.draw(screen)

    pygame.display.flip()