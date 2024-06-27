import numpy as np
from pygame.math import Vector2
from Parameters import *
from Obstacle import Obstacle
from Robot import Robot
import pygame
from pygame.locals import *
from sys import exit

pygame.init()
clock = pygame.time.Clock()
paused = True

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('Simulation')

robots = []
while len(robots) < NUMROB:
    start = (0.20*WIDTH*np.random.rand(), HEIGHT*np.random.rand())
    robots.append(Robot(start))

goals = [(0.9*WIDTH, 0.1*HEIGHT), (0.1*WIDTH, 0.9*HEIGHT), (0.9*WIDTH, 0.9*HEIGHT)]
obstacles =  Obstacle.matrix([0.35*WIDTH, HEIGHT], [0.65*WIDTH, 0], 10, 2, 16)
obstacles.append(Obstacle(pygame.mouse.get_pos(), 10))
mouse_ind = len(obstacles) - 1

while True:
    clock.tick(FPS)
    
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            exit()
        elif event.type == KEYDOWN:
            if event.key == K_SPACE:
                paused = not paused 
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                obstacles.append(Obstacle(pygame.mouse.get_pos(), obstacles[mouse_ind].size))
            if event.button == 3: 
                if len(obstacles) > mouse_ind + 1:
                    obstacles.pop()
            if event.button == 4:
                obstacles[mouse_ind].size += 5
            if event.button == 5:
                if obstacles[mouse_ind].size > 5:
                    obstacles[mouse_ind].size -= 5
                

    obstacles[mouse_ind].position = Vector2(pygame.mouse.get_pos())

    if not paused:
        screen.fill((255, 255, 255))

        for robot in robots:
            robot.moving(DELTAT, robots, obstacles, goals)
            
    else:
        screen.fill((230, 230, 230))

    for obj in robots + obstacles:
        if isinstance(obj, Robot):
            obj.draw(screen)
            pygame.draw.rect(screen, obj.color, (*obj.start, 10, 10))
            for goal in goals:  
                pygame.draw.rect(screen, (255, 223, 0), (*goal, 10, 10))
        else:
            pygame.draw.circle(screen, obj.color, obj.position, obj.size)

    pygame.display.flip()