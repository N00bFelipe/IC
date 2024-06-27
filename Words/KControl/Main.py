from pygame.math import Vector2
import random
from Parameters import *
from Obstacle import Obstacle
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
for i in range(13):
    if i >= 8:
        robots.append(Robot((0.2*WIDTH, (14 - i)*HEIGHT/7), [(0, 0)], 0, color=(0, 200, 200)))
    elif i == 0:
        robots.append(Robot((0.2*WIDTH, HEIGHT/7), [(0, 0)], 0, color=(0, 200, 200)))
    else:
        robots.append(Robot((0.8*WIDTH, i*HEIGHT/8), [(0, 0)], 0, color=(0, 200, 200)))
    robots[i].goals.pop()

Robot.letters(robots, 'WILLIAM')

obstacles =  Obstacle.matrix((0.35*WIDTH, HEIGHT), (0.65*WIDTH, 0), 0, 0, 25)
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
        screen.fill((15, 15, 15))
    
        for robot in robots:
            robot.moving(DELTAT, obstacles + robots)

        if all(robot.arrived() for robot in robots):
            for robot in robots:
                robot.updateGoal()
            pygame.time.delay(1000)
    else:
        screen.fill((20, 20, 20))

    for obj in robots + obstacles:
        if isinstance(obj, Robot):
            pygame.draw.rect(screen, obj.color, (*obj.start, 10, 10))
            obj.draw(screen)
        else:
            pygame.draw.circle(screen, obj.color, obj.position, obj.size)

    pygame.display.flip()