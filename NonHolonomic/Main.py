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
while len(robots) < NUMROB:
    start = (0.20*WIDTH*random.random(), HEIGHT*random.random())
    if not robots or all((robot.start - start).length() > 2.5*robot.size for robot in robots):
        robots.append(Robot(start, [(0.9*WIDTH, 0.1*HEIGHT), (0.1*WIDTH, 0.8*HEIGHT), (0.9*WIDTH, 0.9*HEIGHT), start]))

obstacles =  Obstacle.matrix((0.35*WIDTH, HEIGHT), (0.65*WIDTH, 0), 6, 2, 25)
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
            if robot.arrived():
                continue
            robot.moving(DELTAT, obstacles + robots)
            robot.resetForce()

        if all(robot.arrived() for robot in robots):
            for robot in robots:
                robot.resetRobot() 
            paused = not paused
    else:
        screen.fill((230, 230, 230))

    for obj in robots + obstacles:
        if isinstance(obj, Robot):
            pygame.draw.rect(screen, obj.color, (*obj.start, 10, 10))
            for goal, i in zip(obj.goals, range(len(obj.goals) - 1)):
                pygame.draw.rect(screen, (255, 223, 0), (*goal, 10, 10))
            obj.draw(screen)
        else:
            pygame.draw.circle(screen, obj.color, obj.position, obj.size)

    pygame.display.flip()