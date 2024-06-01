import numpy as np
import Parameters as PRT
from Obstacle import Obstacle
from Robot import Robot
import pygame
from pygame.locals import *
from sys import exit

pygame.init()
clock = pygame.time.Clock()
paused = False

screen = pygame.display.set_mode((PRT.WIDTH, PRT.HEIGHT))
pygame.display.set_caption('Simulation')

robots = []
while len(robots) < PRT.NUMROB:
    start = [0.20*PRT.WIDTH*np.random.rand(), PRT.HEIGHT*np.random.rand()]
    robots.append(Robot(start, [[0.9*PRT.WIDTH, 0.1*PRT.HEIGHT], [0.1*PRT.WIDTH, 0.9*PRT.HEIGHT], [0.9*PRT.WIDTH, 0.9*PRT.HEIGHT], start]))

obstacles =  Obstacle.matrix([0.4*PRT.WIDTH, PRT.HEIGHT], [0.60*PRT.WIDTH, 0], 10, 2, 20)

while True:
    screen.fill((255, 255, 255))
    clock.tick(PRT.FPS)
    
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            exit()
        elif event.type == KEYDOWN:
            if event.key == K_SPACE:
                paused = not paused 

    if not paused:
        for robot in robots:
            if robot.arrived():
                continue
            robot.moving(PRT.DELTAT, obstacles + robots)
            robot.resetForce()

    for obj in robots + obstacles:
        pygame.draw.circle(screen, obj.color, obj.position, obj.size)
        if isinstance(obj, Robot):
            pygame.draw.rect(screen, obj.color, (*obj.start, 10, 10))
            for goal, i in zip(obj.goals, range(len(obj.goals) - 1)):
                pygame.draw.rect(screen, robot.color, (*goal, 10, 10))


    pygame.display.update()
        
    