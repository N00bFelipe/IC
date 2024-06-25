from Obstacle import Obstacle
from pygame.math import Vector2
from Parameters import *
import pygame
import numpy as np

class Robot(Obstacle):

    def __init__(self, start, size = SIZEROBOT, sensorRange = SENSOR_RANGE, sensorAngle = SENSOR_ANGLE, color = (0, 200, 200), label = None):
        Obstacle.__init__(self, start, size, color)
        self.start = Vector2(start)
        self.velocity = Vector2(np.random.random()*VELMAX/2 - VELMAX/4, np.random.random()*VELMAX/2 - VELMAX/4)
        self.orientation = np.arctan2(self.velocity.y, self.velocity.x)
        self.front = self.position + Vector2(self.size*np.cos(self.orientation), self.size*np.sin(self.orientation))
        self.label = 'Robot' if label == None else label
        self.sensorRange = sensorRange
        self.sensorAngle = sensorAngle

    def angle(self, robot):
        return np.arctan2((robot.position - self.position).y, (robot.position - self.position).x)
    
    def avoid_others(self, robots):
        move = Vector2(0, 0)

        for robot in robots:
            if self.inRange(robot) and robot is not self:
                if self.distance(robot) < MIN_DISTANCE:
                    move += self.position - robot.position

        self.setVelocity(self.velocity + move*AVOID_FACTOR)
        
    def distance(self, robot):
        return (self.position - robot.position).length()
    
    def draw(self, screen):
        self.orientation = np.arctan2(self.velocity.y, self.velocity.x)
        self.front = self.position + Vector2(self.size*np.cos(self.orientation), self.size*np.sin(self.orientation))
        pygame.draw.circle(screen, self.color, self.position, self.size)
        pygame.draw.line(screen, (0, 0, 0), self.position, self.front, self.size//5)

    def fly_towards_center(self, robots):
        center = Vector2(0, 0)
        num_neighbors = 0

        for robot in robots:
            if self.inRange(robot):
                center += robot.position
                num_neighbors += 1

        if num_neighbors:
            center /= num_neighbors
            self.setVelocity(self.velocity + (center - self.position)*CENTERING_FACTOR)
            
    def inRange(self, robot):
        if self.distance(robot) < self.sensorRange and np.fabs(self.angle(robot)) < self.sensorAngle:
            return True
        return False 
    
    def keep_within_bounds(self):
        if self.position.x < MARGIN:
            self.setVelocity(Vector2(self.velocity.x + TURN_FACTOR, self.velocity.y))
        if self.position.x > WIDTH - MARGIN:
            self.setVelocity(Vector2(self.velocity.x - TURN_FACTOR, self.velocity.y))
        if self.position.y < MARGIN:
            self.setVelocity(Vector2(self.velocity.x, self.velocity.y + TURN_FACTOR))
        if self.position.y > HEIGHT - MARGIN:
            self.setVelocity(Vector2(self.velocity.x, self.velocity.y - TURN_FACTOR))
    
    def match_velocity(self, robots):
        avgVel = Vector2(0, 0)
        num_neighbors = 0

        for robot in robots:
            if self.inRange(robot):
                avgVel += robot.velocity
                num_neighbors += 1

        if num_neighbors:
            avgVel /= num_neighbors
            self.setVelocity(self.velocity + (avgVel - self.velocity)*MATCHING_FACTOR)   

    def moving(self, robots, dt = DELTAT):
        self.fly_towards_center(robots)
        self.avoid_others(robots)
        self.match_velocity(robots)
        self.keep_within_bounds()   
        self.position += self.velocity*dt  

    def setVelocity(self, velocity):
        if velocity.length() <= VELMAX:
            self.velocity = velocity
        else:
            self.velocity = VELMAX*velocity.normalize()