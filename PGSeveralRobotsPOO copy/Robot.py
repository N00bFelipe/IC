from pygame.math import Vector2
import pygame
from Obstacle import Obstacle
import numpy as np
from Parameters import *

class Robot(Obstacle):

    NumOfRobots = 0

    def __init__(self, start, goals, size = 5, vMax = VELMAX, aMax = ACCEMAX, sensorRange = SENSOR_RANGE, color = None, label = None):
        Obstacle.__init__(self, start, size,  self.randomColor() if color == None else color)
        Robot.NumOfRobots += 1
        self.start = Vector2(start)
        self.whichGoal = 0
        self.goals = [Vector2(goal) for goal in goals]
        self.velocity = 0
        self.vMax = abs(vMax)
        self.aceleration = 0
        self.aMax = aMax
        self.force = Vector2(0, 0)
        self.orientation = np.arctan2(self.force.y, self.force.x)
        self.front = self.position + Vector2(self.size*np.cos(self.orientation), self.size*np.sin(self.orientation))
        self.label = f'Robot {Robot.NumOfRobots}' if label == None else label
        self.sensorRange = sensorRange
    
    def accelerate(self, dt, obstacles):
        if self.velocity < VELMIN:
            self.setVelocity(max(min(self.velocity + self.aMax*dt, self.vMax), VELMIN))
            return
        
        self.setAcceleration(self.aMax)
        if (self.position - self.goals[self.whichGoal]).length() < self.sensorRange:
            self.setAcceleration(-self.aMax)
        else:
            for obs in obstacles:
                if obs is self:
                    continue
                
                if self.distance(obs) < self.sensorRange:
                    angle = np.arctan2((obs.position - self.position).y, (obs.position - self.position).x) - self.orientation
                    if np.fabs(angle) < ANGLE:
                        self.setAcceleration(-self.aMax)
                        break
 
        self.setVelocity(max(min(self.velocity + self.aceleration*dt, self.vMax), VELMIN))

    def arrived(self):
        if (self.position - self.goals[self.whichGoal]).length() < RANGEGOAL and self.whichGoal == len(self.goals) - 1:
            return True
        if (self.position - self.goals[self.whichGoal]).length() < RANGEGOAL:
            self.whichGoal += 1 if self.whichGoal + 1 < len(self.goals) else 0
            return False
        
    def attForce(self, katt = 0.001):
        self.force = katt*(self.goals[self.whichGoal] - self.position)

    def distance(self, another):
        return (self.position - another.position).length() - (self.size + another.size)
    
    def draw(self, screen):
        self.front = self.position + Vector2(self.size*np.cos(self.orientation), self.size*np.sin(self.orientation))
        pygame.draw.circle(screen, self.color, self.position, self.size)
        pygame.draw.line(screen, (0, 0, 0), self.position, self.front, self.size//5)

    def moving(self, dt, obstacles):
        self.attForce()
        self.repForce(obstacles)
        self.accelerate(dt, obstacles)
        self.position += self.velocity*dt*self.force.normalize()
        self.orientation = np.arctan2(self.force.y, self.force.x)
        self.resetForce()

    def repForce(self, obs, krep = 1):
        for ob in obs:
            v = self.position - ob.position
            d = self.distance(ob)
            R = self.sensorRange
            if d < R and ob is not self:
                self.force += krep*(1/d**2)*((1/d)-(1/R))*(v/d)

    def resetForce(self):
        self.force = 0 

    @staticmethod
    def randomColor():
        r = np.random.rand()
        g = np.random.rand()
        b = np.random.rand()

        return (int(r*255), int(g*255), int(b*255))
    
    def setAcceleration(self, a):
        self.aceleration = a if abs(a) <= self.aMax else self.aMax*a/abs(a)
    def setVelocity(self, v):
        self.velocity = min(abs(v), self.vMax)