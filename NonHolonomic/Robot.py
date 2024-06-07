from Obstacle import Obstacle
from pygame.math import Vector2
import random
import math
from Parameters import *
import pygame
import numpy as np

class Robot(Obstacle):

    NumOfRobots = 0

    def __init__(self, start, goals, orientation = ORIENTATION, size = SIZEROBOT, vMax = VELMAX, vMin = VELMIN, aMax = ACCEMAX, w = OMEGA, sensorRange = SENSOR_RANGE, color = None, label = None):
        Obstacle.__init__(self, start, size,  self.randomColor() if color == None else color)
        Robot.NumOfRobots += 1
        self.start = Vector2(start)
        self.orientation = self.normalizeAngle(orientation) 
        self.front = self.position + Vector2(self.size*math.cos(self.orientation), self.size*math.sin(self.orientation))
        self.goal_index = 0
        self.goals = [Vector2(goal) for goal in goals]
        self.velocity = 0
        self.vMax = abs(vMax)
        self.vMin = abs(vMin)
        self.w = w
        self.acceleration = 0
        self.aMax = aMax
        self.force = 0
        self.label = f'Robot {Robot.NumOfRobots}' if label == None else label
        self.sensorRange = sensorRange
    
    def accelerate(self, dt, obstacles):
        if self.velocity < VELMIN:
            self.setVelocity(max(min(self.velocity + self.aMax*dt, self.vMax), self.vMin))
            return
        
        self.setAcceleration(self.aMax)
        for obs in obstacles:
            if obs is self:
                continue
            
            if self.distFromAnother(obs) < self.sensorRange * 0.5:
                normSelfStart = (self.goals[self.goal_index] - self.position).length()
                normSelfObs = (obs.position - self.position).length()
                dot = (self.goals[self.goal_index] - self.position).dot(obs.position - self.position)
                angle = math.acos(dot/(normSelfObs*normSelfStart))

                if angle <= ANGLE:
                    self.setAcceleration(-self.aMax)
                    break
 
        self.setVelocity(max(min(self.velocity + self.acceleration*dt, self.vMax), self.vMin))

    def arrived(self):
        if (self.position - self.goals[self.goal_index]).length() < RANGEGOAL and self.goal_index == len(self.goals) - 1:
            return True
        if (self.position - self.goals[self.goal_index]).length() < RANGEGOAL:
            self.goal_index += 1 if self.goal_index + 1 < len(self.goals) else 0
            return False
        
    def attForce(self, katt = 0.001):
        self.force = katt*(self.goals[self.goal_index] - self.position)

    def distFromAnother(self, another):
        return (self.position - another.position).length() - (self.size + another.size)
    
    def draw(self, screen):
        pygame.draw.circle(screen, self.color, self.position, self.size)
        pygame.draw.line(screen, (0, 0, 0), self.position, self.front, self.size//10)
    
    def moving(self, dt, obstacles):
        self.attForce()
        self.repForce(obstacles)
        self.yaw(dt)
        self.accelerate(dt, obstacles)
        dir = self.force.normalize()
        self.position += self.velocity*dt*(self.front - self.position).normalize()

    @staticmethod
    def normalizeAngle(angle):
        return (angle+np.pi)%(2*np.pi) - np.pi
    
    @staticmethod
    def randomColor():
        r = random.random()
        g = random.random()
        b = random.random()

        return (int(r*255), int(g*255), int(b*255))
    
    def repForce(self, obs, krep = 50):
        for ob in obs:
            v = self.position - ob.position
            d = self.distFromAnother(ob)
            R = self.sensorRange
            if d < R and ob is not self:
                self.force += krep*(1/d**2)*((1/d)-(1/R))*(v/d)

    def resetRobot(self):
        self.goal_index = 0
        self.force = 0
        self.velocity = 0
        self.acceleration = 0

    def resetForce(self):
        self.force = 0

    def setAcceleration(self, a):
        self.acceleration = a if abs(a) <= self.aMax else self.aMax*a/abs(a)

    def setVelocity(self, v):
        self.velocity = min(abs(v), self.vMax)

    def yaw(self, dt):
        angle =  self.normalizeAngle(math.atan2(self.force.y, self.force.x) - math.atan2((self.front - self.position).y, (self.front - self.position).x))

        if math.fabs(angle) >= np.deg2rad(5):
            if angle >= 0:
                self.orientation = self.normalizeAngle(self.orientation + dt*self.w)
            else:
                self.orientation = self.normalizeAngle(self.orientation - dt*self.w)
        
        self.front = self.position + Vector2(self.size*math.cos(self.orientation), self.size*math.sin(self.orientation))
