from Obstacle import Obstacle
from pygame.math import Vector2
import random
import math
from Parameters import *
import pygame
import numpy as np

class Robot(Obstacle):

    NumOfRobots = 0

    def __init__(self, start, goals, orientation = ORIENTATION, size = SIZEROBOT, vMax = VELMAX, w = OMEGA, sensorRange = SENSOR_RANGE, color = None, label = None):
        Obstacle.__init__(self, start, size,  self.randomColor() if color == None else color)
        Robot.NumOfRobots += 1
        self.orientation = self.normalizeAngle(orientation) 
        self.start = Vector2(start)
        self.front = self.position + Vector2(self.size*math.cos(self.orientation), self.size*math.sin(self.orientation))
        self.goal_index = 0
        self.goals = [Vector2(goal) for goal in goals]
        self.velocity = 0
        self.vMax = abs(vMax)
        self.w = w
        self.force = 0
        self.label = f'Robot {Robot.NumOfRobots}' if label == None else label
        self.sensorRange = sensorRange
    
    def arrived(self):
        if (self.position - self.goals[self.goal_index]).length() < RANGEGOAL and self.goal_index == len(self.goals) - 1:
            return True
        if (self.position - self.goals[self.goal_index]).length() < RANGEGOAL:
            self.goal_index += 1 if self.goal_index + 1 < len(self.goals) else 0
            return False
        
    def angle(self, another):
         return math.atan2((another.position - self.position).y, (another.position - self.position).x) - self.orientation

    def attForce(self, katt = 0.001):
        self.force = katt*(self.goals[self.goal_index] - self.position)

    def distance(self, another):
        return math.fabs((self.position - another.position).length() - (self.size + another.size))
    
    def draw(self, screen):
        pygame.draw.circle(screen, self.color, self.position, self.size)
        pygame.draw.line(screen, (0, 0, 0), self.position, self.front, self.size//10)

    def kControl(self, futurePos):
        rho = (self.position - futurePos).length()
        alpha = self.normalizeAngle(-self.orientation + math.atan2(futurePos.y, futurePos.x))
        beta = self.normalizeAngle(0 - math.atan2(futurePos.y, futurePos.x))

        kr = 4 *10
        ka = 8 *10
        kb = -1.5 *10

        if abs(alpha) > np.pi/2:
            kr = -kr       
            alpha = self.normalizeAngle(alpha-np.pi)
            beta = self.normalizeAngle(beta-np.pi)

        self.setVelocity(kr*rho) 
        self.setOmega(ka*alpha + kb*beta)

    def moving(self, dt, obstacles):
        self.attForce()
        self.repForce(obstacles)
        # self.yaw(dt)
        dir = (self.front - self.position).normalize()
        self.kControl(self. position + self.velocity*dt*dir)
        
        self.position += self.velocity*dt*dir
        self.orientation = self.normalizeAngle(self.orientation + dt*self.w)

    @staticmethod
    def normalizeAngle(angle):
        return (angle+np.pi)%(2*np.pi) - np.pi
    
    def pdControl(self, obstacles):
        minor = self.sensorRange
        v0 = self.velocity

        for obs in obstacles:
            if obs is self:
                continue

            if self.distance(obs) < self.sensorRange:
                if self.distance(obs) < minor and math.fabs(self.angle(obs)) <= ANGLE:
                    minor = self.distance(obs)

        vp = 0.5 * self.vMax * minor / self.sensorRange
        vd = 0.5 * minor / (math.fabs(v0 - self.velocity) if v0 - self.velocity != 0 else 1)
        self.setVelocity(vp + vd)
        
    @staticmethod
    def randomColor():
        r = random.random()
        g = random.random()
        b = random.random()

        return (int(r*255), int(g*255), int(b*255))
    
    def repForce(self, obs, krep = 50):
        for ob in obs:
            if ob is self:
                continue
            v = self.position - ob.position
            d = self.distance(ob)
            R = self.sensorRange
            if d < R and ob is not self:
                self.force += krep*(1/d**2)*((1/d)-(1/R))*(v/d)

    def resetRobot(self):
        self.goal_index = 0
        self.force = 0
        self.velocity = 0

    def resetForce(self):
        self.force = 0

    def setVelocity(self, v):
        self.velocity = max(min(v, self.vMax), -self.vMax)

    def setOmega(self, w):
        self.w =  max(min(w, OMEGA), -OMEGA)

    def yaw(self, dt):
        angle =  self.normalizeAngle(math.atan2(self.force.y, self.force.x) - self.orientation)

        if math.fabs(angle) >= np.deg2rad(5):
            if angle >= 0:
                self.orientation = self.normalizeAngle(self.orientation + dt*self.w)
            else:
                self.orientation = self.normalizeAngle(self.orientation - dt*self.w)
        
        self.front = self.position + Vector2(self.size*math.cos(self.orientation), self.size*math.sin(self.orientation))

    # def accelerate(self, dt, obstacles):
    #     if self.velocity < VELMIN:
    #         self.setVelocity(max(min(self.velocity + self.aMax*dt, self.vMax), self.vMin))
    #         return
        
    #     self.setAcceleration(self.aMax)
    #     for obs in obstacles:
    #         if obs is self:
    #             continue
            
    #         if self.distance(obs) < self.sensorRange * 0.5:
    #             normSelfStart = (self.goals[self.goal_index] - self.position).length()
    #             normSelfObs = (obs.position - self.position).length()
    #             dot = (self.goals[self.goal_index] - self.position).dot(obs.position - self.position)
    #             angle = math.acos(dot/(normSelfObs*normSelfStart))

    #             if angle <= ANGLE:
    #                 self.setAcceleration(-self.aMax)
    #                 break
 
    #     self.setVelocity(max(min(self.velocity + self.acceleration*dt, self.vMax), self.vMin))

    # def setAcceleration(self, a):
    #     self.acceleration = a if abs(a) <= self.aMax else self.aMax*a/abs(a)