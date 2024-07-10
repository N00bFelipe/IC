from pygame.math import Vector2
import pygame
from Obstacle import Obstacle
import numpy as np
from Parameters import *

class Robot(Obstacle):

    NumOfRobots = 0

    def __init__(self, start, size = 5, vMax = VELMAX, aMax = ACCEMAX, sensorRange = SENSOR_RANGE, color = (0, 200, 200), label = None):
        Obstacle.__init__(self, start, size,  color)
        Robot.NumOfRobots += 1
        self.leader = False
        self.start = Vector2(start)
        self.goals = []
        self.whichGoal = 0
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
        goalIndex = self.whichGoal if self.leader else 0
        if (self.position - self.goals[goalIndex]).length() < self.sensorRange:
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
        
        if self.leader:
            for obs in obstacles:
                if isinstance(obs, Robot) and self.distance(obs) > RANGETOACCEFROMANOTHER:
                    self.setAcceleration(-self.aMax)
                    break

        self.setVelocity(max(min(self.velocity + self.aceleration*dt, self.vMax), VELMIN))

    def arrived(self, robots):
        if self.leader:
            if (self.position - self.goals[self.whichGoal]).length() < RANGEGOAL:
                self.changeLeader(robots)
        
    def attForce(self, katt = 0.0002):
        if self.leader:
            self.force = 5*katt*(self.goals[self.whichGoal] - self.position)
        else:
            self.force = katt*(self.goals[0] - self.position)

    def distance(self, another):
        return (self.position - another.position).length() - (self.size + another.size)
    
    def draw(self, screen):
        self.front = self.position + Vector2(self.size*np.cos(self.orientation), self.size*np.sin(self.orientation))
        if self.leader:
            pygame.draw.circle(screen, (200, 0, 0), self.position, self.size)
        else:
            pygame.draw.circle(screen, self.color, self.position, self.size)
        pygame.draw.line(screen, (0, 0, 0), self.position, self.front, self.size//5)

    def changeLeader(self, robots):
        if robots.index(self) + 1 >= len(robots):
            robots[0].leader = True
        else:
            robots[robots.index(self) + 1].leader = True
        self.leader = False
        if self.whichGoal + 1 < len(self.goals):
            self.whichGoal += 1
        else:
            self.whichGoal = 0

    def moving(self, dt, robots, obstacles, goals):
        self.setLeader(robots)
        self.setGoals(robots, goals)
        self.arrived(robots)
        self.attForce()
        self.repForce(robots + obstacles)
        self.accelerate(dt, robots + obstacles)
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

    def setGoals(self, robots, goals):
        if self.leader:
            self.goals = goals
        else:
            for robot in robots:
                if robot.leader:
                    temp = robot
                    break
            self.goals = [temp.position + (temp.position - temp.goals[temp.whichGoal]).normalize()*DISTLIDER] 
    
    @staticmethod
    def setLeader(robots):
        if all(not robot.leader for robot in robots):
            robots[0].leader = True

    def setVelocity(self, v):
        self.velocity = min(abs(v), self.vMax)