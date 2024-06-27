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
        self.orientation = self.normalizeAngle(np.deg2rad(orientation)) 
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
        if (self.position - self.goals[self.goal_index]).length() < RANGEGOAL:
            return True
        
    def angle(self, another):
         return math.atan2((another.position - self.position).y, (another.position - self.position).x) - self.orientation

    def attForce(self, katt = 0.01):
        self.force = katt*(self.goals[self.goal_index] - self.position)

    def distance(self, another):
        return math.fabs((self.position - another.position).length() - (self.size + another.size))
    
    def draw(self, screen):
        pygame.draw.circle(screen, self.color, self.position, self.size)
        pygame.draw.line(screen, (0, 0, 0), self.position, self.front, self.size//10)

    def getGoal(self):
        return self.goals[self.goal_index]

    def kControl(self):
        rho = self.force.length()
        alpha = self.normalizeAngle(-self.orientation + math.atan2(self.force.y, self.force.x))

        kr = 4 * 3
        ka = 8 * 20

        if abs(alpha) > np.pi/2:
            kr = -kr       
            alpha = self.normalizeAngle(alpha-np.pi)

        v = kr*rho + 1*kr/math.fabs(kr)
        w = ka*alpha
        self.setVelocity(v) 
        self.setOmega(w)
        
    def moving(self, dt, obstacles):
        self.attForce()
        self.repForce(obstacles)
        dir = (self.front - self.position).normalize()
        self.kControl()

        self.position += self.velocity*dt*dir
        self.orientation = self.normalizeAngle(self.orientation + dt*self.w)
        self.front = self.position + Vector2(self.size*math.cos(self.orientation), self.size*math.sin(self.orientation))
        self.resetForce()

    @staticmethod
    def normalizeAngle(angle):
        return (angle+np.pi)%(2*np.pi) - np.pi
    
    @staticmethod
    def randomColor():
        r = random.random()
        g = random.random()
        b = random.random()

        return (int(r*255), int(g*255), int(b*255))
    
    def repForce(self, obs, krep = 10):
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

    @classmethod
    def letters(cls, robots, letters):
        P = []
        P.append(Vector2(WIDTH/2 - 6*SIZEROBOT, HEIGHT/2 - 8*SIZEROBOT)) 
        P.append(Vector2(WIDTH/2, HEIGHT/2 - 8*SIZEROBOT))               
        P.append(Vector2(WIDTH/2 + 6*SIZEROBOT, HEIGHT/2 - 8*SIZEROBOT))
        P.append(Vector2(WIDTH/2 + 6*SIZEROBOT, HEIGHT/2 - 4*SIZEROBOT))
        P.append(Vector2(WIDTH/2 + 6*SIZEROBOT, HEIGHT/2))
        P.append(Vector2(WIDTH/2 + 6*SIZEROBOT, HEIGHT/2 + 4*SIZEROBOT))
        P.append(Vector2(WIDTH/2 + 6*SIZEROBOT, HEIGHT/2 + 8*SIZEROBOT))
        P.append(Vector2(WIDTH/2, HEIGHT/2 + 8*SIZEROBOT))
        P.append(Vector2(WIDTH/2 - 6*SIZEROBOT, HEIGHT/2 + 8*SIZEROBOT))
        P.append(Vector2(WIDTH/2 - 6*SIZEROBOT, HEIGHT/2 + 4*SIZEROBOT))
        P.append(Vector2(WIDTH/2 - 6*SIZEROBOT, HEIGHT/2))
        P.append(Vector2(WIDTH/2 - 6*SIZEROBOT, HEIGHT/2 - 4*SIZEROBOT)) 
        P.append(Vector2(WIDTH/2, HEIGHT/2))

        N = []
        for robot in robots:
            N.append(robot.start)

        letA = [N[0], P[1], N[2], P[3], P[4], P[5], P[6], N[7], P[8], P[9], P[10], P[11], P[12]]
        letB = [P[0], P[1], N[2], P[3], N[4], P[5], P[6], P[7], P[8], P[9], P[10], P[11], P[12]]
        letC = [P[0], P[1], P[2], N[3], N[4], N[5], P[6], P[7], P[8], P[9], P[10], P[11], N[12]]
        letD = [P[0], P[1], N[2], P[3], P[4], P[5], N[6], P[7], P[8], P[9], P[10], P[11], N[12]]
        letE = [P[0], P[1], P[2], N[3], N[4], N[5], P[6], P[7], P[8], P[9], P[10], P[11], P[12]]
        letF = [P[0], P[1], P[2], N[3], N[4], N[5], N[6], N[7], P[8], P[9], P[10], P[11], P[12]]
        letG = [P[0], P[1], P[2], N[3], P[4], P[5], P[6], P[7], P[8], P[9], P[10], P[11], P[12]]
        letH = [P[0], N[1], N[2], N[3], P[4], P[5], P[6], N[7], P[8], P[9], P[10], P[11], P[12]]
        letI = [P[0], N[1], N[2], N[3], N[4], N[5], N[6], N[7], P[8], P[9], P[10], P[11], N[12]]
        letJ = [P[0], P[1], P[2], N[3], N[4], N[5], N[6], P[7], P[8], N[9], N[10], N[11], P[12]]
        letK = [P[0], N[1], P[2], P[3], N[4], P[5], P[6], N[7], P[8], P[9], P[10], P[11], P[12]]
        letL = [P[0], N[1], N[2], N[3], N[4], N[5], P[6], P[7], P[8], P[9], P[10], P[11], N[12]]
        letM = [N[0], N[1], N[2], P[3], P[4], P[5], P[6], N[7], P[8], P[9], P[10], P[11], P[12]]
        letN = [N[0], N[1], N[2], N[3], N[4], P[5], P[6], N[7], P[8], P[9], P[10], N[11], P[12]]
        letO = [P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7], P[8], P[9], P[10], P[11], N[12]]
        letP = [P[0], P[1], P[2], P[3], P[4], N[5], N[6], N[7], P[8], P[9], P[10], P[11], P[12]]
        letQ = [P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7], P[8], P[9], P[10], P[11], P[12]]
        letR = [P[0], P[1], P[2], P[3], N[4], P[5], P[6], N[7], P[8], P[9], P[10], P[11], P[12]]
        letS = [P[0], P[1], P[2], N[3], P[4], P[5], P[6], P[7], P[8], N[9], P[10], P[11], P[12]]
        letT = [P[0], P[1], P[2], N[3], N[4], N[5], N[6], P[7], N[8], N[9], N[10], N[11], P[12]]
        letU = [P[0], N[1], P[2], P[3], P[4], P[5], P[6], P[7], P[8], P[9], P[10], P[11], N[12]]
        letV = [P[0], N[1], P[2], P[3], P[4], P[5], N[6], P[7], N[8], P[9], P[10], P[11], N[12]]
        letW = [P[0], N[1], P[2], P[3], P[4], P[5], N[6], N[7], N[8], P[9], P[10], P[11], P[12]]
        letX = [P[0], N[1], P[2], P[3], N[4], P[5], P[6], N[7], P[8], P[9], N[10], P[11], P[12]]
        letY = [P[0], N[1], P[2], P[3], N[4], N[5], N[6], P[7], N[8], N[9], N[10], P[11], P[12]]
        letZ = [P[0], P[1], P[2], P[3], N[4], N[5], P[6], P[7], P[8], P[9], N[10], N[11], P[12]]

        for let in letters:
            if let == 'A':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letA):
                        robot.goals.append(letA[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'B':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letB):
                        robot.goals.append(letB[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'C':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letC):
                        robot.goals.append(letC[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'D':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letD):
                        robot.goals.append(letD[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'E':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letE):
                        robot.goals.append(letE[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'F':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letF):
                        robot.goals.append(letF[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'G':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letG):
                        robot.goals.append(letG[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'H':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letH):
                        robot.goals.append(letH[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'I':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letI):
                        robot.goals.append(letI[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'J':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letJ):
                        robot.goals.append(letJ[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'K':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letK):
                        robot.goals.append(letK[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'L':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letL):
                        robot.goals.append(letL[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'M':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letM):
                        robot.goals.append(letM[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'N':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letI):
                        robot.goals.append(letN[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'O':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letO):
                        robot.goals.append(letO[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'P':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letP):
                        robot.goals.append(letP[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'Q':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letQ):
                        robot.goals.append(letQ[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'R':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letR):
                        robot.goals.append(letR[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'S':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letS):
                        robot.goals.append(letS[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'T':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letT):
                        robot.goals.append(letT[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'U':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letU):
                        robot.goals.append(letU[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'V':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letV):
                        robot.goals.append(letV[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'W':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letW):
                        robot.goals.append(letW[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'X':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letX):
                        robot.goals.append(letX[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'Y':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letY):
                        robot.goals.append(letY[i])
                    else:
                        robot.goals.append(robot.start)
            elif let == 'Z':
                for robot, i in zip(robots, range(len(robots))):
                    if i < len(letZ):
                        robot.goals.append(letZ[i])
                    else:
                        robot.goals.append(robot.start)

    def updateGoal(self):
        if self.goal_index < len(self.goals) - 1:
            self.goal_index += 1
            return True
        else:
            self.goal_index = 0
            return False