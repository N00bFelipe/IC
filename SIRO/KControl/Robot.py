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

    def kControl(self, futureOri):
        rho = self.force.length()
        alpha = self.normalizeAngle(-self.orientation + math.atan2(self.force.y, self.force.x))
        beta = self.normalizeAngle(futureOri - math.atan2(self.force.y, self.force.x))

        kr = 4 * 3
        ka = 8 * 20
        kb = -1.5 * 4

        if abs(alpha) > np.pi/2:
            kr = -kr       
            alpha = self.normalizeAngle(alpha-np.pi)
            beta = self.normalizeAngle(beta-np.pi)

        v = kr*rho + 1*kr/math.fabs(kr)
        w = ka*alpha + kb*beta
        self.setVelocity(v) 
        self.setOmega(w)
        
    def moving(self, dt, obstacles):
        self.attForce()
        self.repForce(obstacles)
        dir = (self.front - self.position).normalize()

        tempRobot = Robot(self.position + self.force, [self.goals[self.goal_index]])
        tempRobot.attForce()
        tempRobot.repForce(obstacles)

        self.kControl(math.atan2(tempRobot.force.y, tempRobot.force.x))
        tempRobot = None

        self.position += self.velocity*dt*dir
        self.orientation = self.normalizeAngle(self.orientation + dt*self.w)
        self.front = self.position + Vector2(self.size*math.cos(self.orientation), self.size*math.sin(self.orientation))

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
        p0 = Vector2(WIDTH/2 - 6*SIZEROBOT, HEIGHT/2 - 10*SIZEROBOT)
        p1 = Vector2(WIDTH/2, HEIGHT/2 - 10*SIZEROBOT)
        p2 = Vector2(WIDTH/2 + 6*SIZEROBOT, HEIGHT/2 - 10*SIZEROBOT)
        p3 = Vector2(WIDTH/2 + 6*SIZEROBOT, HEIGHT/2 - 5*SIZEROBOT)
        p4 = Vector2(WIDTH/2 + 6*SIZEROBOT, HEIGHT/2)
        p5 = Vector2(WIDTH/2 + 6*SIZEROBOT, HEIGHT/2 + 5*SIZEROBOT)
        p6 = Vector2(WIDTH/2 + 6*SIZEROBOT, HEIGHT/2 + 10*SIZEROBOT)
        p7 = Vector2(WIDTH/2, HEIGHT/2 + 10*SIZEROBOT)
        p8 = Vector2(WIDTH/2 - 6*SIZEROBOT, HEIGHT/2 + 10*SIZEROBOT)
        p9 = Vector2(WIDTH/2 - 6*SIZEROBOT, HEIGHT/2 + 5*SIZEROBOT)
        p10 = Vector2(WIDTH/2 - 6*SIZEROBOT, HEIGHT/2)
        p11 = Vector2(WIDTH/2 - 6*SIZEROBOT, HEIGHT/2 - 5*SIZEROBOT) 
        p12 = Vector2(WIDTH/2, HEIGHT/2)

        p = []
        for robot in robots:
            p.append(robot.start)

        letA = [p[0], p1, p[2], p3, p4, p5, p6, p[7], p8, p9, p10, p11, p12]
        letB = [p0, p1, p[2], p3, p4, p5, p6, p7, p8, p9, p10, p11, p12]
        letC = [p0, p1, p2, p[3], p[4], p[5], p6, p7, p8, p9, p10, p11, p[12]]
        letD = [p0, p1, p[2], p3, p4, p5, p[6], p7, p8, p9, p10, p11, p[12]]
        letE = [p0, p1, p2, p[3], p[4], p[5], p6, p7, p8, p9, p10, p11, p12]
        letF = [p0, p1, p2, p[3], p[4], p[5], p[6], p[7], p8, p9, p10, p11, p12]
        letG = [p0, p1, p2, p[3], p4, p5, p6, p7, p8, p9, p10, p11, p12]
        letH = [p0, p[1], p[2], p[3], p4, p5, p6, p[7], p8, p9, p10, p11, p12]
        letI = [p[0], p1, p[2], p[3], p[4], p[5], p[6], p7, p[8], p[9], p[10], p[11], p12]
        letJ = [p0, p1, p2, p[3], p[4], p[5], p[6], p7, p8, p[9], p[10], p[11], p12]
        letK = [p0, p[1], p2, p3, p[4], p5, p6, p[7], p8, p9, p10, p11, p12]
        letL = [p0, p[1], p[2], p[3], p[4], p[5], p6, p7, p8, p9, p10, p11, p[12]]
        letM = [p[0], p[1], p[2], p3, p4, p5, p6, p[7], p8, p9, p10, p11, p12]
        letN = [p[0], p[1], p[2], p[3], p[4], p5, p6, p[7], p8, p9, p10, p[11], p12]
        letO = [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p[12]]
        letP = [p0, p1, p2, p3, p4, p[5], p[6], p[7], p8, p9, p10, p11, p12]
        letQ = [p0, p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12]
        letR = [p0, p1, p2, p3, p[4], p5, p6, p[7], p8, p9, p10, p11, p12]
        letS = [p0, p1, p2, p[3], p4, p5, p6, p7, p8, p[9], p10, p11, p12]
        letT = [p0, p1, p2, p[3], p[4], p[5], p[6], p7, p[8], p[9], p[10], p[11], p12]
        letU = [p0, p[1], p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p[12]]
        letV = [p0, p[1], p2, p3, p4, p5, p[6], p7, p[8], p9, p10, p11, p[12]]
        letX = [p0, p[1], p2, p3, p[4], p5, p6, p[7], p8, p9, p[10], p11, p12]
        letY = [p0, p[1], p2, p3, p[4], p[5], p[6], p7, p[8], p[9], p[10], p11, p12]
        letZ = [p0, p1, p2, p3, p[4], p[5], p6, p7, p8, p9, p[10], p[11], p12]

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
            return False