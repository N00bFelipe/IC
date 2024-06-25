from Obstacle import Obstacle
import numpy as np
import Parameters as PRT

class Robot(Obstacle):

    NumOfRobots = 0

    def __init__(self, start, goals, size = 0.2, vMax = PRT.VELMAX, aMax = PRT.ACCEMAX, rangeRep = 1, color = None, label = None):
        Obstacle.__init__(self, start, size,  self.randomColor() if color == None else color, rangeRep)
        Robot.NumOfRobots += 1
        self.start = np.array(start)
        self.whichGoal = 0
        self.goals = [np.array(goal) for goal in goals]
        self.velocity = 0
        self.vMax = abs(vMax)
        self.aceleration = 0
        self.aMax = aMax
        self.force = 0
        self.label = f'Robot {Robot.NumOfRobots}' if label == None else label
    
    def setVelocity(self, v):
        self.velocity = min(abs(v), self.vMax)

    def setAcceleration(self, a):
        self.aceleration = a if abs(a) <= self.aMax else self.aMax*a/abs(a)

    def toAccelerate(self, dt, obstacles):
        if self.velocity < PRT.VELMIN:
            self.setVelocity(max(min(self.velocity + self.aMax*dt, self.vMax), PRT.VELMIN))
            return
        
        self.setAcceleration(self.aMax)
        for obs in obstacles:
            if obs is self:
                continue
            
            if self.distFromAnother(obs) < PRT.RADIUSACCE:
                normSelfStart = np.linalg.norm(self.goals[self.whichGoal] - self.position)
                normSelfObs = np.linalg.norm(obs.position - self.position)
                dot = np.dot(self.goals[self.whichGoal] - self.position, obs.position - self.position)
                angle = np.arccos(dot/(normSelfObs*normSelfStart))

                if np.abs(angle) <= PRT.ANGLE:
                    self.setAcceleration(-self.aMax)
                    break

        self.setVelocity(max(min(self.velocity + self.aceleration*dt, self.vMax), PRT.VELMIN))

    def moving(self, dt, obstacles):
        self.attForce()
        self.repForce(obstacles)
        self.toAccelerate(dt, obstacles)
        dir = self.force/np.linalg.norm(self.force)
        self.position += self.velocity*dt*dir
        self.resetForce()

    def distFromAnother(self, another):
        return np.linalg.norm(self.position - another.position)
    
    def arrived(self):
        if np.linalg.norm(self.position - self.goals[self.whichGoal]) < PRT.RANGEGOAL and self.whichGoal == len(self.goals) - 1:
            return True
        if np.linalg.norm(self.position - self.goals[self.whichGoal]) < PRT.RANGEGOAL:
            self.whichGoal += 1 if self.whichGoal + 1 < len(self.goals) else 0
            return False
    
    def attForce(self, katt = 0.01):
        self.force = katt*(self.goals[self.whichGoal] - self.position)

    def repForce(self, obs, krep = 0.01):
        for ob in obs:
            v = self.position - ob.position
            d = np.linalg.norm(v) - ob.size
            R = ob.rangeRep + ob.size
            if d < R and ob is not self:
                self.force += krep*(1/d**2)*((1/d)-(1/R))*(v/d)

    def resetForce(self):
        self.force = 0 

    @staticmethod
    def randomColor():
        r = np.random.rand()
        g = np.random.rand()
        b = np.random.rand()

        return '#%02x%02x%02x' % (int(r*255), int(g*255), int(b*255))