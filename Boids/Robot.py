from Obstacle import Obstacle
from Parameters import *
import pygame
import numpy as np

class Robot(Obstacle):

    def __init__(self, start, bias, size = SIZEROBOT, color = (0, 200, 200), label = None):
        Obstacle.__init__(self, start, size, color)
        self.bias = np.array(bias)
        self.start = np.array(start)
        self.velocity = np.array([np.random.random()*VELMAX/2 - VELMAX/4, np.random.random()*VELMAX/2 - VELMAX/4])
        self.orientation = np.arctan2(self.velocity[1], self.velocity[0])
        self.front = self.position + np.array([self.size*np.cos(self.orientation), self.size*np.sin(self.orientation)])
        self.label = 'Robot' if label == None else label

    def angle(self, robot):
        return np.arctan2((robot.position - self.position)[1], (robot.position - self.position)[0])
    
    def avoid_others(self, robots):
        move = 0

        for robot in robots:
            # if self.inRange(robot) and robot is not self:
            if self.distance(robot) < SENSOR_RANGE_MIN:
                if self.distance(robot) < 3*self.size and self.distance(robot) > 0:
                    self.setVelocity(np.linalg.norm(self.velocity)*self.normalize(self.position - robot.position))
                move += self.position - robot.position
                
        self.setVelocity(self.velocity + move*AVOID_FACTOR)
        
    def distance(self, robot):
        return np.linalg.norm(self.position - robot.position)
    
    def draw(self, screen):
        self.orientation = np.arctan2(self.velocity[1], self.velocity[0])
        self.front = self.position + np.array([self.size*np.cos(self.orientation), self.size*np.sin(self.orientation)])
        pygame.draw.circle(screen, self.color, self.position, self.size)
        pygame.draw.line(screen, (0, 0, 0), self.position, self.front, self.size//5)

    def fly_towards_center(self, robots):
        center = 0
        num_neighbors = 0

        for robot in robots:
            if self.inRange(robot):
                center += robot.position
                num_neighbors += 1

        if num_neighbors:
            center /= num_neighbors
            self.setVelocity(self.velocity + (center - self.position)*CENTERING_FACTOR)
            
    def inRange(self, robot):
        if self.distance(robot) < SENSOR_RANGE_MAX and self.distance(robot) > SENSOR_RANGE_MIN  and np.fabs(self.angle(robot)) < SENSOR_ANGLE:
            return True
        return False 
    




#olaaa






    def keep_within_bounds(self):
        if self.position[0] < MARGIN:
            self.setVelocity(np.array([self.velocity[0] + TURN_FACTOR, self.velocity[1]]))
        if self.position[0] > WIDTH - MARGIN:
            self.setVelocity(np.array([self.velocity[0] - TURN_FACTOR, self.velocity[1]]))
        if self.position[1] < MARGIN:
            self.setVelocity(np.array([self.velocity[0], self.velocity[1] + TURN_FACTOR]))
        if self.position[1] > HEIGHT - MARGIN:
            self.setVelocity(np.array([self.velocity[0], self.velocity[1] - TURN_FACTOR]))
    
    def match_velocity(self, robots):
        avgVel = 0
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

    @staticmethod
    def normalize(vector):
        return vector/np.linalg.norm(vector)

    def setVelocity(self, velocity):
        velocity = velocity + self.bias
        if np.linalg.norm(velocity) > VELMAX:
            self.velocity = VELMAX*self.normalize(velocity)
        elif np.linalg.norm(velocity) < VELMAX/3:
            self.velocity = (VELMAX/3)*self.normalize(velocity)
        else:
            self.velocity = velocity