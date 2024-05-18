import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

numbRob = 1
numObs = 10
rangeGoal = 0.5
deltaT = 0.1
defVelMax = 4
defVelMin = 1
defRadiusAcce = 4
defAcceMax = 3
WORLDX, WORLDY = 60, 30

def randomColor():
    r = np.random.rand()
    g = np.random.rand()
    b = np.random.rand()

    return '#%02x%02x%02x' % (int(r*255), int(g*255), int(b*255))

def obsCollum(start, end, rows, colluns, size):
    if colluns > 1 and rows > 1:
        dr = np.linalg.norm(end[1] - start[1])/(rows - 1)
        dc = np.linalg.norm(end[0] - start[0])/(colluns -  1)
    elif rows <= 1 and colluns > 1: 
        dr = 0
        dc = np.linalg.norm(end[0] - start[0])/(colluns -  1)
    elif rows > 1 and colluns <=1: 
        dr = np.linalg.norm(end[1] - start[1])/(rows - 1)
        dc = 0
    else: dr = dc = 0
    obstacles = []
    for j in range(colluns):
        for i in range(rows):
            if j%2 == 0:
                obstacles.append(Obstacle([start[0] + j*dc, start[1] - i*dr], size))
            else:
                obstacles.append(Obstacle([start[0] + j*dc, start[1] - (i + 1/2)*dr], size))
    return obstacles

class Obstacle:

    def __init__(self, position, size, color = '#000000', rangeRep = .3):
        self.position = np.array(position)
        self.size = size
        self.color = color
        self.rangeRep = rangeRep

class Robot(Obstacle):

    NumOfRobots = 0

    def __init__(self, start, goal, size = 0.2, vMax = defVelMax, aMax = defAcceMax, rangeRep = 1, color = None, label = None):
        Obstacle.__init__(self, start, size,  randomColor() if color == None else color, rangeRep)
        Robot.NumOfRobots += 1
        self.start = np.array(start)
        self.goal = np.array(goal)
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
        if self.velocity < 1:
            self.setVelocity(max(min(self.velocity + self.aMax*dt, self.vMax), defVelMin))
            return
        
        for obs in obstacles:
            if obs is self:
                continue
            if self.distFromAnother(obs) < defRadiusAcce:
                self.setAcceleration(-self.aMax)
                break
            else:
                self.setAcceleration(self.aMax)
        self.setVelocity(max(min(self.velocity + self.aceleration*dt, self.vMax), 1)) 

    def moving(self, dt, obstacles):
        self.attForce()
        self.repForce(obstacles)
        self.toAccelerate(dt, obstacles)
        dir = self.force/np.linalg.norm(self.force)
        self.position += self.velocity*dt*dir

    def distFromAnother(self, another):
        return np.linalg.norm(self.position - another.position)
    
    def arrived(self):
        return True if np.linalg.norm(self.position - self.goal) < rangeGoal else False
    
    def attForce(self, katt = 0.01):
        self.force = katt*(self.goal - self.position)

    def repForce(self, obs, krep = 0.01):
        for ob in obs:
            v = self.position - ob.position
            d = np.linalg.norm(v) - ob.size
            R = ob.rangeRep + ob.size
            if d < R and ob is not self:
                self.force += krep*(1/d**2)*((1/d)-(1/R))*(v/d)

    def resetForce(self):
        self.force = 0 

if __name__ == '__main__':

    robots = []
    while len(robots) < numbRob:
        robots.append(Robot([0.20*WORLDX*np.random.rand(), WORLDY*np.random.rand()], [WORLDX*(0.80 + 0.20*np.random.rand()), WORLDY*np.random.rand()]))
    
    #"Matriz" de Obstáculos
    obstacles =  obsCollum([0.4*WORLDX, WORLDY], [0.60*WORLDX, 0], 10, 2, 1.2)

    # Obstáculos aleatórios
    # obstacles = []
    # while len(obstacles) < numObs:
    #     newObs = Obstacle([WORLDX*np.random.rand(), WORLDY*np.random.rand()], 4*np.random.rand()+0.5)

    #     if all(robot.distFromAnother(newObs) > newObs.size + robot.size for robot in robots) and all(np.linalg.norm(robot.goal - newObs.position) > newObs.size + 2*newObs.rangeRep for robot in robots):
    #         obstacles.append(newObs)

    # Gráfico =======================================================================================================
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(0, WORLDX)
    ax.set_ylim(0, WORLDY)
    ax.set_aspect('equal')

    for obs in obstacles:
        ax.add_patch(patches.Circle((obs.position[0], obs.position[1]), obs.size, color = obs.color))
    
    for robot in robots:
        ax.scatter(robot.goal[0], robot.goal[1], color = robot.color,  marker = '*', s=100)
        ax.scatter(robot.start[0], robot.start[1], color = robot.color, marker = 's', s = 50, label = robot.label)
    robot_plot = ax.scatter(-1, -1)

    # ax.legend(loc='upper right')
    plt.pause(3)

    colors, sizes = [], []
    for robot in robots:
        colors.append(robot.color)
        sizes.append(100*robot.size)
    # Gráfico =======================================================================================================

    i = 0
    while any(not robot.arrived() for robot in robots):

        for robot in robots:
            if robot.arrived():
                continue

            robot.moving(deltaT, obstacles + robots)
            robot.resetForce()
            
            # Mostrar trajeto 
            # ax.scatter(robot.position[0], robot.position[1], color = robot.color, s = 100*robot.size)

        robot_plot.set_color(colors)  
        robot_plot.set_sizes(sizes)
        robot_plot.set_offsets([robot.position for robot in robots])

        plt.pause(0.01)
        plt.draw()

        # Salvar capturas 
        # if i % 50:
        #     plt.savefig(f"subplot{i//50}.png")

        i+=1
        if(i > 1000): 
            print("Number of iterations exceeded")
            break

    # plt.savefig(f"subplot.png")
    plt.show(block=True)