import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

numbRob = 15
numObs = 10
rangeGoal = 0.5
deltaT = 0.1
WORLDX, WORLDY = 60, 30

def randomColor():
    r = np.random.rand()
    g = np.random.rand()
    b = np.random.rand()

    return '#%02x%02x%02x' % (int(r*255), int(g*255), int(b*255))

def obsCollum(start, end, rows, colluns, size):
    obstacles = []
    dc = np.linalg.norm(end[0] - start[0])/(colluns -  1)
    dr = np.linalg.norm(end[1] - start[1])/(rows - 1)
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

    def __init__(self, start, goal, size = 0.2, v = 2, vMax = 4, aMax = 1.5, rangeRep = 1, color = None, label = None):
        Obstacle.__init__(self, start, size,  randomColor() if color == None else color, rangeRep)
        Robot.NumOfRobots += 1
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.force = 0
        self.velocity = min(abs(v), vMax)
        self.vMax = vMax
        self.aceleration = 0
        self.aMax = aMax
        self.label = f'Robot {Robot.NumOfRobots}' if label == None else label
    
    def setVelocity(self, v):
        self.velocity = min(abs(v), self.vMax)

    def setAcceleration(self, a):
        self.aceleration = a if abs(a) <= self.aMax else self.aMax*a/abs(a)

    def toAccelerate(self, dt, obstacles):
        for obs in obstacles:
            if obs is self:
                continue
            if self.distFromAnother(obs) < 3:
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
    
    def pick(self):
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
    obstacles =  obsCollum([0.4*WORLDX, WORLDY], [0.60*WORLDX, 0], 10, 4, 1.2)

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
    while any(not robot.pick() for robot in robots):
        
        # Salvar capturas 
        # if i == 100:
        #     plt.savefig(f"subplot t={t:.0f}s.png")
        #     i = 0

        for robot in robots:

            if robot.pick():
                continue

            robot.moving(deltaT, obstacles + robots)
            robot.resetForce()
            
            # Mostrar trajeto 
            # ax.scatter(robots[i][0], robots[i][1], color = colors[i], s=10)

        robot_plot.set_color(colors)  
        robot_plot.set_sizes(sizes)
        robot_plot.set_offsets([robot.position for robot in robots])

        plt.pause(0.01)
        plt.draw()

        i+=1
        if(i > 1000): 
            print("Number of iterations exceeded")
            break

    plt.show(block=True)