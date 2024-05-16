import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

numbRob = 10
numObs = 10
rangeGoal = 0.5
deltatT = 0.1
WORLDX, WORLDY = 60, 30

def randomColor():
    r = np.random.rand()
    g = np.random.rand()
    b = np.random.rand()

    return '#%02x%02x%02x' % (int(r*255), int(g*255), int(b*255))

def obsCollum(start, end, number, size):
    obstacles = []
    dv = np.linalg.norm(np.array(end) - np.array(start))/(number-1)
    for i in range(number):
        obstacles.append(Obstacle([start[0], start[1] - i*dv], size))
    return obstacles, dv

class Obstacle:
    def __init__(self, position, size, color = '#000000', rangeRep = .5):
        self.position = np.array(position)
        self.size = size
        self.color = color
        self.rangeRep = rangeRep

class Robot(Obstacle):

    NumOfRobots = 0

    def __init__(self, start, goal, size = 0.5, v = 1, a = 0, vMax = 2, aMax = 2, color = None, label = None):
        Obstacle.__init__(self, start, size,  randomColor() if color == None else color, 0.1)
        Robot.NumOfRobots += 1
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.force = 0
        self.velocity = max(v, vMax)
        self.maxVelocity = vMax
        self.aceleration = max(a, aMax)
        self.maxAceleration = aMax
        self.label = f'Robot {Robot.NumOfRobots}' if label == None else label
    
    def setVelocity(self, v):
        self.velocity = max(v, self.vMax)

    def setAcceleration(self, a):
        self.aceleration = max(a, self.aMax)

    def moving(self, dt):
        dir = self.force/np.linalg.norm(self.force)
        self.position += self.velocity*dt*dir + self.aceleration*dt*dt*dir/2

    def distFromAnother(self, another):
        return np.linalg.norm(self.position - another.position)
    
    def pick(self):
        return True if np.linalg.norm(self.position - self.goal) < rangeGoal else False
    
    def attForce(self, katt = 0.01):
        self.force = katt*(self.goal - self.position)

    def repForce(self, obs, krep = .01):
        for ob in obs:
            v = self.position - ob.position
            d = np.linalg.norm(v) - ob.size
            R = ob.rangeRep + ob.size
            if d < R and ob is not self:
                dir = (self.position - ob.position)/d
                self.force += krep*(1/d**2)*((1/d)-(1/R))*(v/d)

    def resetForce(self):
        self.force = 0 

if __name__ == '__main__':

    robots = []
    while len(robots) < numbRob:
        robots.append(Robot([0.20*WORLDX*np.random.rand(), WORLDY*np.random.rand()], [WORLDX*(0.80 + 0.20*np.random.rand()), WORLDY*np.random.rand()]))
    
    # Obstáculos aleatórios
    # obstacles = []
    # while len(obstacles) < numObs:
    #     newObs = Obstacle([WORLDX*np.random.rand(), WORLDY*np.random.rand()], 4*np.random.rand()+0.5)

    #     if all(robot.distFromAnother(newObs) > newObs.size + robot.size for robot in robots) and all(np.linalg.norm(robot.goal - newObs.position) > newObs.size + 2*newObs.rangeRep for robot in robots):
    #         obstacles.append(newObs)

    #Coluna de Obstáculo
    i = 1
    obs = []
    obstacles, dv = obsCollum([0.25*WORLDX, WORLDY], [0.25*WORLDX, 0], 10, 1)
    while 0.25 + 0.1*i <= 0.75:
        if i%2 != 0:
            obs, _ = obsCollum([(0.25 + 0.1*i)*WORLDX, WORLDY - dv/2], [(0.25 + 0.1*i)*WORLDX, -dv/2], 10, 1)
        else:
            obs, _ = obsCollum([(0.25 + 0.1*i)*WORLDX, WORLDY], [(0.25 + 0.1*i)*WORLDX, 0], 10, 1)
        obstacles.extend(obs)
        i += 1

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
    ax.legend(loc='upper right')
    plt.pause(3)

    colors = []
    for robot in robots:
        colors.append(robot.color)
    # Gráfico =======================================================================================================

    i=50
    while any(not robot.pick() for robot in robots):
        
        # Salvar capturas 
        # if i == 50:
        #     plt.savefig(f"subplot t={t:.0f}s.png")
        #     i = 0

        for robot in robots:

            if robot.pick():
                continue

            robot.attForce()
            robot.repForce(obstacles)
            robot.repForce(robots)
            robot.moving(deltatT)
            robot.resetForce()
            
            # Mostrar trajeto 
            # ax.scatter(robots[i][0], robots[i][1], color = colors[i], s=10)

        robot_plot.set_color(colors)  
        robot_plot.set_sizes(np.full(len(robots), 10))
        robot_plot.set_offsets([robot.position for robot in robots])

        plt.pause(0.01)
        plt.draw()

        i+=1
        if(i > 500): 
            print("Number of iterations exceeded")
            break

    plt.show(block=True)