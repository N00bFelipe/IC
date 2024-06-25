import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import Parameters as PRT
from Obstacle import Obstacle
from Robot import Robot

if __name__ == '__main__':

    robots = []
    while len(robots) < PRT.NUMROB:
        start = [0.20*PRT.WORLDX*np.random.rand(), PRT.WORLDY*np.random.rand()]
        robots.append(Robot(start, [[0.9*PRT.WORLDX, 0.9*PRT.WORLDY], [0.1*PRT.WORLDX, 0.1*PRT.WORLDY], [0.9*PRT.WORLDX, 0.1*PRT.WORLDY], start]))
        # robots.append(Robot([0.20*PRT.WORLDX*np.random.rand(), PRT.WORLDY*np.random.rand()], [[PRT.WORLDX*(0.80 + 0.20*np.random.rand()), PRT.WORLDY*np.random.rand()]]))
    
    #"Matriz" de Obstáculos
    obstacles =  Obstacle.matrix([0.4*PRT.WORLDX, PRT.WORLDY], [0.60*PRT.WORLDX, 0], 10, 2, 1.2)

    # Obstáculos aleatórios
    # obstacles = []
    # while len(obstacles) < PRT.NUMOBS:
    #     newObs = Obstacle([PRT.WORLDX*np.random.rand(), PRT.WORLDY*np.random.rand()], 4*np.random.rand()+0.5)

    #     if all(robot.distFromAnother(newObs) > newObs.size + robot.size for robot in robots) and all(np.linalg.norm(robot.goal - newObs.position) > newObs.size + 2*newObs.rangeRep for robot in robots):
    #         obstacles.append(newObs)

    # Gráfico =======================================================================================================
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(0, PRT.WORLDX)
    ax.set_ylim(0, PRT.WORLDY)
    ax.set_aspect('equal')

    for obs in obstacles:
        ax.add_patch(patches.Circle((obs.position[0], obs.position[1]), obs.size, color = obs.color))
    
    for robot in robots:
        ax.scatter(robot.start[0], robot.start[1], color = robot.color, marker = 's', s = 50, label = robot.label)
        for goal, i in zip(robot.goals, range(len(robot.goals) - 1)):
            ax.scatter(goal[0], goal[1], color = robot.color,  marker = '*', s=100)
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

            robot.moving(PRT.DELTAT, obstacles + robots)
            
            # Mostrar trajeto 
            # ax.scatter(robot.position[0], robot.position[1], color = robot.color, s = 100*robot.size)

        robot_plot.set_color(colors)  
        robot_plot.set_sizes(sizes)
        robot_plot.set_offsets([robot.position for robot in robots])

        plt.pause(PRT.PAUSE)
        plt.draw()

        # Salvar capturas 
        # if i % 50:
        #     plt.savefig(f"subplot{i//50}.png")

        i+=1
        if(i > 5000): 
            print("Number of iterations exceeded")
            break

    # plt.savefig(f"subplot.png")
    plt.show(block=True)