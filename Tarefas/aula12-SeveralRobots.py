import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

WORLDX, WORLDY = 60, 30
NumRob = 10
vel=2

def att_force(q, goal, katt=.01):
    return katt*(goal - q)

def rep_force(q, obs, R=3, krep=.1, axis=1):
    
    # Obstáculo: (x, y, r)
    v = q - obs[0:2]
    d = np.linalg.norm(v, axis=axis) - obs[2]
    d = d.reshape((len(v) ,1)) if axis==1 else d
    
    rep = (1/d**2)*((1/d)-(1/R))*(v/d)    
    
    invalid = np.squeeze(d > R)
    rep[invalid, :] = 0
    
    return krep*rep

def random_color():
    r = np.random.rand()
    g = np.random.rand()
    b = np.random.rand()

    return '#%02x%02x%02x' % (int(r*255), int(g*255), int(b*255))

def unit_vector(vector):
    return vector/np.linalg.norm(vector)

if __name__ == '__main__':

    goals, robots, obs = [], [], []

    N=0
    while N < NumRob:
        newgoal = np.array([WORLDX*np.random.rand(), WORLDY*np.random.rand()])
        newrobot = np.array([WORLDX*np.random.rand(), WORLDY*np.random.rand()])

        if(all(np.linalg.norm(newgoal - g) > 1 for g in goals) and all(np.linalg.norm(newrobot - r) > 1 for r in robots) and np.linalg.norm(newgoal - newrobot) > WORLDX/3):
            goals.append(newgoal)
            robots.append(newrobot)
            N+=1

    # Obstáculos: (x, y, r)
    N=np.random.randint(10, 20)
    while len(obs) < N:
        newobstacle=np.array([WORLDX*np.random.rand(), WORLDY*np.random.rand(), 4*np.random.rand()+0.5])

        if(all(np.linalg.norm(g - newobstacle[:2]) > newobstacle[2] + 5*newobstacle[2] for g in goals) and all(np.linalg.norm(r - newobstacle[:2]) > newobstacle[2] + 1 for r in robots)):
            obs.append(newobstacle)

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(0, WORLDX)
    ax.set_ylim(0, WORLDY)
    ax.set_aspect('equal')
    time_text = ax.text(0, -0.1, '', transform=ax.transAxes, ha='left')

    for ob in obs:
        ax.add_patch(patches.Circle((ob[0], ob[1]), ob[2], color='k'))
    
    colors = []
    for goal, robot in zip(goals, robots):
        colors.append(random_color())
        ax.scatter(goal[0], goal[1], color=colors[-1],  marker='*', s=100)
        ax.scatter(robot[0], robot[1], color=colors[-1], marker='s', s=50, label=f"Robô {len(colors)}")

    robot_plot = ax.scatter(-1, -1)
    # ax.legend(loc='upper right')
    plt.pause(3)

    t, dist = 0, 0
    lastTime = time.time()

    while any(np.linalg.norm(robot-goal) > 0.5 for goal, robot in zip(goals, robots)):
        now = time.time()
        dt = now - lastTime
        time_text.set_text(f"Tempo = {t:.2f}s")
        
        # Salvar capturas 
        # if round(np.floor(t+0.5), 0)%10 == 0:
        #     fig.savefig(f"subplot t={t:.0f}s.png")

        for i in range(len(robots)):
            if np.linalg.norm(robots[i]-goals[i]) < 0.5:
                continue
            force = att_force(robots[i], goals[i]) 
            for ob in obs:
                force += rep_force(robots[i], ob, 5*ob[2], axis=None)
            for j in range(len(robots)):
                if(i != j): force += rep_force(robots[i], np.append(robots[j], 0.5), R=1, axis=None)
            robots[i] = robots[i] + vel*dt*unit_vector(force)
            # Mostrar trajeto 
            # ax.scatter(robots[i][0], robots[i][1], color = colors[i], s=10)

        robot_plot.set_color(colors)  
        robot_plot.set_sizes(np.full(len(robots), 10))
        robot_plot.set_offsets([[robot[0], robot[1]] for robot in robots])

        plt.pause(0.01)
        plt.draw()

        t = t + dt      
        lastTime = now + 0.01

        if(t > 40): 
            print("Há um mínimo local")
            break

    # plt.savefig(f"subplot t={t:.0f}s.png")
    plt.show(block=True)