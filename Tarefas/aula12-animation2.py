import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

WORLDX, WORLDY = 60, 30
NumRob = 4
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

        if(all(np.linalg.norm(newgoal - g) > 1 for g in goals) and all(np.linalg.norm(newrobot - r) > 1 for r in robots)):
            goals.append(newgoal)
            robots.append(newrobot)
            N+=1

    # Obstáculos: (x, y, r)
    N=0
    while N < np.random.randint(40, 60):
        newobstacle=np.array([WORLDX*np.random.rand(), WORLDY*np.random.rand(), 3*np.random.rand()+0.50])

        if(all(np.linalg.norm(g - newobstacle[:2]) > newobstacle[2] + 5*newobstacle[2] for g in goals) and all(np.linalg.norm(r - newobstacle[:2]) > newobstacle[2] + 1 for r in robots)):
            obs.append(newobstacle)
            N+=1

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
        ax.scatter(goal[0], goal[1], color=colors[-1], s=100, label=f"Robô {len(colors)}")
        ax.scatter(robot[0], robot[1], color='r', s=100) 
    ax.legend(loc='upper right')

    t, dist = 0, 0
    lastTime = time.time()

    while any(np.linalg.norm(robot-goal) > 0.5 for goal, robot in zip(goals, robots)):
        now = time.time()
        dt = now - lastTime
        time_text.set_text(f"Tempo = {t:.2f}s")
        
        for i in range(len(robots)):
            if np.linalg.norm(robots[i]-goals[i]) < 0.5:
                continue
            force = att_force(robots[i], goals[i]) 
            for ob in obs:
                force += rep_force(robots[i], ob, 5*ob[2], axis=None)
            for j in range(len(robots)):
                if(i != j): force += rep_force(robots[i], np.append(robots[j], 0.5), R=1, axis=None)
            robots[i] = robots[i] + vel*dt*unit_vector(force)
            ax.scatter(robots[i][0], robots[i][1], color = colors[i], s=10)
 
        plt.pause(.01)
        plt.draw()

        t = t + dt        
        lastTime = now

        if(t > 60): 
            print("Há um mínimo local")
            break

    plt.show(block=True)