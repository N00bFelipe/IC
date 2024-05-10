import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

WORLDX, WORLDY = 50, 50
NumRob = 5
vel=1

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
    while N < np.random.randint(10, 20):
        newobstacle=np.array([WORLDX*np.random.rand(), WORLDY*np.random.rand(), 2*np.random.rand()+0.20])

        if(all(np.linalg.norm(g-newobstacle[0:1]) > newobstacle[2]+ 5*newobstacle[2] for g in goals) and all(np.linalg(r-newobstacle[0:1]) > newobstacle[2]+1 for r in robots)):
            obs.append(newobstacle)
            N+=1

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(0, WORLDX)
    ax.set_ylim(0, WORLDY)
    time_text = ax.text(0, -0.1, '', transform=ax.transAxes, ha='left')

    for ob in obs:
        obs_labels= ax.add_patch(patches.Circle((ob[0], ob[1]), ob[2], color='k'))
    
    for g, r in goals, robots:
        goal_label= ax.plot(goal[0], goal[1], 'og', markersize=10, label="Objetivo")
        rob_label= ax.scatter(robot[0], robot[1], color='red', s=100, label="Robô") 
    print(goal_label)
    print(rob_label)
    ax.legend(loc='upper right')

    t, dist = 0, 0
    lastTime = time.time()

    while np.linalg.norm(robot-goal) > 0.1:
        now = time.time()
        dt = now - lastTime
        time_text.set_text(f"Tempo = {t:.2f}s   Distância = {dist:.2f}uc")
        dist += vel*dt
        
        repulsion=0
        for i in range(len(obs)):
            repulsion += rep_force(robot, obs[i], 5*obs[i][2], axis=None)

        f_direction = (att_force(robot, goal) + repulsion)/np.linalg.norm(att_force(robot, goal) + repulsion)

        robot = robot + vel*dt*f_direction
 
        ax.plot(robot[0], robot[1], 'or', markersize=1)
        plt.pause(.01)
        plt.draw()

        t = t + dt        
        lastTime = now

        if(t > 120): 
            print("Há um mínimo local")
            break

    plt.show(block=True)
