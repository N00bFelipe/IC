import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time

WORLDX, WORLDY = 10, 10 

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


if __name__ == '__main__':
    goal = np.array([WORLDX*np.random.rand(), WORLDY*np.random.rand()])
    robot=np.array([WORLDX*np.random.rand(), WORLDY*np.random.rand()])
    vel=1

    XX, YY = np.meshgrid(np.arange(0, WORLDX+.4, .4), np.arange(0, WORLDY+.4, .4))
    XY = np.dstack([XX, YY]).reshape(-1, 2)

    Fatt = att_force(XY, goal)
    Fatt_x = Fatt[:,0]
    Fatt_y = Fatt[:,1]

    # Obstáculos: (x, y, r)
    obs = []
    Frep=0
    for i in range(np.random.randint(5, 16)):
        newobstacle=np.array([WORLDX*np.random.rand(), WORLDY*np.random.rand(), 0.30*np.random.rand()+0.20])
        if(np.linalg.norm(goal-np.array([newobstacle[0], newobstacle[1]])) > newobstacle[2]+ 5*newobstacle[2] and np.linalg.norm(robot-np.array([newobstacle[0], newobstacle[1]])) > newobstacle[2]+1 ):
            obs.append(newobstacle)
            Frep += rep_force(XY, newobstacle)
    
    if(len(obs)):
        Frep_x = Frep[:,0]  # Cuidado com as referências
        Frep_y = Frep[:,1]  # Cuidado com as referências

    Ft = Fatt + Frep

    # Normalização
    # Ft = Ft / np.linalg.norm(Ft, axis=1).reshape((len(Ft), 1))
    Ft_x = np.copy(Ft[:,0])
    Ft_y = np.copy(Ft[:,1])

    # Threshold para visualização
    fmax = .15
    Fm = np.linalg.norm(Ft, axis=1)
    Ft_x[Fm > fmax], Ft_y[Fm > fmax] = 0, 0

    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlim(0, WORLDX)
    ax.set_ylim(0, WORLDY)
    time_text = ax.text(0, -0.1, '', transform=ax.transAxes, ha='left')

    quiver = ax.quiver(XX, YY, Ft_x, Ft_y, color='b', label="Campo Potencial")
    for i in range(len(obs)):
        obs_labels= ax.add_patch(patches.Circle((obs[i][0], obs[i][1]), obs[i][2], color='k'))

    goal_label= ax.plot(goal[0], goal[1], 'og', markersize=10, label="Objetivo")
    rob_label= ax.scatter(robot[0], robot[1], color='red', s=100, label="Robô") 
    print(goal_label)
    print(rob_label)
    ax.legend(loc='upper right')

    t, dist = 0, 0
    lastTime = time.time()

    while np.linalg.norm(robot-goal) > 0.01:
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

        if(t > 25): 
            print("Há um mínimo local")
            break

    plt.show(block=True)
