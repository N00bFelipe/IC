import matplotlib.pyplot as plt
import numpy as np

def update(p):
    p = p + v * dt

    return p

if __name__ == '__main__':
    X = np.arange(-10, 10, 1)
    Y = np.arange(-10, 10, 1)
    U, V = np.meshgrid(X, Y)

    plt.ion()
    fig, ax = plt.subplots()
    # q = ax.quiver([2,-2], [2,-2], [[0,1 ],[1,-1]], [[0,-1],[0,1]], color='g')
    # [2,2]->[0,0]
    # [-2,2]->[1,-1]
    # [2,-2]->[1,0]
    # [-2,-2]->[-1,1]
    q = ax.quiver(X, Y, U, V)
    ax.quiverkey(q, X=0.3, Y=1.1, U=10, label='Quiver key, length = 10', labelpos='E')
    ax.set_xticks(np.arange(-10, 10, 1)) 
    ax.set_yticks(np.arange(-10, 10, 1))
    ax.grid(1)
    
    x = y = 0
    p = np.array([x, y])

    vx = vy = 1
    v = np.array([vx, vy])

    dt = 0.1

    for a in range(1,100):
        p = update(p)

        ax.plot(p[0], p[1], 'or')

        plt.pause(.2)
        plt.draw()