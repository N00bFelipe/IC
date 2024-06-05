import numpy as np
import matplotlib.pyplot as plt

def plotFrame(FA, cor):
    origem = FA[:3, 3]

    ax.quiver(*origem, *FA[:3, 0], color=cor)
    ax.quiver(*origem, *FA[:3, 1], color=cor)
    ax.quiver(*origem, *FA[:3, 2], color=cor)

def plotAparaB(FA, FB, cor):
     ax.quiver(*FA[:3,3], *FB[:3,3], color=cor)

def Rz(th):
    return np.array([[np.cos(th), -np.sin(th), 0, 0],
                     [np.sin(th),  np.cos(th), 0, 0],
                     [         0,           0, 1, 0],
                     [         0,           0, 0, 1]])


if __name__ == '__main__':
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    F0 = F1 = np.identity(4)

    DeltaX = 5
    DeltaY = 0
    DeltaZ = 0

    T = np.array([[1, 0, 0, DeltaX],
                  [0, 1, 0, DeltaY],
                  [0, 0, 1, DeltaZ],
                  [0, 0, 0, 1]])

    F1 = T @ Rz(np.deg2rad(30)) @ F0

    plotFrame(F0, 'g')
    plotFrame(F1, 'r')
    plotAparaB(F0, F1, 'b')

    ax.set_xlim([0, 10])
    ax.set_ylim([0, 10])
    ax.set_zlim([0, 10])
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    plt.show()
