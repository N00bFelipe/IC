import numpy as np

class Obstacle:

    def __init__(self, position, size, color = (0, 0, 0)):
        self.position = np.array(position)
        self.size = size
        self.color = color

    @classmethod
    def matrix(cls, start, end, rows, colluns, size):
        obstacles = []
        if colluns > 1 and rows > 1:
            dr = np.fabs(end[1] - start[1])/(rows - 1)
            dc = np.fabs(end[0] - start[0])/(colluns -  1)
        elif rows <= 1 and colluns > 1: 
            dr = 0
            dc = np.fabs(end[0] - start[0])/(colluns -  1)
        elif rows > 1 and colluns <= 1: 
            dr = np.fabs(end[1] - start[1])/(rows - 1)
            dc = 0
        else: dr = dc = 0
        for j in range(colluns):
            for i in range(rows):
                if not j & 1:
                    obstacles.append(Obstacle([start[0] + j*dc, start[1] - i*dr], size))
                else:
                    obstacles.append(Obstacle([start[0] + j*dc, start[1] - (i + 1/2)*dr], size))
        return obstacles
