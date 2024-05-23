import numpy as np

class Obstacle:

    def __init__(self, position, size, color = '#000000', rangeRep = .3):
        self.position = np.array(position)
        self.size = size
        self.color = color
        self.rangeRep = rangeRep

    @classmethod
    def obsMatrix(cls, start, end, rows, colluns, size):
        if colluns > 1 and rows > 1:
            dr = np.linalg.norm(end[1] - start[1])/(rows - 1)
            dc = np.linalg.norm(end[0] - start[0])/(colluns -  1)
        elif rows <= 1 and colluns > 1: 
            dr = 0
            dc = np.linalg.norm(end[0] - start[0])/(colluns -  1)
        elif rows > 1 and colluns <= 1: 
            dr = np.linalg.norm(end[1] - start[1])/(rows - 1)
            dc = 0
        else: dr = dc = 0
        obstacles = []
        for j in range(colluns):
            for i in range(rows):
                if j%2 == 0:
                    obstacles.append(Obstacle([start[0] + j*dc, start[1] - i*dr], size))
                else:
                    obstacles.append(Obstacle([start[0] + j*dc, start[1] - (i + 1/2)*dr], size))
        return obstacles
