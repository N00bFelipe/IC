import pygame
from pygame.locals import *
from sys import exit
import numpy as np

escala = 150
r = escala*((1/2) * (195/1000))
l = escala*((1/2) * (381/1000))

corpo = np.array([[100   , -190.5],      
                  [227.5 , -50   ],
                  [227.5 , 50    ],
                  [100   , 190.5 ],
                  [-200  , 190.5 ],
                  [-227.5, 163   ],
                  [-227.5, -163  ],
                  [-200  , -190.5]])*(escala/1000)
corpo = np.hstack((corpo, np.ones((corpo.shape[0], 1)))).T

rodaE = np.array([[ 97.5, 170.5],
                  [ 97.5, 210.5],
                  [-97.5, 210.5],
                  [-97.5, 170.5]])*(escala/1000)
rodaE = np.hstack((rodaE, np.ones((rodaE.shape[0], 1)))).T

rodaD = np.array([[ 97.5, -170.5],
                  [ 97.5, -210.5],
                  [-97.5, -210.5],
                  [-97.5, -170.5]])*(escala/1000)
rodaD = np.hstack((rodaD, np.ones((rodaD.shape[0], 1)))).T

def T(deltaX, deltaY):
    return np.array([[1, 0, deltaX],
                     [0, 1, deltaY],
                     [0, 0, 1     ]])

def Rz2D(theta):
    return np.array([[ np.cos(theta), -np.sin(theta), 0 ],
                     [ np.sin(theta),  np.cos(theta), 0 ],
                     [ 0            ,  0            , 1 ]])

def draw(surface, P, color):
    TR = T(P[0], P[1]) @ Rz2D(P[2])
    pontos = (TR @ corpo).T
    pygame.draw.polygon(surface, color, pontos[:,:2])

    pontos = (TR @ rodaD).T
    pygame.draw.polygon(surface, (0, 0, 0), pontos[:,:2])

    pontos = (TR @ rodaE).T
    pygame.draw.polygon(surface, (0, 0, 0), pontos[:,:2])

    pygame.draw.circle(surface, (0, 0, 0), (P[0:2]), 3)

pygame.init()
clock = pygame.time.Clock()
paused = True

HEIGHT = pygame.display.Info().current_h - 60
WIDTH = pygame.display.Info().current_w
FPS = 60
Pdif = Pinc = np.array([WIDTH/2, HEIGHT/2, np.deg2rad(90)])
dt = 1
wr = np.deg2rad(40)
wl = np.deg2rad(50)

screen = pygame.display.set_mode((WIDTH, HEIGHT), RESIZABLE)
pygame.display.set_caption('Simulation')

trajetoDif = [(Pdif[0], Pdif[1])]
trajetoInc = trajetoDif
while True:
    clock.tick(FPS)
    
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            exit()
        elif event.type == KEYDOWN:
            if event.key == K_SPACE:
                paused = not paused 

    if not paused:
        screen.fill((255, 255, 255))

        v = (r/2)*(wl + wr)      
        w = (r/(2*l))*(wl - wr) # (wl - wr) = -1*(wr - wl), isso foi feito dado o eixo y apontando para baixo no display

        # Modelo diferencial
        dP = np.array([v * np.cos(Pdif[2]),
                       v * np.sin(Pdif[2]),
                       w])

        Pdif = Pdif + dP*dt
        
        trajetoDif.append((Pdif[0], Pdif[1]))

        # Modelo incremental
        deltaS = v*dt
        deltath = w*dt

        dP = np.array([deltaS*np.cos(Pinc[2] + deltath/2),
                       deltaS*np.sin(Pinc[2] + deltath/2),
                       deltath])
            
        Pinc = Pinc + dP

        trajetoInc.append((Pinc[0], Pinc[1]))
    else:
        screen.fill((220, 220, 220))
 
    for ponto1, ponto2 in zip(trajetoInc, trajetoDif):
        pygame.draw.circle(screen, (46, 160, 67), ponto1, 2)
        pygame.draw.circle(screen, (38, 132, 252), ponto2, 2)

    draw(screen, Pinc, (46, 160, 67))
    draw(screen, Pdif, (38, 132, 252))

    pygame.display.flip()