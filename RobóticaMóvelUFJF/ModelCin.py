import pygame
from pygame.locals import *
from sys import exit
import numpy as np

escala = 150
r = escala*((1/2) + (195/1000))
l = escala*((1/2) + (381/1000))

corpo = escala* np.array([[100   , -190.5],      
                  [227.5 , -50   ],
                  [227.5 , 50    ],
                  [100   , 190.5 ],
                  [-200  , 190.5 ],
                  [-227.5, 163   ],
                  [-227.5, -163  ],
                  [-200  , -190.5]])/1000
corpo = np.hstack((corpo, np.ones((corpo.shape[0], 1))))

rodaE = escala*np.array([[ 97.5, 170.5],
                  [ 97.5, 210.5],
                  [-97.5, 210.5],
                  [-97.5 , 170.5]])/1000
rodaE = np.hstack((rodaE, np.ones((rodaE.shape[0], 1))))

rodaD = escala*np.array([[ 97.5, -170.5],
                  [ 97.5, -210.5],
                  [-97.5, -210.5],
                  [-97.5 , -170.5]])/1000
rodaD = np.hstack((rodaD, np.ones((rodaD.shape[0], 1))))

def T(deltaX, deltaY):
    return np.array([[1, 0, deltaX],
                     [0, 1, deltaY],
                     [0, 0, 1     ]])

def Rz2D(theta):
    return np.array([[ np.cos(theta), -np.sin(theta), 0 ],
                     [ np.sin(theta),  np.cos(theta), 0 ],
                     [ 0            ,  0            , 1 ]])

def draw(surface, P):
    pontos = []
    for ponto in corpo:
        P_rot = ponto.T @ Rz2D(-P[2])
        pontos.append((P[0] + P_rot[0], P[1] + P_rot[1]))
    pygame.draw.polygon(surface, (38, 132, 252), pontos)

    pontos = []
    for ponto in rodaD:
        P_rot = ponto.T @ Rz2D(-P[2])
        pontos.append((P[0] + P_rot[0], P[1] + P_rot[1]))
    pygame.draw.polygon(surface, (0, 0, 0), pontos)

    pontos = []
    for ponto in rodaE:
        P_rot = ponto.T @ Rz2D(-P[2])
        pontos.append((P[0] + P_rot[0], P[1] + P_rot[1]))
    pygame.draw.polygon(surface, (0, 0, 0), pontos)

    pygame.draw.circle(surface, (0, 0, 0), (P[0:2]), 3)

pygame.init()
clock = pygame.time.Clock()
paused = True

HEIGHT = pygame.display.Info().current_h - 60
WIDTH = pygame.display.Info().current_w
FPS = 60
P = np.array([WIDTH/2, HEIGHT/2, np.deg2rad(0)])
dt = 0.1
wr = np.deg2rad(0)
wl = np.deg2rad(10)

screen = pygame.display.set_mode((WIDTH, HEIGHT), RESIZABLE)
pygame.display.set_caption('Simulation')

trajeto = [(P[0], P[1])]
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

        dP = np.array([v * np.cos(P[2]),
                       v * np.sin(P[2]),
                       w])

        P = P + dP*dt
        
        trajeto.append((P[0], P[1]))
            
    else:
        screen.fill((220, 220, 220))
 
    for ponto in trajeto:
        pygame.draw.circle(screen, (255, 0, 0), ponto, 2)
    draw(screen, P)

    pygame.display.flip()