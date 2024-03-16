# c:\users\bastien\appdata\local\programs\python\python39\python
import sys,os
sys.path.append('C:\Program Files\Webots\lib\controller\python39')
import math
import time;
from controller import *
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from matplotlib import collections  as mc
import random
from scipy.spatial import cKDTree as KDTree

robot = Supervisor()

timestep = int(robot.getBasicTimeStep())

motor_left = robot.getDevice("motor.left");
motor_right = robot.getDevice("motor.right");
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))



##Values## 
#Length 	112 mm
#Width 	117 mm
#Height 	53 mm
#Weight 	0.25 kg
#Max. forward/backward speed 	0.2 m/s
#Max. rotation speed 	9.53 rad/s

#choix du mode
mode=2

#definission du capteur
captf=robot.getDevice('prox.horizontal.2')
captf.enable(timestep)


#initialisations
cpt=0
etat=0

#angle de rotation souhaité
x=180
l_cpt_a=x/90*215


#mode auto
if mode==0 :
    while (robot.step(timestep) != -1): #Appel d'une étape de simulation
        cpt=cpt+1
    
        if etat == 0:
              motor_left.setVelocity(-2)
              motor_right.setVelocity(2)
              if cpt>=l_cpt_a:
                  cpt=0
                  etat=1
        elif etat == 1:
              motor_left.setVelocity(5)
              motor_right.setVelocity(5)
              if cpt>=200:
                  cpt=0
                  etat=0
        else:
              cpt=0
              etat=0

#mode clavier
elif mode==1 :
    keyboard=Keyboard()
    keyboard.enable(timestep)
    while (robot.step(timestep) != -1): #Appel d'une étape de simulation
        key=keyboard.getKey()
        if (key==ord('Z')) :
            motor_left.setVelocity(9)
            motor_right.setVelocity(9)
        elif (key==ord('S')) :
            motor_left.setVelocity(-9)
            motor_right.setVelocity(-9)
        elif (key==ord('Q')) :
            motor_left.setVelocity(-5)
            motor_right.setVelocity(5)
        elif (key==ord('D')) :
            motor_left.setVelocity(5)
            motor_right.setVelocity(-5)
        else :
            motor_left.setVelocity(0)
            motor_right.setVelocity(0)
        
        
if mode==2 :
    keyboard=Keyboard()
    keyboard.enable(timestep)
    while (robot.step(timestep) != -1): #Appel d'une étape de simulation
        key=keyboard.getKey()
        if (key==ord('Z'))&(captf.getValue()>) :
            motor_left.setVelocity(9)
            motor_right.setVelocity(9)
        elif (key==ord('S')) :
            motor_left.setVelocity(-9)
            motor_right.setVelocity(-9)
        elif (key==ord('Q')) :
            motor_left.setVelocity(-5)
            motor_right.setVelocity(5)
        elif (key==ord('D')) :
            motor_left.setVelocity(5)
            motor_right.setVelocity(-5)
        else :
            motor_left.setVelocity(0)
            motor_right.setVelocity(0)


        
        
        
        