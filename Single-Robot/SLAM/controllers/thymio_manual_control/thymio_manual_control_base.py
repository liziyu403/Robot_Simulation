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

def smooth(difference): return math.exp(-abs(difference)*0.07)+0.1
        

class Operation:
    def __init__(self):
        self.speed = 5

    def forward(self, speed_factor=1.0):
        self.speed = 5 * speed_factor
        motor_left.setVelocity(self.speed)
        motor_right.setVelocity(self.speed)
    
    def back(self, speed_factor=1.0):
        self.speed = 5 * speed_factor
        motor_left.setVelocity(-self.speed)
        motor_right.setVelocity(-self.speed)

    def rotate_L(self, speed_factor=1.0):
        self.speed = 5 * speed_factor
        motor_left.setVelocity(-self.speed)
        motor_right.setVelocity(self.speed)

    def rotate_R(self, speed_factor=1.0):
        self.speed = 5 * speed_factor
        motor_left.setVelocity(self.speed)
        motor_right.setVelocity(-self.speed)

    def stop(self):
        self.speed = 0
        motor_left.setVelocity(self.speed)
        motor_right.setVelocity(self.speed)
    
def Robot_R(robot, operation, degree):
    rotation = degree/90*86
    counter = 0
    while (robot.step(timestep) != -1):
        if(counter>rotation):
            operation.stop()
            break
        counter += 1
        operation.rotate_R()
        
def Robot_L(robot, operation, degree):
    rotation = degree/90*86
    counter = 0
    while (robot.step(timestep) != -1):
        if(counter>rotation):
            operation.stop()
            break
        counter += 1
        operation.rotate_L()

def Robot_B(robot, operation, square):
    step = square*240
    counter = 0
    while (robot.step(timestep) != -1):
        if(counter>step):
            operation.stop()
            break
        counter += 1
        operation.back()
        
def Robot_F(robot, operation, square):
    step = square*240
    counter = 0
    while (robot.step(timestep) != -1):
        if(counter>step):
            operation.stop()
            break
        counter += 1
        operation.forward()
        
def keyBoard_Controller(keyboard, robot, operation):
    while (robot.step(timestep) != -1): #Appel d'une étape de simulation
        key=keyboard.getKey()
        if (key==ord('Z')) :
            Robot_F(robot, operation, 0.1)
        elif (key==ord('S')) :
            Robot_B(robot, operation, 0.1)
        elif (key==ord('Q')) :
            Robot_L(robot, operation, 5)
        elif (key==ord('D')) :
            Robot_R(robot, operation, 5)
        elif (key==ord('O')) :
            break
        else:
            operation.stop()
           
def avoid_obstacle(prox_values, operation):
    # Threshold for obstacle detection
    obstacle_threshold = 100
    
    # Indices of the sensors
    left_sensors = [0, 1]
    right_sensors = [3, 4]

    # Readings from the left and right sensors
    prox_values[0] *= 5
    prox_values[4] *= 5
    left_readings = [prox_values[i] for i in left_sensors]
    right_readings = [prox_values[i] for i in right_sensors]

    sens_factor = 0.2
    difference = sum(left_readings) - sum(right_readings)
    # If left sensors are closer than right sensors
    if difference > 0 and abs(difference) > obstacle_threshold:
        # Proportional control for smooth turning
        speed_factor = min(1.0, 1)
        operation.rotate_R(speed_factor)  # Turn right
    elif difference < 0 and abs(difference) > obstacle_threshold:
        # Proportional control for smooth turning
        speed_factor = min(1.0, 1)
        operation.rotate_L(speed_factor)  # Turn left
    else:
        print(smooth(difference))
        operation.forward(smooth(difference))  # Move forward


         
if __name__=="__main__":
    robot = Supervisor()

    timestep = int(robot.getBasicTimeStep())

    motor_left = robot.getDevice("motor.left");
    motor_right = robot.getDevice("motor.right");
    motor_left.setPosition(float('inf'))
    motor_right.setPosition(float('inf'))
    operation = Operation()
    
    # Passive-control by plan
    print("Pre-defined controller activated [####################] 100%")
    rotation_degree = 1440
    N_square = 3
    #Robot_L(robot, operation, rotation_degree)
    #print(f"  - operation: Left")
    #Robot_B(robot, operation, N_square)
    #print(f"  - operation: Back")
    #Robot_R(robot, operation, rotation_degree)
    #print(f"  - operation: Right")
    #Robot_F(robot, operation, N_square)
    #print(f"  - operation: Forward")
    print(">> Exit plan control")
    Robot_F(robot, operation, N_square)
   
    """
    # Passive-control by Keyboard
    print("keyboard controller activated [####################] 100%")
    keyboard=Keyboard()
    keyboard.enable(timestep)
    keyBoard_Controller(keyboard, robot, operation)
    print(">> Exit keyboard control")
    """
    """
    # Self-Control by sensor
    print("Auto controller activated [####################] 100%")
    prox_sensors = [robot.getDevice("prox.horizontal." + str(i)) for i in range(7)]
    # print(f'Sensors: {prox_sensors}')
    
    for idx, sensor in enumerate(prox_sensors):
        sensor.enable(timestep)

    while robot.step(timestep) != -1:
        # Read proximity sensor values
        prox_values = [sensor.getValue() for sensor in prox_sensors]
        # print(f'Sensor values: {sensor_values}')

        # Obstacle avoidance logic
        avoid_obstacle(prox_values, operation)

    operation.stop()
    print(">> Exit obstacle avoidance control")
    """
        
        
         
        
        
    

   
    
    


        
        
        
        