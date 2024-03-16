# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This controller gives to its node the following behavior:
Listen the keyboard. According to the pressed key, send a
message through an emitter or handle the position of Robot1.
"""

import math
import numpy as np
import pickle
from sklearn.neural_network import MLPRegressor
from controller import *

class Operation:
    def __init__(self):
        self.speed = 0.0

    def forward(self, speed):
        self.speed = speed 
        if self.speed<2:
            self.speed+=0.2
        motor_right.setVelocity(self.speed)
        motor_left.setVelocity(self.speed)

    def back(self, speed):
        self.speed = speed
        if self.speed>-2:
            self.speed-=0.2
        motor_right.setVelocity(self.speed)
        motor_left.setVelocity(self.speed)

    def rotate_L(self, speed):
        self.speed = speed
        motor_left.setVelocity(0)
        motor_right.setVelocity(self.speed)

    def rotate_R(self, speed):
        self.speed = speed
        motor_left.setVelocity(self.speed)
        motor_right.setVelocity(0)

    def stop(self):
        self.speed = 0
        motor_right.setVelocity(self.speed)
        motor_left.setVelocity(self.speed)

def neuron(x1, x2, logic):

    def activation(x):
        return 1 if x > 0 else 0
        
    if (logic == "OR"):
        weights = [0.5, 0.5, -0.2]
    elif (logic == "AND"):
        weights = [0.5, 0.5, -0.7]

    inputs = np.array([x1, x2, 1])
    weights = np.array(weights)
    weighted_sum = np.dot(inputs, weights)
    output = activation(weighted_sum)

    return output
    
def motor_neuron_left(x1, x2, x3, weightSelect):

    def activation(x): return np.tanh(x)
    """
        if x>1:
            return 1
        elif x<-1:
            return -1
        else:
            return x  
    """
    # x1=pos, x2=-back, x3=-neg, x4=fwd
    if weightSelect == 1:
        weights = [10, -1, -10, 0.5]
    elif weightSelect == 2:
        weights =  [0.8, -1, -0.8, 0.5]
    elif weightSelect == 3:
        weights = [10, -0.5, -10, 1]
    elif weightSelect == 4:
        weights = [1, -5, -1, 5]
    

    inputs = np.array([x1, x2, x3, 1])
    weights = np.array(weights)
    weighted_sum = np.dot(inputs, weights)
    speed = activation(weighted_sum)

    return speed
    
    
def motor_neuron_right(x1, x2, x3, weightSelect):

    def activation(x): return np.tanh(x)
    """
        if x>1:
            return 1
        elif x<-1:
            return -1
        else:
            return x 
    """
    # w_fwd==w_back w_neg==w_pos
    # x1=pos, x2=-back, x3=-neg, x4=fwd
    
    if weightSelect == 1:
        weights = [-10, -1, 10, 0.5]
    elif weightSelect == 2:
        weights = [-0.8, -1, 0.8, 0.5]
    elif weightSelect == 3:
        weights = [-10, -0.5, 10, 1]
    elif weightSelect == 4:
        weights = [-1, -5, 1, 5]

    inputs = np.array([x1, x2, x3, 1])
    weights = np.array(weights)
    weighted_sum = np.dot(inputs, weights)
    speed = activation(weighted_sum)

    return speed
    
    

def speed_control(x1):
    x1_norm = x1/4500 
    speed = x1_norm*9.5
    return speed

def sensor_info():

    distanceSensors = [robot.getDevice("prox.horizontal." + str(i)) for i in range(7)]
    for idx, sensor in enumerate(distanceSensors):
        sensor.enable(timestep)
    prox_values = [sensor.getValue() for sensor in distanceSensors]      
    # print(prox_values)  
    return prox_values
    
    
def switch_mode(key, mode):
    """
    Switch between "avoid" and "follow" modes based on keyboard input.
    """
    if key == ord(' '):
        mode = "follow" if mode == "avoid" else "avoid"
        print(f"switch to {mode} mode")
    return mode
    

def select_weights(key):
    """
    Select weights for motor neurons based on keyboard input {}.
    """
    global weightSelect
    
    if key!=-1:  
        if 49 <= int(key) <= 52 and (int(key)-48)!=weightSelect:
            weightSelect = int(key)-48
            if weightSelect==1:
                print(f"weights 1 activated: left_neuron = [wpos, wback, wneg, wfwd] = [10, -1, -10, 0.5]")
            elif weightSelect==2:
                print(f"weights 2 activated: left_neuron = [wpos, wback, wneg, wfwd] = [0.8, -1, -0.8, 0.5]")
            elif weightSelect==3:
                print(f"weights 3 activated: left_neuron = [wpos, wback, wneg, wfwd] = [10, -0.5, -10, 1]")
            elif weightSelect==4:
                print(f"weights 4 activated: left_neuron = [wpos, wback, wneg, wfwd] = [1, -5, -1, 5]")

if __name__=="__main__":

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)

    print("Initialization")

    led = robot.getDevice('leds.top')
    lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    operation = Operation()

    motor_left = robot.getDevice("motor.left");
    motor_right = robot.getDevice("motor.right");
    motor_left.setPosition(float('inf'))
    motor_right.setPosition(float('inf'))
    motor_left.setVelocity(0)
    motor_right.setVelocity(0)

    robot_speed = 0.0
    distanceVal = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    state = 0
    time= 0
    once = True
    weightSelect = 3
    mode = "avoid"
    
    # warm up
    distanceVal = sensor_info()

    while (robot.step(timestep) != -1):
  
        # Process sensor data here
        # Enter here functions to send actuator commands, like:
        led.set(0x0000ff)
        command = keyboard.getKey()
        mode = switch_mode(command, mode)
        select_weights(command)
        """
        if neuron(int(distanceVal[5]>0), int(distanceVal[6]>0), "OR"):
            operation.forward(2)
        else:
            operation.stop()
        if distanceVal[2]>0:
            operation.back(speed_control(-distanceVal[2]))
        """
        distanceVal = sensor_info()

        distanceVal[0],distanceVal[2],distanceVal[4]
        
        if mode == "follow":
            motor_left.setVelocity(motor_neuron_right(distanceVal[0]/4500, distanceVal[2]/4500, distanceVal[4]/4500, weightSelect)*9)
            motor_right.setVelocity(motor_neuron_left(distanceVal[0]/4500, distanceVal[2]/4500, distanceVal[4]/4500, weightSelect)*9)
        elif mode=="avoid":
            motor_left.setVelocity(motor_neuron_left(distanceVal[0]/4500, distanceVal[2]/4500, distanceVal[4]/4500, weightSelect)*9)
            motor_right.setVelocity(motor_neuron_right(distanceVal[0]/4500, distanceVal[2]/4500, distanceVal[4]/4500, weightSelect)*9)
        
            
            
      

# Enter here exit cleanup code
