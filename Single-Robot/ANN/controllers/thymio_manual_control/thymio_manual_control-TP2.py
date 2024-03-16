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
import h5py
import random
from sklearn.model_selection import *




        
def activation(x):return np.tanh(x)

def network(x1, x2, velocity_left, velocity_right):

    w11, w12 = 10, 10 
    w22, w23 = -2, -2
    w21, w24 = 1, 1
    # Compute the values at the first layer
    f11 = activation(w11 * x1)
    f12 = activation(w12 * x2)
    
    factor = 1
    # Compute the output values
    y1 = activation(w21 * f11 + w23 * f12 )
    y2 = activation(w22 * f11 + w24 * f12 )
    
    return y1, y2
    
def action_net(x0, x1, x2, x3, x4):

    def filter_layer(x0, x1, x2, x3, x4): return activation(4*x0-4*x1), activation(-2*x0+4*x1-2*x2), activation(-2*x1+4*x2-2*x3), activation(-2*x2+4*x3-2*x4), activation(-4*x3+4*x4)
    def hidden_layer(x0, x1, x2, x3, x4): return activation(sum(np.array([x0, x1])*np.array([3, -5]))), activation(x2*(-2)), activation(sum(np.array([x3, x4])*np.array([-5,3]))) 
    def output_layer(x0, x1, x2): return activation(sum(np.array([x0, x1])*np.array([3, 3]))),activation(sum(np.array([x1, x2])*np.array([3, 3])))
    
    L10, L11, L12, L13, L14 = filter_layer(x0, x1, x2, x3, x4)
    L20, L21, L22 = hidden_layer(L10, L11, L12, L13, L14)  
    return output_layer(L20, L21, L22)
        

factor = 0.2 # poids de récurrent
def motor_neuron_left(x1, x2, x3, v):

    weights = [10, -1, -10, 0.5, factor]
    inputs = np.array([x1, x2, x3, 1, v])
    weights = np.array(weights)
    weighted_sum = np.dot(inputs, weights)
    speed = activation(weighted_sum)

    return speed
    
def motor_neuron_right(x1, x2, x3, v):

    weights = [-10, -1, 10, 0.5, factor]
    inputs = np.array([x1, x2, x3, 1, v])
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
 


def motor_neuron_left(x1, x2, x3, weightSelect):

    def activation(x): return np.tanh(x)
  
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
    

if __name__=="__main__":

    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    keyboard = robot.getKeyboard()
    keyboard.enable(timestep)
    
    lidar = robot.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    print("Initialization")

    led = robot.getDevice('leds.top')
 
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
    
    velocity_left, velocity_right = 0, 0
    
    # warm up
    distanceVal = sensor_info()
    
    list_velocity = []
    list_sensor = []
    
   
    
    GenData = False
    mode = "Lidar" # Cloud or Lidar or both
    load = False
    
    if not GenData:
        if load:
            print(f"Loading {mode} Model ...")
            with open(f"ai_controller_model_hyper_prox_{mode}.model", 'rb') as f:
                Webot = pickle.load(f)
        else:
            if mode=="Cloud":
                print("Training Mode for Cloud Points ...")
                with h5py.File("dataset_Cloud_webots.hdf5", "r") as f:

                    sensor_data = np.nan_to_num(f['thymio_sensor'][:], nan=np.nan, posinf=np.finfo(float).max)
                    velocity_data = f['thymio_velocity'][:]/9
                Webot = MLPRegressor(hidden_layer_sizes=(360, 300, 200, 50 ,2), activation='tanh', solver='adam', max_iter=1000, verbose=False, random_state=4, momentum=0.9)

        
            elif mode=="Lidar":
                print("Training Mode for Lidar ...")
                with h5py.File("dataset_Lidar_webots.hdf5", "r") as f:

                    sensor_data = f['thymio_sensor'][:]/4500
                    velocity_data = f['thymio_velocity'][:]/9
                Webot = MLPRegressor(hidden_layer_sizes=(7, 500, 100, 2), activation='tanh', solver='adam', max_iter=1000, verbose=False)


            prox_x_train, prox_x_test, prox_y_train, prox_y_test = train_test_split( sensor_data, velocity_data)

            np.random.seed(1)
            print(f"START Training......")
            Webot.fit(prox_x_train, prox_y_train)
            score = Webot.score(prox_x_train, prox_y_train)
            print("Train Score: ", score)
            score = Webot.score(prox_x_test, prox_y_test)
            print("Test Score: ", score)

    else:
        print("Data Collection Mode ...")
    
    
    
    
    while (robot.step(timestep) != -1):
  
        # Process sensor data here
        # Enter here functions to send actuator commands, like:
        led.set(0x0000ff)
      
        distanceVal = sensor_info()
        point_cloud = lidar.getRangeImage()

        # point_cloud = np.nan_to_num(lidar.getRangeImage(), nan=np.nan, posinf=np.finfo(float).max)

        distanceVal[0],distanceVal[2],distanceVal[4]
        
        if not GenData and mode=="Lidar":        
            [[left, right],] = Webot.predict([[distanceVal[0]/4900, distanceVal[1]/4900, distanceVal[2]/4900, distanceVal[3]/4900, distanceVal[4]/4900, distanceVal[5]/4900, distanceVal[6]/4900],])
            motor_left.setVelocity(min(9.53,left*9))
            motor_right.setVelocity(min(9.53,right*9))
        if not GenData and mode=="Cloud":    
            point_cloud = np.nan_to_num(lidar.getRangeImage(), nan=np.nan, posinf=np.finfo(float).max)    
            [[left, right],] = Webot.predict([point_cloud,])
            motor_left.setVelocity(min(9.53,left*9))
            motor_right.setVelocity(min(9.53,right*9))
       
        """
        # Multicouches
        velocity_left, velocity_right = network(distanceVal[1]/4500, distanceVal[3]/4500, velocity_left, velocity_right)
        motor_left.setVelocity(velocity_left*9.5)
        motor_right.setVelocity(velocity_right*9.5)
        
        # récurrent
        velocity_left = motor_neuron_left(distanceVal[0]/4500, distanceVal[2]/4500, distanceVal[4]/4500, velocity_left)
        velocity_right = motor_neuron_right(distanceVal[0]/4500, distanceVal[2]/4500, distanceVal[4]/4500, velocity_right)
        motor_left.setVelocity(velocity_left*9.5)
        motor_right.setVelocity(velocity_right*9.5)
        
        # Filtre spatial
        velocity_left, velocity_right = action_net(distanceVal[0]/4500, distanceVal[1]/4500,  distanceVal[2]/4500,  distanceVal[3]/4500,  distanceVal[4]/4500)
        motor_left.setVelocity(velocity_left*9.5)
        motor_right.setVelocity(velocity_right*9.5)
        """
        
        if GenData and mode=="Lidar":
            weightSelect = 1
            motor_left.setVelocity(motor_neuron_left(distanceVal[0]/4500, distanceVal[2]/4500, distanceVal[4]/4500, weightSelect)*9)
            motor_right.setVelocity(motor_neuron_right(distanceVal[0]/4500, distanceVal[2]/4500, distanceVal[4]/4500, weightSelect)*9)
            if float(motor_left.getVelocity())== float(motor_right.getVelocity()) :
                if random.random()<0:
                    continue
            list_sensor.append([distanceVal[0], distanceVal[1], distanceVal[2], distanceVal[3], distanceVal[4], distanceVal[5], distanceVal[6]])
            list_velocity.append([motor_left.getVelocity(), motor_right.getVelocity()])
        elif GenData and mode=="Cloud":
            weightSelect = 1
            motor_left.setVelocity(motor_neuron_left(distanceVal[0]/4500, distanceVal[2]/4500, distanceVal[4]/4500, weightSelect)*9)
            motor_right.setVelocity(motor_neuron_right(distanceVal[0]/4500, distanceVal[2]/4500, distanceVal[4]/4500, weightSelect)*9)
            if float(motor_left.getVelocity())== float(motor_right.getVelocity()) :
                if random.random()<0:
                    continue
            list_sensor.append(point_cloud)
            list_velocity.append([motor_left.getVelocity(), motor_right.getVelocity()])
        
        
        command = keyboard.getKey()
        #print(command)

          
        if command==ord('S'):
            print('stop')
            robot_speed = 0
            break

        
    
    if GenData:
        with h5py.File(f"dataset_{mode}_webots.hdf5", "w") as f:
            array1 = np.array(list_sensor)
            array2 = np.array(list_velocity)
       
            # Create the main dataset (can be empty or contain additional data)
            scans_dataset = f.create_dataset('thymio_sensor', data=array1)
            commands_dataset = f.create_dataset('thymio_velocity', data=array2)
        
        # Close the HDF5 file
        f.close()
    else:
        filename = f"ai_controller_model_hyper_prox_{mode}.model"
        pickle.dump(Webot, open(filename, 'wb'))

    

# Enter here exit cleanup code
