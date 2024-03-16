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


## Reference
## https://www.cyberbotics.com/doc/guide/thymio2?version=develop

import math

from controller import *

robot = Robot()
timestep = int(robot.getBasicTimeStep())
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

print(chr(27) + "[2J") # ANSI code for clearing command line
print("Initialization of thymio_variables controller")

lidar = robot.getDevice('lidar')
lidar.enable(timestep)
lidar.enablePointCloud()

distanceSensors = []
for i in list(range(0,7)):
  distanceSensors.append(robot.getDevice('prox.horizontal.'+str(i)))
  distanceSensors[i].enable(timestep)

motor_left = robot.getDevice("motor.left");
motor_right = robot.getDevice("motor.right");
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

robot_speed = 0.0
distanceVal = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
state = 0
time= 0
once = True

# Touch sensors
# button.[forward-backward-left-right-center]
# the first measurement will be available only after the first sampling period has elapsed.
button_fwd = robot.getDevice("button.forward")
button_fwd.enable(timestep)

# LEDs 
led_top = robot.getDevice("leds.top")
led_bottomr = robot.getDevice("leds.bottom.right")
led_bottoml = robot.getDevice("leds.bottom.left")
led_buttons0= robot.getDevice("leds.buttons.led2")


print("Sampling period : ",timestep,"ms")

while (robot.step(timestep) != -1):
  points = []
  point_cloud = lidar.getRangeImage()
  angle=0.0
  
  if(button_fwd.getValue()):
      print("Forward button was pressed")
  led_top.set(0xffffff)
  led_bottomr.set(0xffffff)
  led_bottoml.set(0xffffff)
  led_buttons0.set(0xffffff)
  #leds.top, leds.bottom.[right-left]
  #leds.buttons.led[0-3], 
  #leds.circle.led[0-7], 
  #leds.prox.h.led[0-7], 
  #leds.prox.v.led[0-1], 
  #leds.sound, 
  #leds.rc, 
  #leds.temperature.[red-blue]

  # Read the LiDAR sensor, like:
  for i in point_cloud:
    time +=1
    points.append([i*math.sin(angle),i*math.cos(angle),0.0])
    angle+= 2*math.pi / lidar.getHorizontalResolution()

  # Read the proximity sensors, like:
  for i in list(range(0,7)):
    distanceVal[i] = distanceSensors[i].getValue()

  #Set motors speed :
  motor_left.setVelocity(robot_speed)
  motor_right.setVelocity(robot_speed)

  # Process sensor data here

  # Enter here functions to send actuator commands, like:
  command = keyboard.getKey()
  #print(command)

  if command==keyboard.LEFT:
    #print('Left')
    motor_left.setVelocity(0.0) #-robot_speed
    motor_right.setVelocity(robot_speed)
  elif command==keyboard.RIGHT:
    #print('right')
    motor_left.setVelocity(robot_speed)
    motor_right.setVelocity(0.0) #-robot_speed
  elif command==keyboard.UP:
    print('up')
    if robot_speed<2:
      robot_speed+=0.2
  elif command==keyboard.DOWN:
    print('down')
    if robot_speed>-2:
      robot_speed-=0.2
  elif command==83: # capture S key
    print('stop')
    robot_speed = 0

# Enter here exit cleanup code
