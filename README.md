# Flocking Algo

In this repository, your goal will be to implement a simple control law to perform a flocking motion within a multi-agent system. You will be using a low-level simulator to observe variations in the behavior of the agents and compute some metrics to decide which model is the best.

------

<img src="./README.assets/截屏2022-12-22 15.46.59.png" alt="截屏2022-12-22 15.46.59" style="zoom:100%;" />

------

<img src="./README.assets/截屏2022-12-22 15.09.28.png" alt="截屏2022-12-22 15.09.28" style="zoom:100%;" />

------

# Table of contents

* [Getting Starded](#getting-started)
* [Controlling the Swarm](#controlling-the-swarm)
* [Performance Analysis](#performance-analysis)
* [Conclusion](#conclusion)
# Getting Started

## Installation
!! Note that the program only works under Linux. (It is possible that UNIX OS is enough and hence MAC OS could work) !!

You first need to clone this repository into a 'directory_of_your_choice'. Then you'll have to install the required libraries by running the install.sh file.
```
$ ./tools/install.sh
```
For testing a specific model, one should compile it, the process of the compilation depends
on which algorithm does the user would like to test. For example, there is already an algorithm called spp_evol in the simulator. With the following command, this algorithm can be compiled:
```
$ make spp_evol
```
After the compilation is ready, one can run the program by typing the following command:
```
$ ./robotflocksim_main
```
If everything works, a visualization window should appear.
If one writes a “-novis” flag after this command, then the software starts without
visualization and some statistical parameters will be saved into a specific directory:
```
$ ./robotflocksim_main -novis
```
## Input files and parameters

There are 5 different types of basic input files, which files can be loaded with specific flags.
This files are responsible for the following features:
1. Parameters of flocking algorithm (default is “parameters/flockingparams.dat”) - some of
these parameters can be changed during visualization. These parameters are the
fundamentals of the functions of any algorithm, e.g. in the case of spring-like force
terms, the spring constant and the equilibrium distance should be in this file.

2. Parameters of the flying robot (default is “parameters/unitparams.dat”) - these
parameters can be changed during visualization. This file contains the value of time
delay, noise levels, parameters of inertial effects, etc.

3. Initialization parameters (default is “parameters/initparams.dat”) - e.g. number of units,
length of simulation, range of “collision zone”, etc.

4. Color setups (default is “config/default_colors.ini”) .

5. Setup file for details of output mode (default is “config/output_defaul.ini”) - details will
be discussed later.

If the user starts the program simply with the command mentioned in the previous section,
the default files listed above will be loaded. But of course one can load other files by using the
following flags:
```
./robotflocksim_main -f flockingparams_1.dat -u unitparams_1.dat -c config/colors.ini -i
initparams_2.dat
```

Instead of creating new files and use them with the appropriate flag, you can directly overwrite the default files. For example if you want a different number of units for your simulation, open the parameters/initparams.dat file, modify the 'NumberOfAgents' parameter and save it. It doesn't require a new compilation as the file is being read whenever you run the program.

There is also a flag for selecting a different obstacle distribution. This is probably be the only flag that you will be using for this project. The obstacles files are located in the obstacles folder and this is how you select it when running the program:
```
$ ./robotflocksim_main -obst obstacles/obst_test.default
```


## Output files
If the program has been started with “-novis” flag, it generates different output files that are set in the configuration file which looks like this:
```
# Inner states and trajectories (false or true?)
SaveTrajectories=false
SaveInnerStates=false
# Order parameters (general) (false, timeline, stat or steadystat?)
SaveDistanceBetweenUnits=timeline
SaveDistanceBetweenNeighbours=timeline
SaveVelocity=steadystat
SaveCorrelation=steadystat
SaveCoM=steadystat
SaveCollisions=steadystat
SaveCollisionRatio=steadystat
SaveAcceleration=false
SaveReceivedPowers=timeline
SaveHullArea=steadystat
SavePressure=timeline
# Order parameters (model-specific) (false, timeline, stat or steadystat?)
SaveModelSpecificStats=stat
```
As you can see, many metrics can be computed and there are different types of calculation:
* timeline : the complete timeline will be saved.
* steadystat : the “steadystat” option is capable of saving a time-average with the exclusion of the first part of the measurement (5 secondes for example as the initialisation can be messy and hence skew the results).
* stat : the time-average of the full timeline will be saved.

You can directly modify this file located in the config directory : 
```
vim config/output_config.ini
```

This file will be correctly set for you but you may want to change some flags to get some different results.

## User interactions during visualization

When the visualization is on, the mouse and the keyboard can be used to act on the simulator:
* F12: Reset situation (positions will be redistributed on the visible territory, velocities will be set to zero)
* F2: Camera is centered in the center of mass of the swarm
* F3: Switching between 2D and 3D visualization
* F5: Saving parameters (both the flocking algo's params and the robot models' params will be saved)
* F7: Displaying communication graph (agents which are closer to each other than the communication range will be connected with an edge)
* z: Switching between the two sets of changeable parameters (flocking algo parameters and unit model parameters)
* 1, 2, 3, 4, ..., a, b, c, ...: Choosing parameter to change (For details, see the menu system on the right)
* Space: Pause
* Mouse scroll: Zoom
* ...

There exist many more functions, feel free to try any button on your keyboard or your mouse, you won't break anything ;)

# Controlling the Swarm

Most of the information for this part have already been shown in the [course](doc/cours.pdf). Also in this part, you will find your first coding assignments. All the instructions are located in the src/utilities/interactions_project.c file.
Whenever you want to compile your code please use the following command :
```
$ make project
```

## Repulsion and Attraction 
Both the attraction and the repulsion are based on the spring model or harmonic oscillator.
Indeed, the swarm can be considered as an elastic network model that can be described as a graph with an agent at each vertex and a spring on the edges.
If you consider a unique equilibrium distance between the agents, both interactions can be concatenated in one formula (see the course). However, it can be interesting to divide the interactions in two parts in order to adapt the gains and the equilibrium distances to create a slack distance between repulsion and attraction. 

### Coding the repulsion 
You should be familiar with the simulator by now. For this first coding part, you will have to implement the repulsion force.
Go to the interactions_project.c file and start with EXERCICE 1. You will be guided through the code. You just have to code where "YOUR CODE HERE" is mentionned. 

### Coding the attraction
Go to the interactions_project.c file and complete EXERCICE 2. It is the same as the repulsion. However the parameters are not the same as we will see later.

## Alignment 
The alignment force is based on a friction-like model. The goal is for a given agent to reach the velocity and heading of its neighbour. It is actually very similar to the repulsion and attraction formulation. Please refer to the course to see the formula.

### Coding the alignment
Go to the interactions_project.c file and complete EXERCICE 3.

## Tuning the parameters during a simulation
During the simulation, you can directly act on the parameters of the control law you designed. You will also have access to other parameters regarding the interactions with the obstacles for example. The parameters window is devided in two parts, use the key "z" to switch between them. The first window concerns the unit parameters and the second one concerns the control law. 

### Assignment 

Goal: Reach a stable flocking motion.

Tune the following parameters to improve the flocking behavior: 
* Slope of Repulsion
* Slope of Attraction 
* Slope of Alignment 

* Size of Neighbourhood
* Equilibrium Distance
* Order of alignment error 

The first 3 parameters are the most important one and should be sufficient to reach the goal. But you can also play with the next 3 to see if you can get an improvement.

Once you are satisfied with your setting, you can press F5 to save your parameters into the flockingparams.dat file. This function overwrites the last version so be careful. You can use the -f flag to use different flocking files. 

If you wish to get back to your saved setting during the simulation (for example if you changed a parameter you weren't supposed to and don't remember the correct value) you can press the key 'p'. 

# Performance Analysis

As we mentionned in the [Output files](#output-files) section, many files are available for performance analysis. 

Once you completed the previous assignment and found a good parameters set don't forget to save it with the 'F5' command. Then you can run the simulation without the visual to get all the statistics : 
```
$ ./robotflocksim_main (-f parameters/flockingparams_new.dat (if you used a different file)) -novis
```
All the results will be saved in the output_default/ folder.

## Assignment

Goal : Write a short report that shows the performances of your model.

You may want to compare your model with the initial one. 
The output configuration file will be already set for you in order to have a baseline between all projects. That is to say that you will only have access to time average metrics. However if you have time you can change the metrics display (from 'stat' to 'timeline') and get the full timeline of the metrics. 

## Going Further

If you have reached this part it could be interesting to try your model in an environment including obstacles. You will see that the performances might be much lower. Try to find another set of parameters that performs better and include it in your report. 
You will find a constrained environment in the obstacles/ folder. To use it, run the following command : 
```
$ ./robotflocksim_main -obst obstacles/obst_test.default (-novis)
```
# Conclusion
I hope this project gave you more insights about the control theory in general and about the flocking more precisely. Feel free to ask any question. You can also contribute to this project if you wish to, all ideas are more than welcome. And don't forget to add a star to this repository if you liked it! 
