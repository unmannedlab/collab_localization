# collab_localization
A repository of simulation and experimental work in Decentralized Collaborative Localization

# MatLab Package 
The MatLab package provided in this repository contains a vectorized and parallelized simulation framework for testing collaborative localization algorithms. This framework can handle an indeterminate number of vehicles, sensors, and models. Additionally, Monte-Carlo analysis can be quickly produced

## Experimental Setup

Experiments can easily be setup using a input *.m file such as this. Simply definiting initial conditions for each vehicle and the desired control inputs is all one needs to run the simulations. 

```matlab
nCars = 2;
in_name = 'par';

x0 = [ ...
    2,0,0,2;...
   -2,0,0,2]';

acc_cmd = [ ...
    0,  0,  0;...
    4,  0,  0];

del_cmd = [ ...
    0,      0,      0;...
    4,      0,      0;...
    8,      0,      0;...
    12,     0,      0];
```

## Monte-Carlo Analysis

Using very large numbers of numbers of simulations, it is possible to create error distributions for various localization algorithms. In order to take advantage of a multi-core CPU in addition to simultaneous, vectorized simultions, simply run

```matlab
multithread.m 
```

Otherwise, for single-threaded (but still vectorized) simulations, run
```matlab
main.m
```


# ROS Pacakge

The Included ROS node sets up and runs the collaborative localization algorithm used in experimentation with the UWB sensors and the autonomous trolley. 

## Dependencies
Currently, running the experimental ROS node relies on the following packages 
- [ubxtranslator](https://github.com/unmannedlab/ubxtranslator)
- [decawave_ros](https://github.com/unmannedlab/decawave_ros)
- [vn300](https://github.com/unmannedlab/vn300)


### Launch
Using ROS, the following launch file (ROS/launch) initializes 
```bash
roslaunch collab_localization estimation.launch
```