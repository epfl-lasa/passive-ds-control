# Passive DS Control
Catkin package implementing various DS controllers, including controller proposed in
```
K. Kronander and A. Billard; Passive Interaction Control with Dynamical Systems;
IEEE Robotics and Automation Letters; 2016

```

[![Build Status](https://magnum.travis-ci.com/epfl-lasa/passive-ds-control.svg?token=BqUQb763tsVV4QyzLgBy&branch=master)](https://magnum.travis-ci.com/epfl-lasa/passive-ds-control)

## Minimal example

```c++

PassiveDSController * my_passive_ds;
int D = 3 // dimensionality
float eigval_0,eigval_1,max_tank_level, dz;
eigval_0 = 50;
eigval_1 = 100;
max_tank_level = 10;
dz = 0.01
my_passive_ds = new PassiveDSController(D,eigval_0, eigval_1, max_tank_level, dz);

```
The parameters `eigval_0` and `eigval_1` are the feedback gains along the desired direction of motion and its orthogonal complement respectively. The `max_tank_level` is the maximum energy level that is allowed to be produced when using non-conservative DS as task-model. The smoothness for cutting of non-conservatve control effort when approaching non-passive behavior is controller with `dz`. 

```c++
//my_passive_ds->set_damping_eigval(0,10);
Vec desired_velocity(D), actual_velocity(D), actual_pos(D);
// get the position and velocity from the robot sensors
// these lines obviously depend on your infrastructure and robot interface.
// These are placeholder function calls
current_pos = get_position();
actual_velocity = get_velocity();

// get your desired velocity from your task-level DS
desired_velocity = your_awesome_ds(current_pos) // placeholder

Vec control_force(D);
// update ensuring passivity
control_force = my_passive_ds->UpdatePassive(actual_velocity, desired_velocity, dt);

// update ignoring the passivity 
control_force = my_passive_ds->Update(actual_velocity, desired_velocity);

// should then apply these control forces to your robot, for example using Jacobian transpose to get joint torques.
// you may also want to combine it with some orientation control, or include orientation control in your dynamical system.

// you can reset the storage level to the max level if needed
my_passive_ds->reset_storage();


      
      
```
## Documentation
You can get some basic source code documentation by running doxygen.

```
sudo apt-get install doxygen
roscd passive_ds_control
doxygen Doxyfile
```
