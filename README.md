# CarND-MPC-Project
Udacity Self-Driving Car Nanodegree - Model Predictive Control (MPC) Project

# Overview

This project implements a [Model Predictive Controller(MPC)](https://en.wikipedia.org/wiki/Model_predictive_control) to control a car in Udacity's simulator. The simulator sends car telemetry information (the data specifications are [here](./DATA.md)) to the MPC using [WebSocket](https://en.wikipedia.org/wiki/WebSocket) and it receives the steering angle and throttle. The MPC uses the [uWebSockets](https://github.com/uNetworking/uWebSockets) WebSocket implementation to handle this communication. Udacity provides a seed project to start from on this project ([here](https://github.com/udacity/CarND-MPC-Project)).

# Prerequisites

The project has the following dependencies (from Udacity's seed project):

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.

For instructions on how to install these components on different operating systems, please, visit [Udacity's seed project](https://github.com/udacity/CarND-MPC-Project). As this particular implementation was done on Mac OS, the rest of this documentation will be focused on Mac OS. I am sorry to be that restrictive.

In order to install the necessary libraries, use the [install-mac.sh](./install-mac.sh).

After that installation, there are two other required libraries:

- [Ipopt](https://projects.coin-or.org/Ipopt): To install this library we need to tap homebrew/science first:

```
> brew tap homebrew/science
```

Then install the package:

```
> brew install ipopt
```

- [CppAD](https://www.coin-or.org/CppAD/): `brew install cppad`

**NOTE** Unfortunately after repeated tries I was unable to install the Ipopt library as it wasn't located in the homebrew/science tap as a formula. I tried several other taps but to no avail. As a last resort I had to use Udacity's provided workspace to compile and execute my program.

# Compiling and executing the project

I created a build directory in the project folder by running the command:

``` mkdir build ```

Then I compiled the project using:

``` cmake .. && make ```

This was the output produced during the compilation process:

```
-- Configuring done-- Generating done
-- Build files have been written to:
-- /home/workspace/CarND-MPC-Project/build
Scanning dependencies of target mpc
[ 33%] Building CXX object CMakeFiles/mpc.dir/src/main.cpp.o
[ 66%] Linking CXX executable mpc
[100%] Built target mpc
```

The compiled project can then be run as:

``` ./mpc ```


Now the MPC controller is running and listening on port 4567 for messages from the simulator. Next step is to open Udacity's simulator and select Project 5: MPC Controller from the options.

Click the "Select" button, and the car starts driving. You will see the debugging information on the PID controller terminal.

# [Rubic](https://review.udacity.com/#!/rubrics/896/view) points

## Compilation

### Your code should compile.

I ran into many problems while installing Ipopt package using the homebrew package manager on Mac. Ultimately I had to use Udacity's simulator to implement the algorithm and get it to compile.

## Implementation

### The Model

The model used is a Kinematic model neglecting the complex interactions between the tires and the road. The model equations are as follow:

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

Where:

- `x, y` : Car's position.
- `psi` : Car's heading direction.
- `v` : Car's velocity.
- `cte` : Cross-track error.
- `epsi` : Orientation error.

Those values are considered the state of the model. In addition to that, `Lf` is the distance between the car of mass and the front wheels (this is provided by Udacity's seed project). The other two values are the model output:

- `a` : Car's acceleration (throttle).
- `delta` : Steering angle.

The objective is to find the acceleration (`a`) and the steering angle(`delta`) in the way it will minimize an objective function that is the combination of different factors:

- Square sum of `cte` and `epsi`.
- Square sum of the difference actuators to penalize a lot of actuator's actions.
- Square sum of the difference between two consecutive actuator values to penalize sharp changes.

All of these factors were taken into account on lines 55-69 in src/MPC.cpp file.

Weight of each of the factors was tuned manually to get the car to drive around the track successfully.

### Timestep Length and Elapsed Duration (N & dt)

The number of points(`N`) and the time interval(`dt`) define the prediction horizon. The number of points impacts the controller performance as well. I tried to keep the horizon around the same time the waypoints were on the simulator. With too many points the controller starts to run slower, and some times it went wild very easily. After trying with `N` from 10 to 20 and `dt` 100 to 500 milliseconds, I decided to leave them fixed to 10 and 100 milliseconds to have a better result tuning the other parameters.

### Polynomial Fitting and MPC Preprocessing

The waypoints provided by the simulator are transformed to the car coordinate system at [./src/main.cpp](./src/main.cpp#L133) from line 133 to line 141. Then a 3rd-degree polynomial is fitted to the transformed waypoints to calculate the coefficients. The state is then initialzed while considering the actuator delay into account in the function `initStateWithDelay` at [./src/main.cpp](./src/main.cpp#L71).
These polynomial coefficients and the state are then fed into the MPC solve method to calculate the steer and the throttle value. The solver output is also used to create a reference trajectory(L179-194).

### Model Predictive Control with Latency

To handle actuator latency, the state values are calculated using the model and the delay interval. These values are used instead of the initial one. The code implementing that could be found in initStateWithDelay function at [./src/main.cpp](./src/main.cpp#L71) from line 71 to line 94.

## Simulation

### The vehicle must successfully drive a lap around the track.

The vehicle successfully drives a lap around the track.