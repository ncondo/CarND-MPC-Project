# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## Project Description

The goal of this project is to implement Model Predictive Control (MPC) to drive a car around a track in a simulated environment. In an effort to reproduce a real world setting, the model also must account for a 100 millisecond latency between actuation commands.

### Vehicle model

A kinematic model is used with a goal of capturing how the state evolves over time, and how we can provide an input to change it. The state is represented by the x and y coordinates of the vehicle, the vehicle's orientation (psi), the velocity (v), the error between the center of the road and the vehicle aka cross-track error (cte), and the orientation error (epsi). The actuators which control the vehicle state are the steering angle (delta) and acceleration (a). The parameter Lf is the distance from the front of the vehicle to its center of gravity, which is needed because different sized vehicles have different turning radii. Updating the state each timestep is done via the following equations:
```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = cte[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] / Lf * delta[t] * dt
```

### Polynomial Fitting and MPC Preprocessing

The simulator provides waypoints using the map's coordinate system, which I transform into the vehicle's coordinate space. This makes it easier to calculate the cte and epsi values for the MPC, and also for displaying the trajectories. The transformation is done by first shifting the origin to the current position of the vehicle and rotating the x-axis to align with the heading of the vehicle. The following equations are used to perform the transformation:
```
shift_x = waypoint_x - car_x
shift_y = waypoint_y - car_y

waypoint_x = shift_x * cos(-psi) - shift_y * sin(-psi)
waypoint_y = shift_x * sin(-psi) - shift_y * cos(-psi)
```
A third order polynomial is then fit to the waypoints in vehicle space.

### Timestep Length and Elapsed Duration (N & dt)

The prediction horizon is the duration over which future predictions are made, and is the product of two variables: N and dt. N is the number of timesteps in the horizon, and dt is how much time elapses between actuations. N and dt are hyperparameters which need to be tuned for different models. In general, the horizon should only be a couple seconds at most in the case of driving a vehicle. This is because the environment will change quickly enough so that predictions 10s of seconds into the future will be inaccurate. In this project, I found values of N=10 and dt=0.1 to work well, producing a prediction horizon of one second. I initially experimented with a timestep of 20 and durations as low as 0.05, but most of the test runs ended with my car in the lake. Specifically, making the prediction horizon more than one second resulted in very poor performance around sharp bends.

### Model Predictive Control with Latency

One of the requirements of this project was to create a model that could handle a 100 millisecond latency, since there is often some amount of latency in actuators in real world systems. My solution to this problem was to first use the kinematic equations described earlier to predict the state of the vehicle after 100ms. After receiving the data from the simulator, and before making the transformation into vehicle space, I apply the following equations:
```
car_x = car_x + v * cos(psi) * latency
car_y = car_y + v * sin(psi) * latency
psi = psi - v * delta / Lf * latency
v = v + a * latency
```
With the hyperparameters tuned and latency accounted for, my model is able to reach speeds of over 100mph on the test track while safely and smoothly navigating the course. [Link to demo on YouTube](https://www.youtube.com/watch?v=Va-4HuDMhfo)
<p align="center">
  <img width="480" height="270" src="demo/MPC-demo.gif">
</p>
