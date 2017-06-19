# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Summary

1. Codes compile without errors with `cmake` and `make`.
2. The vehicle successfully drive a lap around the track without leaving the driving portion of track surface.
3. The maximum vehicle speed is about 80 mph.

## Procedures

1. Coordination system transformation.
2. Fit the polynomial to the waypoints.
3. Calculate initial cross track error and orientation error values.
4. Define the components of the cost function (state, actuators, etc).
5. Define the model constraints. These are the state update equations defined in the Vehicle Models module.
6. Optimize cost function weights and verify in simulator.

## Coordination transformation

There are three coordination system in this projects:
1. map coordination system - **the global coordination**  
2. vehicle coordination system - **psi**
3. vehicle coordination system in navigation model - **psi_untiy**

At first it is necessary to translate and rotate from the global coordination to psi coordination. There are two steps one by one and shown in below pictures. The fist step is to translate (-px,-py) then vechile will be at psi coordination origin position. The second step is to rotate psi then vehicle will be traveling down a straight road and the longitudinal direction is the same as the x-axis. In this case, cross crack error and orientation error equations in the class can be used here directly.

Details translating and rotating methods can be found [Translating and rotaing equaitons](http://www.cs.brandeis.edu/~cs155/Lecture_06.pdf ).

![](/home/xufq/proj/CarND-MPC-Project/images/cs_translating_rotating.jpg)

After translating and rotating to psi coordination, there is still difference between psi and psi_unity coordination systems. As [DATA](https://github.com/udacity/CarND-MPC-Project/blob/master/DATA.md) shows, except a rotating 90 degree, there is a reverse between psi and psi_unity coordination systems. Since steering angle here is only variable value, it is only necessary to consider reverse to transform between psi_unity and psi for finally steering value parameter determination.

`double steer_value = -vars[0]/deg2rad(25);`

psi and psi_unity representations

`psi`
```
//            90
//
//  180                   0/360
//
//            270
psi_unity

//            0/360
//
//  270                   90
//
//            180
```

## Latency

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds. This is a problem called "latency" and should be considered in mpc controller codes.

In this projects, latency has been implemented after transforming from the global coordination to psi coordiantion systems. At that moment, the vehicle is moving along x-axis.

```
// consider 100ms latency.
const double latency = 0.1;
const double Lf = 2.67;
double px_car = v*latency;
double py_car = 0;
double psi_car = -v*delta*latency/Lf;
v = v + a * latency;
```

## Weights to optimize MPC controller behaviors

Similar to mpc-quizzes projects, fitting the polynomial to the waypoints.

`auto coeffs = polyfit(ptsx_car, ptsy_car, 3);`

Calculating initial cross track error and orientation error values.

```
// The cross track error is calculated by evaluating at polynomial at x, f(x) and subtracting y.
// f(x) = c0 + c1*x + c2*x^2 + c3*x^3.
double cte = polyeval(coeffs, px_car) - py_car;
// the orientation error is psi -f'(x), f'(x) is derivative of f(x).
// f'(x) = c1 + 2*c2*x + 3*c3*x^2.
double epsi = psi_car - atan(coeffs[1] + 2*px_car*coeffs[2] + 3*px_car*px_car*coeffs[3]);
```

The cost functions has been defined as follow.

```
// The part of the cost based on the reference state.
for (int i = 0; i < N; i++) {
  fg[0] += factor_cte * CppAD::pow(vars[cte_start + i] - ref_cte, 2);
  fg[0] += factor_epsi * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2);
  fg[0] += factor_v * CppAD::pow(vars[v_start + i] - ref_v, 2);
}

// Minimize the use of actuators.
for (int i = 0; i < N - 1; i++) {
  fg[0] += factor_steering * CppAD::pow(vars[delta_start + i], 2);
  fg[0] += factor_throttle * CppAD::pow(vars[a_start + i], 2);
}

// Minimize the value gap between sequential actuations.
for (int i = 0; i < N - 2; i++) {
  fg[0] += factor_diff_steering * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
  fg[0] += factor_diff_throttle * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
}

fg[0] *= 1/(factor_cte + factor_epsi + factor_v + factor_steering + factor_throttle + factor_diff_steering + factor_diff_throttle);
```
Most of time has been spent on optimizing cost functions weights and then to confirm in simulator. Finally there are 2 sets of weights in codes.
1. Cost functions weights A

For weights A, the maximum vehicle speed is about 80 mph, the vehicle speed will drop down automatically when turning a sharp corner.

The disadvantages for weights A is after the first loop, the vehicle will vibrate heavily and unstable.

```
// Factors for the cost computation
// 1, 1, 1, 50, 100, 500, 300, 90
// 1000, 1000, .1, 10000, 1, 50000, 10000, 50
const double factor_cte   = 1000;  
const double factor_epsi  = 1000;
const double factor_v     = .1;
const double factor_steering = 10000;
const double factor_throttle = 1;  
const double factor_diff_steering = 50000;
const double factor_diff_throttle = 10000;
```
while `double ref_v = 50;`

2. Cost functions weights B

For weights B, the maximum vehicle speed is about 60 mph but stable after the first loop. the vehicle speed does not obviously drop down automatically when turning a sharp corner.

```
const double factor_cte   = 1;  
const double factor_epsi  = 1;
const double factor_v     = 1;
const double factor_steering = 50;
const double factor_throttle = 100;  
const double factor_diff_steering = 500;
const double factor_diff_throttle = 300;
```
while `double ref_v = 90;`

## Display the waypoints and MPC predicted trajectory

The transformed `ptsx_car` and `ptsy_car` have been feed into `next_x_vals` and `next_y_vals` and to show in yellow line as vehicle waypoints/reference line.

MPC codes generate mpc class data `mpc.mpc_x` and `mpc.mpc_y` and applied to `mpc_x_vals` and `mpc_y_vals` to display the mpc predicted trajectory.

```
//Display the MPC predicted trajectory
vector<double> mpc_x_vals = mpc.mpc_x;
vector<double> mpc_y_vals = mpc.mpc_y;

//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
// the points in the simulator are connected by a Green line

msgJson["mpc_x"] = mpc_x_vals;
msgJson["mpc_y"] = mpc_y_vals;

//Display the waypoints/reference line
vector<double> next_x_vals;
vector<double> next_y_vals;

for (int i=0;i<n_ptsx;i++){
  next_x_vals.push_back(ptsx_car(i));
  next_y_vals.push_back(ptsy_car(i));
}

//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
// the points in the simulator are connected by a Yellow line

msgJson["next_x"] = next_x_vals;
msgJson["next_y"] = next_y_vals;
```

## Unit transformation

The vehicle speed mph should be transferred into m/s since mph is not ISO unit for calculation. The realted transformation link can be found [mph to m/s](https://www.unitjuggler.com/convert-speed-from-mph-to-ms.html).

```
// mph -> m/s
// https://www.unitjuggler.com/convert-speed-from-mph-to-ms.html
v = v*0.44704;
```

Of course it seems unit transformation is not obligatory since it can be implemented in cost functions weights optimization.  

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
