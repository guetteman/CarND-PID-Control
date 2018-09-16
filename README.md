# **Self-Driving Car PID Controller**

![alt text][image1]

---


[//]: # (Image References)

[image1]: ./images/main.png "Kidnapped Vehicle"

### This is the fourth project of term 2 of self-driving cars engineer nanodegree.

In this project we will In this project we will revisit the lake race track from the Behavioral Cloning Project. This time, however, we'll implement a PID controller in C++ to maneuver the vehicle around the track!

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## PID Controller

PID or Proportional-Derivative-Integral controller continuosly calculates the error as the difference between the desire setpoint and a measured process variable (PV) and applies a correction based on proportional, integral, and derivative terms (denoted P, I, and D respectively).

In this case, it will help us to maneuver the car across the track.

## Proportional Controller

The proportional portion of the PID controller affect the steering to **Proportionally** correct the car on the lane center, based on `CTE` which tells us the error between the lane center and the car's position. One issue with this controller is that the car move oscillating from right to left, and it goes easily off the track.

## Derivative Controller

The derivative portion helps to better estimate the future trend of `CTE` based on the current rate of change. This means that the car will smoothly get close to lane center and reduce the overshooting that creates the **P** portion.

## Integral Controller

If there is a residual `CTE` error (bias) after the application of proportional control that prevents the car to get to center lane, the integral term seeks to eliminate the residual error by adding a control effect due to the historic cumulative value of the error. In this case, the **I** term helps to correct the car to the center lane on curves. This cannot be solved only with a **PD** controller. [Check Fully Funtional PID](https://youtu.be/_M_V785kh6A)

## Hyperparameters

The hyperparameters were chosen manually. First, I added a **Proportional Controller** with a `Kp = 1.0` and tested. Next, to prevent overshooting I added a **Derivative Controller** with a `Kd = 1.0` this improved the oscillations, but the car went easily off the track because th bias. So, I reduced `Kp = 0.1` and `Kd == 0.01`, and I added a **Integral Controller**  with a `Ki = 2`. After, some tests I finally found that this values worked very well `Kp = 0.17` `Kd = 0.0001` `Ki = 4.0`.