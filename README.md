# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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

1. Clone/Download this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

```
cd <repo>
mkdir build
cd build
cmake ..
make
./pid
```

Start the Term 2 simulator and the start the Project 4: PID controller. The vehicle should drive within the edges of the road.  

-----------------------------------------
## PID controller
## Description

The following properties were observed during this project which helped me to choose the parameters for the PID.  

1. The propotional component helps to steer the vehcile towards the desired trajectory i.e the propotional portion of the controller steers the vehicle to planned path trying to reduce the cross track error(cte). If the value is too high or too low results in over and understeeing respectively. If the P compopnent value is too high and if the cte is also higher the car might end up driving in a circle or just overshoots beyond the desired path. This produces oscillatory movement alond the trajectory, which could make the passengers/driver uncomfortable like sea-sickness. Meanwhile if the P component is too small, the car might take a long time to reach the desired path, which is too dangerous while driving around the curves.

2. The integral component helps to reinforce the vehcile movement. In order to reduce the steady state error, the integral compoent could be useful. For example, if the vehicle is close the desired trajectory, the sum of the differences could be used to reduce the stady state error. When the vehcile has achived the desired path, the integral path could be set to null. If the _I_ compoenet is too high, the vehicle might result in oscillations, while if the I component is too low, the vehicle reats slowly to correct the state.

3. The Derivative component helps to control the steering based on the proposed change in the steering angle. The _D_ component provides resistance to reduce the CTE. In other words, it pull the vehicle away from the desired trajectory if the rate of approach towards the center line is higher, thus preventing the overshoot of the vehicle. If the _D_ component is too high, the resistance dapens the movement resulting in slower correction rate. If the _D_ compoent is too low, the vehicle would be underdamped resulting in less dampening of the vehicle oscillation.

### Parameters optimization

First the parameters used in lectures were used but the vehicle oversteered off the lanes.   

By trial and error the final values for the parameters were achieved.  

Increasing the _P_ component resulted in overshooting of the vehicle, it started with smaller oscillation and then it widened that the vehicle left the track. Then the _D_ component is adjusted to increase the resistance which brought the oscillations under control. A very smaller _I_ component is introduced to minimize the stady state error.  

The final parameters are as follows,  

```
p: 0.23
I: 0.000003
D:5.3
```


### Potential optimization

In the submissin video it could be observed that at sharp curves the vehcile oscillation suddenly. This could be avoided by choosing different weights for curves and straight roads. Due to Time constraints, I haven't further developed the controller.  

A PID based on the steering angle to controll the throttle could also be implemented to make the vehicle movement smoother.  
