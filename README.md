## PID-Controller-Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Project - 8: PID Controller Project.
---

In this project we will implement a PID Controller in C++ to maneuver the vehicle around the lake race track.

Overview
---
In this project we try to be in lane center by generating vehicle's steering angle, and drive around the race track, using PID Controller

Goal
---
The car will try to be in lane. No tire will leave the drivable portion of the track. The car will not pop up onto ledges or roll over any surfaces that would otherwise be considered


## Output

![Output_gif](./Results/result.gif)

## Implementation Details
Refer to [this](./Model_Documentation-PID_Controller_Project.pdf) for detailed Implementation strategy.

#### Simulator Details

Download Simulator from [here](https://github.com/udacity/self-driving-car-sim/releases).

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```
The simulator will provide you the cross track error (CTE) and the velocity (mph) in order to compute the appropriate steering angle.

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
