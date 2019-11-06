# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Introduction
This project implements a PID controller in order to steer a car around a track using the Udacity Simulator.  The simulator passes cross-track error (CTE), speed and angle to the PID controller.  The PID controller then calculates the steering angle and throttle and passes the information back to the simulator.

An important component to the project is tuning the PID hyper parameters to maximize accuracy.  While there is no speed requirement, we were encouraged to make the car go as fast as possible while also driving safely (keep the car on the track).

## Reflection
*Describe the effect each of the P, I, D components had in your implementation.*

The P (proportional) component has the most effect on the car's driving behavior.  It makes the car steer in proportion to the car's distance from the center lane.  If the car is far to the left or right of the center lane then it will steer "hard" to try and get back to center.  If the car is close, but not centered then it will steer "soft" to get back to center.

The I (integral) component counteracts bias in the CTE which prevents the controller from reaching the center line.  The bias can take several forms, including steering "drift" caused by the car being out of alignment.

The D (differential) component counteracts the P components tendency to overshoot the center line.  A properly tuned D parameter will cause the car to approach the center line without overshooting.  This also results in less "jerky" movement to the left and right.

I tuned the hyper parameters manually.  While I played with the idea of implementing twiddle for tuning the parameters, I took the advice of the previously recorded Project Q&A and instead modified the application so that I could pass in different hyperparameters without having to compile the program each time.

I basically went through a lot of trial and error to get the car to get close to moving around the track.  While I did test changing of the I parameter, the controller seemed to work best with a value of 0.0.  Once I was making it around the track without crashing I left the D parameter constant and tested various values of P component until getting the best results.  I then did similar testing where I left P component constant and varying the value of the K component.

Once I was satisfied with hyperparameter tuning I turned my attention to speed considerations.  Note that when testing changes to hyperparameters I overrode the value passed in from simulator and set the value to 0.4.  While I am sure there are more accurate approaches to tuning the throttle, I kept it simple and focused on the steering angle.  Depending on the absolute value of the calculated steering angle, I increased the value anywhere from 0.7 to 0.9.  This allowed me to reach a top speed of 63+ mph.

## Original project markup below this line
------------------------------------------------------------------------------------------------


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

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

