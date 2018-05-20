## Rubric Points

* **Student describes their model in detail. This includes the state, actuators and update equations.**

I use a global kinematic model to simulate the dynamics of the vehicle. For a given state and actuator inputs we can predict what the next state vector will be using the following equations:

**x = x + v * cos(psi_n) * dt**

**y = y + v * sin(psi_n) * dt**

**psi = psi + v * delta / Lf * dt**

**v = v + a * dt**

**cte = cte + (v * sin(epsi) * dt)**

**epsi = epis + v * delta / Lf * dt**

In the optimizer the dynamics are enforced via equality constraints by setting the above equations to zero.
The state variables are **x, y, psi,v, cte and epsi**.
The actuator inputs are **a** (the throttle) and **delta** (steering angle).

* **Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.**

Selecting an appropriate N is very important. If we predict too far ahead, our predicted trajectory will be very different then the reference trajectory, due to an approximate model and changing environment. Initially I tried **N** value of 20 seconds and **dt** = 0.2 seconds. This did not work that well and the car did not stick to the track path. I noticed increasing **N** also made the computational load higher. I then tested with **N** = 10 and **dt** = 0.1 seconds. This caused the car to swerve a bit around the track and again not follow the path. I tuned the **dt** parameter a bit and found a **dt** = 0.15 seconds worked well. 

As rule of thumb

A smaller **dt** is good as we can produce a more continuous path. But the tradeoff is predicting a larger number of steps for a given N.

A larger **N** is not always good as even though we can predict further ahead, it affects the computational load on the PC. So we need to be careful with this value as well.


* **A polynomial is fitted to waypoints.
If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.**

The waypoints provided from the simulator are transformed from the map co-ordinate system to the vehicle co-ordinate system using a transformation based on the current position and heading of the vehicle. The equations are:

**vehicle_waypoints_x(i) = (ptsx(i) - px) * cos(psi) + (ptsy(i) - py) * sin(psi)**

**vehicle_waypoints_y(i) = (ptsy(i) - py) * cos(psi) - (ptsx(i) - px) * sin(psi)**

Where ptsx(i) is the current x waypoint in the map, ptsy(i) is the current y waypoint in the map. (px, py) is the position of the vehicle in the map. psi is the heading of the vehicle.

After this a third order polynomial is fitted to the transformed points and the coefficients passed to the MPC solver.

* **The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.**

I dealt with latency by predicting the state **latency_dt** time into the future using my kinematic model equations:


**double dt = 0.1;**

**const double Lf = 2.67;**

**x += v * cos(psi_n) * dt;**

**y += v * sin(psi_n) * dt;**

**psi_n -= v * delta / Lf * dt;**

**v = v + a * dt;**

**cte += (v * sin(epsi) * dt);**

**epsi -= v * delta / Lf * dt;**

I then provided this predicted state to the MPC solver as the current state. This accounted for the time lag between the inputs.

* **No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).
The car can't go over the curb, but, driving on the lines before the curb is ok.**

Here is a [final video](https://drive.google.com/file/d/1KkwDFw7BzsDXkhTneA2GYm25Hav6TluX/view?usp=sharing) of my vehicle driving around the track.
