# MPC Project
In this project, I used C++ to write a program that uses Model Predictive Control to drive a car around a simulated track. 

## Project Info
For a great introduction to the topic, check out [the wiki page](https://en.wikipedia.org/wiki/Model_predictive_control).

To see the implementation please visit the following file in the 'src' folder:

1. MPC.cpp
2. main.cpp

## Setup instructions
FYI, I ran my code on a Macbook Pro. Please ensure you have downloaded the Udacity SDCND Simulator [here](https://github.com/udacity/self-driving-car-sim/releases/) and have installed cmake (3.5), make (4.1), and gcc/gcc+ (5.4).

1. Open Terminal
2. `git clone https://github.com/tlapinsk/CarND-MPC-Project.git`
3. `cd CarND-MPC-Project`
4. `sh install-mac.sh`
5. `mkdir build && cd build`
6. `cmake`
7. `make`
8. `./mpc`
9. Run the term2_sim application, select Project 5, and click 'Start'

## MPC Explained
Models in MPC are used to represent the behavior of a complex dynamic system. It will take into account current measurements, or the current dynamic state, and process variable targets/limits to calculate future changes in dependent variables. When the changes are calculated, it will send out the change to each variable and repeat the calculation, waiting for the next change.

You can also refer to future changes as the prediction horizon, or the duration over which future predictions are made. We call this T.

T is the product of N and dt. N being the number of timesteps in the horizon and dt being how much time elapses between actuations.

If N = 20 and dt = 0.5, then T = 10 seconds.

Within the model predictive controller, we tune these hyperparameters. T should be as large as possible, while dt should be as small as possible.

## Model
I utilized the global kinematic model that was discussed in Lesson 18: Vehicle Models.

```
  // The equations for the model:
  // x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
  // y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
  // psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
  // v[t+1] = v[t] + a[t] * dt
  // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
  // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

Variables explained below:
- Lf = physical characteristics of vehicle
- `x[t]` is x coordinate of vehicle at time t
- `y[t]` is y coordinate of vehicle at time t
- `psi[t]` is direction of vehicle at time t
- `epsi` is the orientation error
- `cte` is cross-track error
- `psides` is desired orientation of vehicle
- `f(x)` is ground truth position of vehicle

You want to minimize the objective function by using the following code:

```
  fg[0] = 0;

  for (int i = 0; i < N; i++) {
    fg[0] += 3000*CppAD::pow(vars[cte_start + i], 2);
    fg[0] += 3000*CppAD::pow(vars[epsi_start + i], 2);
    fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
  }

  // State penalty
  for (int i = 0; i < N - 1; i++) {
    fg[0] += 5*CppAD::pow(vars[delta_start + i], 2);
    fg[0] += 5*CppAD::pow(vars[a_start + i], 2);
    fg[0] += 700*CppAD::pow(vars[delta_start+i] * vars[v_start+i],2);
  }

  // Control input penalty
  for (int i = 0; i < N - 2; i++) {
    fg[0] += 1200*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
    fg[0] += 10*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
  }
```

I manually tuned hyperparameters, which is discussed in the next section.

## Hyperparameter Tuning
To tune N and dt, I followed the guidelines from the lesson.

I tested the following for N and dt.
- `N = [5, 10, 15, 20, 25]`
- `dt = [0.05, 0.1, 0.15, 0.2, 0.25]`

I landed on the following after testing:
```
  size_t N  = 20;
  double dt = 0.2;
```

I followed [this advice](https://discussions.udacity.com/t/mpc-cost-paramter-tuning-question/354670/2) to tune MPC parameters. I landed on the following:

```
  for (int i = 0; i < N; i++) {
    fg[0] += 3000*CppAD::pow(vars[cte_start + i], 2);
    fg[0] += 3000*CppAD::pow(vars[epsi_start + i], 2);
    fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
  }

  // State penalty
  for (int i = 0; i < N - 1; i++) {
    fg[0] += 5*CppAD::pow(vars[delta_start + i], 2);
    fg[0] += 5*CppAD::pow(vars[a_start + i], 2);
    fg[0] += 700*CppAD::pow(vars[delta_start+i] * vars[v_start+i],2);
  }

  // Control input penalty
  for (int i = 0; i < N - 2; i++) {
    fg[0] += 1200*CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
    fg[0] += 10*CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
  }
```

Importantly, you must take into account latency, which can be found in main.cpp on line 115-133. See the next section, which discusses latency.

## Latency
Here is my implementation to deal with latency.
```
  // Account for latency
  double dt        = 0.1;
  double x1        = 0;
  double y1        = 0;
  double first_psi = 0;
  double v1        = v * 0.44704;
  double Lf        = 2.67;

  x1        += v*cos(delta)*dt;
  y1        += v*sin(delta)*dt;
  first_psi += v*delta/Lf*dt;
  v1        += acc*dt;

  // Kinematic update
  cte  += v*sin(delta)*dt;
  epsi += v*delta/Lf*dt;

  Eigen::VectorXd state(6);
  state << x1, y1, first_psi, v1, cte, epsi;
```

[This post](https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391/4) specifically helped me deal with latency and implement it into my model. 

By delaying the actuator commands (using dt), you will now be accounting for latency (~100ms). As a side note, this section also converts from mph to m/s.

Next up, polynomial fitting and MPC preprocessing.

## Polynomial Fitting and MPC Preprocessing
Check out the code below:
```
  Eigen::VectorXd x_points(ptsx.size());
  Eigen::VectorXd y_points(ptsy.size());

  for (size_t i = 0; i < ptsx.size(); ++i) {
    double dx = ptsx[i]-px;
    double dy = ptsy[i]-py;

    x_points[i] = dx*cos(-psi) - dy*sin(-psi);
    y_points[i] = dx*sin(-psi) + dy*cos(-psi);
  }

  auto coeffs = polyfit(x_points, y_points, 3);

  double cte  = polyeval(coeffs, 0);
  double epsi = atan(coeffs[1]);
```

First, way points were transformed into the vehicle coordinate system. 

The third order polynomial is fitted using the transformed points here:
`auto coeffs = polyfit(x_points, y_points, 3);`

The polynomial coefficients are then used to calculate `cte` and `epsi` in the last two lines above.

## Results
The car successfully made it around the simulated track consistently. Although, I could not record a video due to hardware limitations on my 2010 Macbook Pro.

Anytime I attempted to record the screen, the car would behave erratically. Most likely due to CPU/RAM limitations.

## Resources
Shoutout to the tutorials provided by Udacity on MPC. Below are further resources and helpful links that I used to complete this project:

- [Accounting for latency](https://discussions.udacity.com/t/how-to-take-into-account-latency-of-the-system/248671/2)
- [More latency](https://discussions.udacity.com/t/calibration-for-the-acceleration-and-steering-angle-for-latency-consideration/276413)
- [Latency equations](https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391/4)
- [MPC parameter tuning](https://discussions.udacity.com/t/mpc-cost-paramter-tuning-question/354670/2)
- [Cppad brew install](https://discussions.udacity.com/t/cppad-will-not-install-mac/258710/3)
- [MPC wiki](https://en.wikipedia.org/wiki/Model_predictive_control)
- [Stanford MPC slides](https://stanford.edu/class/ee364b/lectures/mpc_slides.pdf)